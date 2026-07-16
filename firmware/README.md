# `slider_lab` firmware — Phase-1 lab rig

The identification instrument (spec: `../docs/lab-rig-spec.md`, R1–R9). Plays
scripted open-loop torque profiles at a deterministic 1 kHz tick, records raw
data, dumps CSV over HTTP. **It is not the controller.**

Hardware (fixed): ESP32-S2 Mini (`lolin_s2_mini`, 2 MB PSRAM) — pot wiper on
GPIO9 (ADC1 ch8, 13-bit, ATTEN_DB_12, 4 oneshot samples/tick), DRV8833
AIN1=GPIO11 (forward side) / AIN2=GPIO10 (LEDC 19531 Hz, 12-bit), scope-sync
on GPIO12 → PicoScope **EXT trigger**.

## Build / flash

```sh
# secrets.yaml (gitignored) needs wifi_ssid / wifi_password
esphome config  slider_lab_rig.yaml
esphome compile slider_lab_rig.yaml
esphome run     slider_lab_rig.yaml     # flash — only with Jan's OK (house rule)
```

Any ESPHome ≥2026.7 works; the EsphomeZoo venv has one:
`/Users/jan/dev/repos/EsphomeZoo/.venv/bin/esphome`.

## Running experiments

Experiments start **only** on explicit command (R9): the HA service or the
per-program buttons. Nothing runs at boot; boot state is torque 0, player idle.

HA service: `esphome.slider_lab_run_experiment` with `name` + `params_json`
(empty string = protocol defaults). Examples, one per campaign program:

| Program (M#) | Example call |
|---|---|
| `noise_floor` (M1) | `{"duration_s": 10}` — motor unpowered; raw-sample mode auto-on |
| `pwm_coupling` (M2) | `{"duty": 0.2, "dir": "fwd", "mode": "fast"}` — duty clamped ≤0.35 (stays below breakaway); raw-sample mode auto-on |
| `duty_ladder` (M3) | `{"dir": "bwd", "mode": "slow"}` — 0.05…0.6 in 0.05 steps, 2 s each, 0.5 s settles between rungs |
| `breakaway_grid` (M4) | `{"positions": [5, 27.5, 50, 72.5, 95], "repeats": 1, "dir": "both"}` |
| `step_response` (M5) | `{"duty": 0.7, "dir": "fwd"}` — starts at 20 % (fwd) / 80 % (bwd), holds until the end-zone interlock cuts |
| `triangle` (M5 quasi-static) | `{"peak": 0.6, "ramp_ms": 4000, "dir": "fwd"}` — extra program, not in the spec list |
| `reversal` (M6) | `{"duty": 0.45, "position": 50, "cycles": 3, "dir": "fwd"}` |
| `impulse_ladder` (M7) | `{"position": 50, "repeats": 5}` — variant B: add `"cold_reversal": true` |
| `dither_grid` (M8) | `{"freq": 100, "amp": 0.1}` — ONE grid point per run (ear test is the instrument); `baseline: true` (default) prepends an undithered breakaway ramp |
| `dither` (bare R5 segment) | `{"freq": 100, "amp": 0.1, "bias": 0.2, "duration_s": 5}` |

Common params for every program: `mode` (`fast`/`slow` decay, default fast),
`raw` (force raw-sample mode), `endstop_safe` (bypasses the end-zone interlock
— logged loudly; also the runtime negative-control for acceptance test 3).

**Campaign chunking:** the recorder holds ~75 s at 1 kHz. Grid defaults are
sized to fit one buffer. Full M4 (10 pos × 2 dir × 3 rep) = run chunks, e.g.
3 runs of `{"positions": [5,15,25,35,45], "repeats": 1}` / `[55,65,75,85,95]`
etc., pooling repeats across runs. Full M7 = one run per position
(25/50/75), per direction with `"repeats": 10, "dir": "fwd"`.

## CSV dump

```sh
curl -o run.csv http://slider-lab.local/lab/dump.csv
```

- Refused with **503 while an experiment is running/pending** (R1 stays
  honest). Conversely, a `run_experiment` during a dump is queued/refused.
- `#` meta lines: firmware version/date, experiment name, params JSON, nominal
  tick_us, ADC config, record/capacity/storage, aborted flag, flags legend.
- Columns: `t_ms,adc_raw,duty_milli,seg,flags,tick_us` (+ `s0,s1,s2,s3` when
  raw-sample mode was on for that run).
  - `t_ms`: ms since experiment start **with 3 decimals** (µs resolution — the
    R1 jitter histogram needs it; stored internally as µs u32).
  - `adc_raw`: mean of the 4 raw samples, 2 decimals, unfiltered.
  - `duty_milli`: signed torque ×1000 **as actually applied** (after interlocks).
  - `flags`: 1=segment_start, 2=interlock_endzone, 4=interlock_thermal,
    8=abort, 16=goto_active. **Drop flag-16 ticks in analysis** — the
    repositioning helper is not experiment data.
  - `tick_us`: tick execution time (ADC+player+interlocks+apply).
- Recording is armed at experiment start, runs until buffer full or ~500 ms
  after the program ends. Buffer: 75 000 records (1.8 MB) in PSRAM, ladder
  down to 30 000 if allocation fails; internal-RAM fallback (12 s → 2 s, with
  a warning) if PSRAM is absent — "Lab Recording Full" binary sensor + log
  tell you when a run outgrew the buffer.

## Sync pin (R8)

GPIO12 → PicoScope EXT trigger: **LOW when idle**, **HIGH while a program
runs**, **1 ms LOW blip at every segment boundary**. The rising edge marks
t=0 of the run; blips align scope captures with CSV `seg` transitions to one
tick. The post-run 500 ms recording tail happens with sync already LOW.

## Safety interlocks (R4 — in the tick, not in the player)

| Interlock | Behavior |
|---|---|
| End-zone | torque forced 0 while raw-scaled position is outside [3, 97] % (flag 2), unless the run carries `endstop_safe`. Cut, not abort — step_response uses the cut as data. |
| Wall-clock budget | planned duration +25 % +5 s, floor 30 s, hard cap 15 min → abort (flag 8) |
| Thermal | 10 s moving average of applied \|duty\| ≤ 0.5, else abort (flags 4+8) |
| Abort switch | "Lab Abort" in HA → torque 0 within one tick (flag 8); player refuses to start while ON |
| Boot | torque 0 (both LEDC outputs 0 = coast, silent), player idle |

Torque-0 semantics: idle/settle/interlock cuts are always hard-off (both pins
low, coast, silent). Inside an active SLOW-decay segment, commanded 0 uses the
slow-decay recipe (both high = brake) as per R3.

## Repositioning helper (`goto`, R7)

Bang-bang at fixed duty 0.55, tolerance 2 %, timeout 4 s — **NOT a
controller**, used only between segments of grid programs. Its ticks carry
flag 16. If goto consistently times out ("goto timed out" warning), the drive
direction is probably reversed on the rig: set `invert_direction: true` under
`slider_lab:` in the YAML. Corner case: if a goto overshoots past 97 %/below
3 %, the end-zone interlock also blocks the way back (spec-literal) until the
goto times out — grid positions default to ≤95 % to keep margin.

## HA surface (R9)

- `sensor.slider_position` — ~10 Hz, provisional scaling raw 8000→0 %,
  100→100 % (inverted wiring)
- `sensor.lab_tick_time` — diagnostic, last tick execution µs
- `text_sensor.lab_player_state` — `idle` / `running:<name>:<segment>` /
  `settling:<name>`
- `switch.lab_abort`, `switch.lab_raw_sample_mode`
- `binary_sensor.lab_recording_full`
- service `run_experiment(name, params_json)` + one button per campaign program
