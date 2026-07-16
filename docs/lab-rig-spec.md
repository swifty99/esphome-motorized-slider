# `slider_lab` — Experiment Firmware Spec (Phase 1)

The identification instrument. **It is not the controller** and must not grow into
one. It plays scripted open-loop torque profiles, records raw data deterministically,
and dumps it for analysis. One deliberate exception to "no closed loop": a crude
repositioning helper (R7) so grid experiments (M4/M7) can run unattended.

Target: ESPHome external component, ESP-IDF, on the existing rig
(ESP32-S2 class board, H-bridge on GPIO10/11 via LEDC 19.5 kHz, wiper on GPIO9/ADC1).

## Requirements

### R1 — Deterministic 1 kHz tick
`esp_timer` periodic at 1 ms. Each tick: sample ADC, evaluate experiment player,
write one record. Tick jitter and duration are themselves recorded (A9 needs a
baseline). No WiFi-dependent work in the tick path.

### R2 — Raw sensing, no filtering
ADC oneshot, 13-bit, N=4 averaged samples per tick — but the recorder stores the
**raw mean in counts** (float or ×16 fixed), and (switchable) the 4 individual
samples for noise work in M1/M2. All filtering happens offline; the lab rig must
never pre-shape the data it exists to characterize.

### R3 — Drive with selectable decay mode
Sign-magnitude output like legacy (PWM one side / LOW other) **plus** the slow-decay
recipe determined in M0 (typically PWM one side / HIGH other, inverted duty —
depends on the H-bridge truth table). Mode selectable per experiment. Duty resolution
≥10 bit at 19.5 kHz.

### R4 — Safety interlocks (non-negotiable, enforced in the tick, not in the player)
- End-zone cut: |torque| forced 0 outside position [3, 97] % unless the running
  experiment carries an `endstop_safe` flag (endpoint calibration needs it).
- Per-experiment wall-clock budget (default 30 s) → abort + torque 0.
- Thermal budget: moving average of |duty| over 10 s ≤0.5 → else abort.
- HA abort switch → torque 0 within one tick.
- Boot state: torque 0, player idle. Watchdog stays enabled.

### R5 — Experiment player
A tiny interpreter over torque-profile programs, each built from segments
`(duration_ms, mode, params)` with segment types:
`hold(duty)`, `ramp(d0→d1)`, `pulse(duty, width_ms)`, `dither(f, A, bias)`,
`breakaway_ramp(step, interval, move_threshold)` (self-terminating on motion),
`settle(ms)` (torque 0, keep recording).
Built-in parametric programs for the campaign: `noise_floor`, `pwm_coupling(duty,dir,mode)`,
`duty_ladder`, `breakaway_grid`, `step_response(duty,dir)`, `reversal(duty)`,
`impulse_ladder`, `dither_grid`. Started via HA service
`run_experiment(name, params_json)`; also exposed as buttons for the frequent ones.

### R6 — Recorder + dump
Ring buffer, one record per tick: `t_ms u32, adc_raw u16(×4 optional), duty_milli i16,
flags u8` (flags: experiment id/segment boundary/interlock events). ≥30 s capacity —
in PSRAM if present (S2 Mini: 2 MB), else ≥10 s internal RAM. Recording is armed by
experiment start and runs until buffer full or `settle` timeout after the program ends.
**Dump: HTTP GET `/lab/dump.csv`** on the device (esp_http_server; runs only outside
experiments to keep R1 honest). A `fetch.sh` in `analysis/` pulls runs into
`data/<date>/`.

### R7 — Repositioning helper (test infrastructure, explicitly non-representative)
`goto(position, tolerance=2%)`: naive bang-bang at fixed duty with generous tolerance
and its own timeout, usable only *between* program segments (never recorded as data).
Marked loudly in code as NOT-A-CONTROLLER.

### R8 — Scope sync
One GPIO (pin TBD on the rig): HIGH during an experiment program, plus a 1 ms LOW
blip at every segment boundary. Picoscope records it as a channel → sample-exact
alignment of scope captures with CSV records.

### R9 — Minimal HA surface
Position sensor at ~10 Hz (sanity only), experiment-state text sensor,
abort switch, `run_experiment` service, buttons for the campaign programs.
No OTA-triggered experiments; nothing runs without an explicit start.

## Non-requirements
No closed-loop control (except R7), no filtering, no trajectory planner, no force
feedback, no persistence of calibration. The lab rig may be ugly where the servo
must be clean — but interlocks (R4) and determinism (R1/R2) are held to full standard.

## Acceptance (Phase-1 gate)
1. `step_response(0.7, fwd)` recorded; CSV loads in Python; tick Δt histogram shows
   1000 µs ±100 µs with zero gaps.
2. Same event captured on the picoscope with the sync channel; firmware and scope
   timelines align at every segment boundary within one tick.
3. Interlock demo: end-zone cut fires during an over-long step (negative control:
   with the interlock disabled in a test build, the run visibly reaches the stop —
   proves the test isn't vacuous).
