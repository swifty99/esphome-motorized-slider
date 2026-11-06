# analysis/ — motorslider measurement analysis

uv-managed Python project (`slideranalysis`) for the Phase-1/2 tooling: load
`slider_lab` CSV dumps, gate-check the recorder timeline, and run the M1 noise
analysis. This scaffold grows into the identification fitting and the plant
simulator in Phase 3.

## Setup / run

Everything goes through uv (installs deps on first use):

```sh
cd analysis
uv run pytest                    # test suite
uv run fetch --host <ip[:port]> [--note "wifi off, knob 50%"]
uv run tickhealth data/2026-07-16/01_step_response.csv
uv run noise data/2026-07-16/01_noise_floor.csv [more.csv ...]
```

- `fetch` — pulls `http://<host>/lab/dump.csv` from the rig into
  `../data/<YYYY-MM-DD>/<seq>_<experiment>.csv`. `<seq>` is the next free
  2-digit number in that day's folder, the experiment name comes from the
  dump's meta lines. `--note` appends a timestamped line to `notes.md` in the
  same folder (one notes.md per bench session/day, per campaign protocol).
- `tickhealth` — Phase-1 gate: delta-t histogram from `t_ms`, structural
  defect count, delta-t jitter stats, `tick_us` (work duration) p50/p95/p99.
  PASS = zero structural defects (missed: Δt ≥ 1500 µs; duplicated:
  Δt ≤ 500 µs) AND Δt jitter max ≤ 250 µs AND tick_us p99 ≤ 900 µs.
  Saves `<csv>_tickhealth.png`, prints the verdict, exit code 1 on FAIL.
- `noise` — M1: RMS + peak-to-peak in counts and % of travel, intra-tick
  sample spread (when s0..s3 are recorded), Welch PSD (fs = 1 kHz) with
  labeled peaks. Multiple CSVs overlay in one figure (e.g. WiFi on vs off).

Library use: `from slideranalysis import load_run` → `(meta, df)`.

## Data folder convention

```
data/<YYYY-MM-DD>/
    01_noise_floor.csv       # lab-rig recordings, fetched via `uv run fetch`
    02_pwm_coupling_0.2.csv
    03_step_response.csv
    notes.md                 # session notes, --note appends here
    *.npz                    # PicoScope exports land next to the CSVs
```

PicoScope captures come from the **separate** repo
`/Users/jan/dev/repos/pico-bench` (x86/Rosetta venv, Python 3.9). Do **not**
depend on it or import it from here — its `.npz` exports are just files that
land in the same `data/<date>/` folder as the lab CSVs.

## CSV contract (fixed — firmware is built against the same one)

- `#`-prefixed meta lines first, `# key: value` (first colon splits). Values
  are JSON-parsed when possible (`params` becomes a dict, `tick_us_nominal`
  a number); `#` lines without a colon end up in `meta["comments"]`.
  Keys: `firmware`, `experiment`, `params`, `tick_us_nominal`, `adc`.
- Header: `t_ms,adc_raw,duty_milli,seg,flags,tick_us` plus optional
  `s0,s1,s2,s3` when the firmware's raw-sample mode is on.
- `adc_raw` = mean of 4 ADC samples (float, counts); `duty_milli` = signed
  applied torque ×1000; `seg` = segment index; `tick_us` = measured tick
  **work duration** in µs (the tick budget is 1000 µs).
- `flags` bitfield: 1=segment_start, 2=interlock_endzone, 4=interlock_thermal,
  8=abort, 16=goto_active — the loader decodes these into boolean columns
  (`segment_start`, `ilk_endzone`, `ilk_thermal`, `abort`, `goto_active`).
- Percent of travel (provisional, until endpoint self-cal): raw 8000 → 0 %,
  raw 100 → 100 % (pot inverted), linear in between → 79 counts per % travel.
  Helpers: `raw_to_percent()` (position), `counts_to_percent()` (amplitudes).

Resolutions taken here that the firmware side must match:

- **Meta line syntax** is exactly `# key: value`; the params line is
  `# params: {...json...}`.
- **tickhealth timeline gate** separates two failure classes (calibrated on
  5 real quiet-mode hardware runs, 52k intervals, worst jitter ≈ 115 µs):
  *structural* — missed = Δt ≥ 1500 µs (1.5 periods, counted as
  `round(dt/1000) − 1`), duplicated/non-monotonic = Δt ≤ 500 µs; both must be
  zero. Everything in between is *jitter* |Δt − 1000|, reported as p50/p99/max
  and gated at max ≤ 250 µs (quarter period, `--jitter-limit-us`). A tick
  101 µs late is jitter, not a miss.
- **tick_us** is the tick *work duration* (firmware-confirmed). Default gate:
  p99 ≤ 900 µs — guard band under the 1000 µs budget, computed as
  nominal − tol. The period interpretation is kept behind
  `tickhealth --tick-us-period` (band check 1000 ± 100 µs on p50/p95/p99).

## Layout

```
src/slideranalysis/
    loadrun.py     # CSV loader: meta dict + DataFrame, flags decode, % helpers
    tickhealth.py  # Phase-1 gate check + figure + verdict
    noise.py       # M1: RMS/p-p, PSD, multi-run compare
    fetch.py       # HTTP fetch from the rig into data/<date>/
tests/
    synth.py       # synthetic-CSV fixture generator (same contract)
    test_*.py      # incl. negative controls (seeded 3 ms gap must FAIL)
```

Style: plain engineering plots, matplotlib defaults, axes labeled with units.
Keep functions pure where possible — Phase 3 (identification fits, plant
simulator) builds directly on these modules.
