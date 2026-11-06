# servo-hil — HIL tuning tools for slider_servo

Bench instruments for the deep-dive tuning loop (docs/handover-tuning-2026-07-18.md).
All talk to the servo over the ESPHome native API / HTTP dump. Run from
`analysis/` so they use its venv (aioesphomeapi):

```
cd analysis
uv run python servo-hil/servo_status.py            # state + A9 + reset reason (retries through a reboot)
uv run python servo-hil/servo_move.py 70 0 12      # one move to 70 %, fast; watch live
uv run python servo-hil/servo_move.py 60 15 18     # one CRAWL to 60 % at 15 %/s
uv run python servo-hil/a1_sweep.py 40 1           # A1 scorecard, 40 moves, seed 1 (the metric)
```

Host defaults to `192.168.178.115`; override with `SERVO_HOST=...`.

## The loop

1. **Tune** — a YAML knob in `firmware/slider_lab_rig.yaml` (see the handover for
   the exposed set) or a core strategy change.
2. **Flash** — `cd firmware && esphome compile slider_lab_rig.yaml && esphome upload slider_lab_rig.yaml --device 192.168.178.115` (needs Jan's OK; no allow-rules).
3. **Measure** — `a1_sweep.py 40 1` (same seed = same 40 moves as the baseline).
4. **Compare** — against the baseline scorecard below.
5. **Commit** if strictly better on the axis you targeted without regressing the
   others; else revert and try the next strategy.

Crawl acoustics: fetch a trace and characterize the pulse cadence:
```
curl http://192.168.178.115/servo/dump.csv -o /tmp/c.csv
python servo-hil/crawl_analyze.py /tmp/c.csv
```

## Baseline scorecard (firmware commit e28f965, seed 1) — the bar

```
all  gave_up 0   overshoot 5%   max|err| 0.43%   out-of-tol 12%   max pulses 3   settle med 0.7s
fwd  overshoot 0%
rev  overshoot 11%
```

The ~12 % out-of-tol is the observer-vs-sensor ~0.3 % offset (a calibration/
auto-ident axis), not endgame bounce — don't chase it with endgame knobs.
