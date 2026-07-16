- 2026-07-17 00:00 `01_breakaway_grid.csv`: Block 2 M4: breakaway grid pos 10 RERUN with move_threshold 1.5 (default 0.3 false-triggers on ADC noise, run 58 invalid), VM 5.00 V
- 2026-07-17 00:03 `02_breakaway_grid.csv`: Block 2 M4: grid pos 10 fwd x2 (thr 5.0, step 0.006, hold 150ms), VM 5.00 V
- 2026-07-17 00:04 `03_breakaway_grid.csv`: Block 2 M4: grid pos 10 rev x2 (into low zone, isolated run), VM 5.00 V
- 2026-07-17 00:04 `04_dither.csv`: Block 2 UNSTICK #8 attempt 1: +0.78/120ms out of LOW zone (2.7 %) after pos-10-rev grid trial
- 2026-07-17 00:05 `05_breakaway_grid.csv`: Block 2 M4: grid pos 10 rev repeat 2/2 (recovery run after zone latch), VM 5.00 V
- 2026-07-17 00:05 `06_dither.csv`: Block 2 UNSTICK #9 attempt 1: +0.78/120ms out of LOW zone (2.7 %) after pos-10-rev repeat
- 2026-07-17 00:06 `07_breakaway_grid.csv`: Block 2 M4: grid pos 25 both x2, VM 5.00 V
- 2026-07-17 00:07 `08_dither.csv`: Block 2 UNSTICK #10 attempt 1: +0.78/120ms out of LOW zone (2.5 %) after pos-25 rev flight
- 2026-07-17 00:07 `09_breakaway_grid.csv`: Block 2 M4: grid pos 25 both, repeat 2/2 recovery (hold_ms 50 from here on), VM 5.00 V
- 2026-07-17 00:09 `10_breakaway_grid.csv`: Block 2 M4: grid pos 40 both x2, VM 5.00 V
- 2026-07-17 00:10 `11_breakaway_grid.csv`: Block 2 M4: grid pos 55 both x2 (band), VM 5.00 V
- 2026-07-17 00:11 `12_breakaway_grid.csv`: Block 2 M4: grid pos 65 both x2 (band), VM 5.00 V
- 2026-07-17 00:12 `13_breakaway_grid.csv`: Block 2 M4: grid pos 75 rev x2 (band), VM 5.00 V
- 2026-07-17 00:13 `14_breakaway_grid.csv`: Block 2 M4: grid pos 75 fwd x2 (isolated, toward high zone), VM 5.00 V
- 2026-07-17 00:14 `15_breakaway_grid.csv`: Block 2 M4: grid pos 90 rev x2, VM 5.00 V
- 2026-07-17 00:14 `16_breakaway_grid.csv`: Block 2 M4: grid pos 90 fwd x2 (isolated, into high zone), VM 5.00 V
- 2026-07-17 00:15 `17_dither.csv`: Block 2 UNSTICK #11 attempt 1: -0.78/120ms out of high zone (97.1 %) after pos-90 fwd chunk
- 2026-07-17 00:16 `18_breakaway_grid.csv`: Block 2 M4: grid pos 75 rev extra x2 (band oversampling, total 4), VM 5.00 V
- 2026-07-17 00:17 `19_breakaway_grid.csv`: Block 2 M4: grid pos 65 rev extra x2 (band oversampling, total 4), VM 5.00 V
- 2026-07-17 00:18 `20_breakaway_grid.csv`: Block 2 M4: grid pos 55 rev extra x2 (band oversampling, total 4), VM 5.00 V

## Block 2 (campaign-runner, M4 breakaway grid, 00:3x-01:2x, VM = 5.00 V)
- DETECTOR FIX (important for anyone rerunning M4): breakaway_grid's default move_threshold 0.3 %
  false-triggers on the FIRST ramp step (5 milli, <20 ms) — raw per-tick wiper noise is ~1.2 % p-p
  idle and ~3 % p-p with drive on (PWM coupling). Threshold 1.5 still false-triggered (2026-07-16
  run 58 = junk at 0.3; 2026-07-17 run 01 = junk at 1.5, smoothed traces show zero real motion).
  Working params: move_threshold 5.0, step 0.006 (reaches 0.9 within the 3 s cap), hold_ms 50
  (150 ms hold flights hit +/-18 % and parked the knob in the end zone, losing chunk remainders —
  runs 03 trial 2, 07 trials 3-4). NOTE for the modeler: bk_duty is late-terminated (ramp stops
  ~5 % of travel after true onset, bias up to ~+0.05 at creep speeds) — refit onset from raw traces.
- Grid runs (all tickhealth structurally clean, 0 missed/dup; goto_duty 0.8 everywhere):
  02 (10 fwd x2), 03 (10 rev, trial 1 + zone latch), 05 (10 rev recovery), 07 (25 both, rep 1
  valid + zone latch), 09 (25 both recovery), 10 (40 both x2), 11 (55 both x2), 12 (65 both x2),
  13 (75 rev x2), 14 (75 fwd x2), 15 (90 rev x2), 16 (90 fwd x2, brief zone touch 73 ticks),
  18 (75 rev extra x2), 19 (65 rev extra x2), 20 (55 rev extra x2).
- Breakaway duty table (actual positions; ramp-termination duty, sign = direction):
  pos ~11 : fwd +0.708 +0.714           | rev -0.750 -0.738
  pos ~25 : fwd +0.654 +0.648           | rev -0.726 -0.720
  pos ~40 : fwd +0.696 +0.660           | rev -0.732 -0.690
  pos ~55 : fwd +0.696 +0.690           | rev -0.714 -0.720 -0.726 -0.720
  pos ~65 : fwd +0.690 +0.684           | rev -0.708 -0.714 -0.720 -0.696
  pos ~75 : fwd +0.696 +0.714           | rev -0.678 -0.678 -0.750 -0.696
  pos ~90 : fwd +0.678 +0.702           | rev -0.786 -0.732
- Findings: rev breakaway systematically ABOVE fwd by ~0.02-0.06 nearly everywhere; extremes at
  pos 90 rev (-0.786 max) and pos 25 fwd (+0.648 min). The 55-80 rev band's signature is NOT a
  higher breakaway but POOR SUSTAIN: hold_dpos after rev breakaway at 65-75 ranged -0.5 to -1.6 %
  in half the trials (vs -5..-18 % elsewhere) — motion starts, then re-sticks immediately;
  consistent with Block 1's 0.7-rev dead stop at 58.8 %. Rev scatter at 75 is the widest
  (0.678..0.750). Fwd sustains everywhere.
- UNSTICK uses (same protocol, all attempt-1 success, all interlocks live):
  #8  06_dither.csv  2.7 -> 44.9 % (+0.78) after pos-10-rev trial
  #9  08_dither? see notes lines: 2.7 -> 45.7 % (+0.78) after pos-10-rev repeat  [file 06/08 per fetch lines above]
  #10 (fetch line above) 2.5 -> 42.3 % (+0.78) after pos-25 rev flight
  #11 17_dither.csv 97.1 -> 74.6 % (-0.78) after pos-90 fwd chunk
- Rig end state: player idle, ~55 %, no reboot, no thermal ticks anywhere in the block.
  Scope: not re-checked this block beyond the once-per-block rule (still absent at Block-2 start).
- CORRECTION to the unstick list above (exact files per the fetch lines):
  #8 = `04_dither.csv` (2.7 -> 44.9 %), #9 = `06_dither.csv` (2.7 -> 45.7 %),
  #10 = `08_dither.csv` (2.5 -> 42.3 %), #11 = `17_dither.csv` (97.1 -> 74.6 %).
