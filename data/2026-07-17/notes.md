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
- 2026-07-17 00:23 `21_reversal.csv`: Block 3 M6: reversal pos 25 fwd c1 (approach 0.68 creep, reverse 0.75), VM 5.00 V
- 2026-07-17 00:24 `22_dither.csv`: Block 3 UNSTICK #12 attempt 1: -0.78/120ms out of high zone (97.4 %) after reversal run 21
- 2026-07-17 00:25 `23_reversal.csv`: Block 3 M6: REVERSAL pos 50 (start 70.6 above, goto arrives bwd, reverse +0.75 fwd; dummy approach 0.3), VM 5.00 V
- 2026-07-17 00:25 `24_dither.csv`: Block 3 UNSTICK #13 attempt 1: -0.78/120ms out of high zone (98.2 %) after M6 run A
- 2026-07-17 00:26 `25_reversal.csv`: Block 3 M6: CONTROL pos 50 (start 66.6 above, goto arrives bwd, reverse -0.75 bwd = same dir), VM 5.00 V
- 2026-07-17 00:26 `26_dither.csv`: Block 3 UNSTICK #14 attempt 1: +0.78/120ms out of LOW zone (2.6 %) after M6 run B
- 2026-07-17 00:27 `27_reversal.csv`: Block 3 M6: REVERSAL pos 65 BAND (start 49.8 below, goto arrives fwd, reverse -0.75 bwd), VM 5.00 V
- 2026-07-17 00:27 `28_dither.csv`: Block 3 UNSTICK #15 attempt 1: +0.78/120ms out of LOW zone (2.5 %) after M6 run C
- 2026-07-17 00:28 `29_reversal.csv`: Block 3 M6: CONTROL pos 65 BAND (start 49.6 below, goto arrives fwd, reverse +0.75 fwd = same dir), VM 5.00 V
- 2026-07-17 00:28 `30_dither.csv`: Block 3 UNSTICK #16 attempt 1: -0.78/120ms out of high zone (98.1 %) after M6 run D
- 2026-07-17 00:29 `31_reversal.csv`: Block 3 M6: REVERSAL pos 25 (start 65.2 above, goto arrives bwd, reverse +0.75 fwd), VM 5.00 V
- 2026-07-17 00:29 `32_dither.csv`: Block 3 UNSTICK #17 attempt 1: -0.78/120ms out of high zone (98.1 %) after M6 run E
- 2026-07-17 00:30 `33_reversal.csv`: Block 3 M6: CONTROL pos 25 (start 63.9 above, goto arrives bwd, reverse -0.75 bwd = same dir), VM 5.00 V
- 2026-07-17 00:30 `34_dither.csv`: Block 3 UNSTICK #18 attempt 1: +0.78/120ms out of LOW zone (2.2 %) after M6 run F
- 2026-07-17 00:31 `35_reversal.csv`: Block 3 M6: REVERSAL pos 75 (start 50.0 below, goto arrives fwd, reverse -0.75 bwd), VM 5.00 V
- 2026-07-17 00:32 `36_dither.csv`: Block 3 UNSTICK #19 attempt 1: +0.78/120ms out of LOW zone (2.4 %) after M6 run G
- 2026-07-17 00:32 `37_reversal.csv`: Block 3 M6: CONTROL pos 75 (start 49.7 below, goto arrives fwd, reverse +0.75 fwd = same dir), VM 5.00 V
- 2026-07-17 00:33 `38_dither.csv`: Block 3 UNSTICK #20 attempt 1: -0.78/120ms out of high zone (98.1 %) after M6 run H
- 2026-07-17 00:36 `39_impulse_ladder.csv`: Block 3 M7: impulse ladder fwd, duty 0.8, widths 10-80ms x3, recenter q3, VM 5.00 V
- 2026-07-17 00:37 `40_dither.csv`: Block 3 UNSTICK #21 attempt 1: -0.78/120ms out of high zone (98.0 %) after M7 fwd run
- 2026-07-17 00:38 `41_impulse_ladder.csv`: Block 3 M7: impulse ladder fwd FINE widths 2-8ms x3, duty 0.8, VM 5.00 V
- 2026-07-17 00:39 `42_impulse_ladder.csv`: Block 3 M7: impulse ladder bwd widths 3-30ms x3, duty 0.8, VM 5.00 V
- 2026-07-17 00:40 `43_impulse_ladder.csv`: Block 3 M7: impulse ladder bwd 50ms x3, recenter every shot, VM 5.00 V

## Block 3 (campaign-runner, M6 reversal + M7 impulse ladder, 01:3x-02:2x, VM = 5.00 V)
- M6 DESIGN NOTE: reversal's hard-coded 600 ms approach hold cannot creep at these breakaways —
  approach 0.68 fwd FLEW 26->97.4 % in <600 ms (`21_reversal.csv`, discarded as evidence).
  Workaround: dummy approach 0.3 (sub-breakaway, no motion), reverse 0.75, cycles 1 per run,
  backlash seated by the goto's arrival direction (start side chosen before each run);
  every reverse flight parks in an end zone -> 1 unstick per run (all logged below).
- M6 runs (all tickhealth clean; reverse segment starts mid-travel, zone cut only in the flight tail):
  `23_` pos50 rev=fwd | `25_` pos50 rev=bwd | `27_` pos65 rev=bwd (band) | `29_` pos65 rev=fwd (band)
  `31_` pos25 rev=fwd | `33_` pos25 rev=bwd | `35_` pos75 rev=bwd | `37_` pos75 rev=fwd
  (arrival-seated "reversal" vs "control" pairs at each position; goto overshoot makes the seating
  direction approximate — traces contain the full history for the modeler.)
- M6 RESULT: dead time at +/-0.75 reversal is 6-9 ms UNIFORMLY (0.5 % threshold, pre-segment
  baseline; 4 positions x both dirs; no difference between seated-opposite and seated-same runs).
  Tick-level traces (23, 27): motion begins ~6 ms after torque application, smooth acceleration.
  => No resolvable backlash plateau at 1 kHz/0.75; upper bound ~8 ms. KEY: breakaway ~1 s after
  prior motion is nearly INSTANT vs 0.65-0.79 duty needed from rest (Block 2) — stiction is
  strongly dwell-time-dependent. (Earlier threshold-artifact numbers 22 ms fwd / 0 ms bwd were
  travel-time bias — corrected with proper baseline.)
- M7 impulse ladder, duty 0.8, settle 400 ms, recenter goto_duty 0.8:
  `39_` fwd coarse [10,20,30,50,80]x3 q3 from 45: WARN — 3rd 50 ms shot clipped the high zone,
  80 ms group lost to zone-cut gotos (data 10-50 ms valid). `41_` fwd fine [2,3,5,8]x3: PASS.
  `42_` bwd [3,5,10,20,30]x3 from 50: PASS. `43_` bwd [50]x3 recenter-every-shot from 60: PASS.
- M7 NUDGE TABLE (median displacement per pulse width, duty 0.8, mid-travel):
  width   fwd        bwd
   2 ms  +0.06 %      --
   3 ms  +0.20 %    -0.27 %
   5 ms  +0.56 %    -0.62 %
   8 ms  +1.75 %      --
  10 ms  +2.64 %    -1.04 %
  20 ms  +6.20 %    -4.49 %
  30 ms  +10.3 %    -8.14 %
  50 ms  +21.7 %    -20.1 %
  Sub-1 % nudges live at 3-5 ms (both dirs). Directional asymmetry: fwd ~2x more effective than
  bwd at 8-30 ms, converging by 50 ms. Repeat scatter is small (e.g. bwd 50 ms: 19.8/20.1/20.2).
  NOTE: shots fire ~0.4-1 s after prior motion -> warm-stiction regime (consistent with M6);
  cold nudges (long dwell) may differ — morning follow-up if wanted (cold_reversal or longer settles).
- UNSTICK uses #12-#21 (10 total, all attempt-1 success, |bias| 0.78, 120 ms, all interlocks live):
  #12 `22_` 97.4->70.6 | #13 `24_` 98.2->66.6 | #14 `26_` 2.6->49.8 | #15 `28_` 2.5->49.6
  #16 `30_` 98.1->65.2 | #17 `32_` 98.2->63.9 | #18 `34_` 2.2->50.0 | #19 `36_` 2.4->49.7
  #20 `38_` 98.1->66.0 | #21 `40_` 98.0->67.8
- Scope: still off USB at block start (VID 0x0CE9 absent, checked once). No thermal ticks all block.
- 2026-07-17 00:43 `44_step_response.csv`: Block 4: reposition to 20 % (fine ladder fwd runway)
- 2026-07-17 00:44 `45_duty_ladder.csv`: Block 4: FINE duty ladder fwd 0.60-0.70/0.02 from ~21 %, VM 5.00 V
- 2026-07-17 00:45 `46_dither.csv`: Block 4 UNSTICK #22 attempt 1: -0.78/120ms out of high zone (97.9 %) after fwd fine ladder
- 2026-07-17 00:45 `47_step_response.csv`: Block 4: reposition to 80 % (fine ladder rev runway)
- 2026-07-17 00:46 `48_duty_ladder.csv`: Block 4: FINE duty ladder rev 0.60-0.74/0.02 from ~80 %, VM 5.00 V
- 2026-07-17 00:46 `49_dither.csv`: Block 4 UNSTICK #23 attempt 1: +0.78/120ms out of LOW zone (2.9 %) after rev fine ladder
- 2026-07-17 00:48 `50_dither_grid.csv`: Block 4 M8: dither grid 100Hz/0.10 pos 50 fwd (with baseline), VM 5.00 V
- 2026-07-17 00:49 `51_dither.csv`: Block 4 UNSTICK #24 attempt 1: -0.78/120ms out of high zone (97.9 %) after dither_grid 50
- 2026-07-17 00:50 `52_dither_grid.csv`: Block 4 M8: dither grid 100Hz/0.10 pos 20 fwd RERUN (50 lost dithered half to zone), VM 5.00 V
- 2026-07-17 00:51 `53_dither_grid.csv`: Block 4 M8: dither grid 100Hz/0.20 pos 20 fwd, VM 5.00 V
- 2026-07-17 00:52 `54_dither_grid.csv`: Block 4 M8: dither grid 200Hz/0.10 pos 20 fwd, VM 5.00 V
- 2026-07-17 00:53 `55_dither_grid.csv`: Block 4 M8: dither grid 100Hz/0.10 pos 65 BAND rev, VM 5.00 V
- 2026-07-17 00:54 `56_dither_grid.csv`: Block 4 M8: dither grid 100Hz/0.20 pos 65 BAND rev, VM 5.00 V
- 2026-07-17 00:55 `57_pwm_coupling.csv`: Block 4 M2-CSV: coupling fast duty 0.1, 10s stall, VM 5.00 V
- 2026-07-17 00:56 `58_pwm_coupling.csv`: Block 4 M2-CSV: coupling fast duty 0.2, 10s stall, VM 5.00 V
- 2026-07-17 00:57 `59_pwm_coupling.csv`: Block 4 M2-CSV: coupling fast duty 0.3, 10s stall, VM 5.00 V
- 2026-07-17 00:58 `60_pwm_coupling.csv`: Block 4 M2-CSV: coupling fast duty 0.4, 10s stall, VM 5.00 V
- 2026-07-17 00:59 `61_pwm_coupling.csv`: Block 4 M2-CSV: coupling fast duty 0.5, 9s stall, VM 5.00 V
- 2026-07-17 00:59 `62_pwm_coupling.csv`: Block 4 M2-CSV: coupling slow duty 0.1, 10s stall, VM 5.00 V
- 2026-07-17 01:00 `63_pwm_coupling.csv`: Block 4 M2-CSV: coupling slow duty 0.2, 10s stall, VM 5.00 V
- 2026-07-17 01:01 `64_pwm_coupling.csv`: Block 4 M2-CSV: coupling slow duty 0.3, 10s stall, VM 5.00 V
- 2026-07-17 01:02 `65_pwm_coupling.csv`: Block 4 M2-CSV: coupling slow duty 0.4, 10s stall, VM 5.00 V
- 2026-07-17 01:03 `66_pwm_coupling.csv`: Block 4 M2-CSV: coupling slow duty 0.5, 9s stall, VM 5.00 V

## Block 4 (campaign-runner, FINAL measurement block: fine ladders + M8 dither + M2-CSV coupling, 02:3x-03:2x, VM = 5.00 V)
- FINE DUTY LADDERS (0.02 steps, step_ms 800, pause 700, mode fast; duty -> velocity map):
  `45_duty_ladder.csv` FWD from ~21 %: 0.60 +3.4 %/s (first rung, warm from goto) | 0.62 +0.36 |
  0.64 +0.88 | 0.66 +0.91 | 0.68 +127 %/s FLIES | 0.70 rung lost (zone park 97.9, WARN).
  `48_duty_ladder.csv` REV from ~80 %: 0.60 -0.72 | 0.62 -0.40 | 0.64 -0.26 | 0.66/0.68 DEAD
  STALL (0.0) | 0.70 -91 %/s FLIES (79.5 -> 6.6) | 0.72/0.74 zone-cut (WARN).
  => The stall->fly transition is a cliff, ~0.66->0.68 fwd and ~0.68->0.70 rev; the creep window
  is <= 1 %/s wide in duty terms. Feedforward design implication: there is effectively NO usable
  proportional creep regime; position control must be impulse-based (see M7 nudge table).
- M8 DITHER (dither_grid with Block-2 detector recipe thr 5.0 / step 0.006; square dither; each
  run has its own no-dither baseline; "dithered base" = ramp base duty at breakaway, i.e. max-amp):
  pos 20 fwd 100 Hz/0.10: baseline 0.648 -> base 0.576 (delta 0.072)   [`52_dither_grid.csv`]
  pos 20 fwd 100 Hz/0.20: baseline 0.696 -> base 0.474 (delta 0.222)   [`53_`]
  pos 20 fwd 200 Hz/0.10: baseline 0.672 -> base 0.576 (delta 0.096)   [`54_`]
  pos 65 rev 100 Hz/0.10: baseline 0.690 -> base 0.624 (delta 0.066)   [`55_`]
  pos 65 rev 100 Hz/0.20: baseline 0.666 -> base 0.510 (delta 0.156)   [`56_`]
  => LAW: breakaway happens when base+amp reaches the undithered breakaway (peaks 0.674-0.724
  vs baselines 0.648-0.696) — dither reduces required BASE duty by ~its amplitude, no
  super-additive effect at 100/200 Hz. Objective part done; audibility = Jan's morning item.
  `50_dither_grid.csv`: first attempt at pos 50 fwd — baseline hold flight parked 97.9, dithered
  half zone-cut (only baseline 0.696@51 valid); rerun from pos 20 = `52_`. WARN on 50 only.
- M2-CSV COUPLING GRID (`57_`-`66_pwm_coupling.csv`, stall at 59.7 %, 10/9 s, all PASS):
  wiper adc p-p spread [counts]: FAST 0.1/0.2/0.3/0.4/0.5 = 220/218/200/224/308
                                 SLOW 0.1/0.2/0.3/0.4/0.5 = 253/249/286/326/415
  (idle noise floor reference: 93-100 counts, Block 1). SLOW decay couples markedly more noise
  into the wiper and grows with duty; FAST is flat until it jumps at 0.5. Both modes complete.
- Repositioning: `44_`, `47_step_response.csv` (zero-duty gotos to 20/80). UNSTICK uses
  #22 `46_dither.csv` 97.9 -> 66.4 | #23 `49_` 2.9 -> 51.3 | #24 `51_` 97.9 -> 66.7
  (all attempt-1 success, protocol unchanged).
- All runs tickhealth structurally clean (0 missed / 0 dup); zero thermal ticks; no reboot all
  night. Scope: still off USB at block start (VID 0x0CE9 absent — morning replug item).
- Rig end state: player idle, position 59.5 %, mid-travel — left quiet as instructed.
