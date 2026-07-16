- 2026-07-16 18:27 `01_noise_floor.csv`: gate run: quiet noise_floor (no API clients during recording), fw with integer-formatter dump fix, USB-powered from Mac, no scope
- 2026-07-16 18:41 `02_noise_floor.csv`: v3 gate evidence (GPTimer + tick task prio 24): 5 quiet runs, 52490 intervals, 0 structural missed/dup, max jitter 115us, tick_us p99 418us
- 2026-07-16 19:42 `03_pwm_coupling.csv`: session 1: pwm_coupling 0.2 fwd fast for alignment proof, knob ~38.5%, rig on PSU
- 2026-07-16 19:55 `05_pwm_coupling.csv`: alignment run captured on scope as 04_alignment_scope.npz (trigger B shunt 25mV rising)
- 2026-07-16 20:10 `07_triangle.csv`: stall-verification triangle run, scope saw flat shunt
- 2026-07-16 20:37 `09_step_response.csv`: shunt check run: scope 08, wiper twitch only
- 2026-07-16 20:43 `11_noise_floor.csv`: preflight actuation blip
- 2026-07-16 20:48 `12_dither.csv`: labctl: endstop_safe nudge out of the end zone before preflight
- 2026-07-16 20:48 `13_dither.csv`: preflight actuation blip
- 2026-07-16 21:54 `15_dither.csv`: preflight actuation blip
- 2026-07-16 22:36 `17_dither.csv`: preflight actuation blip
- 2026-07-16 22:43 `23_dither.csv`: coder: endstop_safe nudge out of the end zone before pwm_coupling verification
- 2026-07-16 22:44 `24_reversal.csv`: coder: reposition to mid-travel (also 2nd goto direction check)
- 2026-07-16 22:45 `25_dither.csv`: coder: -0.85 blip to unstick from high-end zone
- 2026-07-16 22:45 `26_dither.csv`: coder: endstop_safe nudge out of bottom end zone
- 2026-07-16 22:46 `27_pwm_coupling.csv`: coder verification: new 0.5 clamp, mid-travel stall check
- 2026-07-16 22:48 `29_reversal.csv`: coder: goto_duty param end-to-end check
- 2026-07-16 23:02 `30_dither.csv`: preflight actuation blip
- 2026-07-16 23:22 `31_pwm_coupling.csv`: Block 0: alignment run CSV-only (PicoScope off USB bus, openUnit 0x3, no capture possible), VM 5.00 V

## Block 0 (campaign-runner, night session, 23:00-23:25, VM = 5.00 V)
- `labctl status`: player idle, pos 51 %, no reboot — OK to start.
- `labctl preflight --scope`: PREFLIGHT FAIL, but only on the scope gate.
  - [1 connectivity] PASS (api 0.2 s, http 200, player idle)
  - [2 position] PASS 51.0 % (std 0.16 %)
  - [3 actuation] PASS: blip moved 7.11 % (peak duty 700 milli, no interlocks) [`30_dither.csv`]
  - [4 scope] FAIL: capture.py never armed — `ps3000aOpenUnit failed: 0x3` (PICO_NOT_FOUND).
- PicoScope 3203D is OFF THE USB BUS entirely (ioreg: Pico VID 0x0CE9 absent; no process holds it;
  last successful capture was `08_shunt_check_scope.npz` at 20:35). Polled re-enumeration for 3 min +
  retried openUnit at 23:21 — still gone. Needs a physical replug; nobody at the bench.
  Consequence: co-swing check (4a) and shunt check (4b) NOT verified tonight; EXT trigger still
  hardware-unvalidated; Phase-1 scope-alignment gate NOT closed.
- Alignment run executed CSV-only: `31_pwm_coupling.csv` (duty 0.35, 10 s, mid-travel 44.3 %).
  VERDICT PASS — 11099 records, duty 0->350 milli for exactly 10000 ticks, 3 segments,
  0 interlocks, position held 44.3 % -> 44.3 % (true stall), tick health green
  (0 missed/dup, jitter p99 42 us, tick_us p99 451 us). No npz — no scope.
- Campaign recommendation: NO-GO for scope in M1/M2/M3 until the scope is replugged;
  CSV-only blocks remain valid.
- 2026-07-16 23:28 `32_noise_floor.csv`: Block 1 M1: noise floor pt 1/3, mid-travel, VM 5.00 V
- 2026-07-16 23:29 `33_step_response.csv`: Block 1: zero-duty goto reposition to 20 %
- 2026-07-16 23:29 `34_noise_floor.csv`: Block 1 M1: noise floor pt 2/3, low-travel ~22 %, VM 5.00 V
- 2026-07-16 23:30 `35_step_response.csv`: Block 1: zero-duty goto reposition to 80 %
- 2026-07-16 23:30 `36_noise_floor.csv`: Block 1 M1: noise floor pt 3/3, high-travel ~80 %, VM 5.00 V
- 2026-07-16 23:30 `37_step_response.csv`: Block 1: zero-duty goto reposition to 13 % (ladder runway)
- 2026-07-16 23:31 `38_duty_ladder.csv`: Block 1: duty ladder FAST mode, fwd from ~15 %, rungs 0.05-0.70, VM 5.00 V
- 2026-07-16 23:33 `39_step_response.csv`: Block 1: zero-duty goto reposition to 13 % (slow ladder runway)

## Block 1 (campaign-runner, CSV-only, 23:2x-23:4x, VM = 5.00 V)
- M1 noise floor (raw samples, 10 s each, motor unpowered) — COMPLETE, all VERDICT PASS, tickhealth
  structurally clean (0 missed / 0 dup), jitter p99 53-54 us, tick_us p99 421-424 us:
  - `32_noise_floor.csv` @ 44.2 % (adc 4456..4550, spread 93 counts)
  - `34_noise_floor.csv` @ 21.8 % (adc 6224..6325, spread 100 counts)
  - `36_noise_floor.csv` @ 79.5 % (adc 1670..1766, spread 96 counts)
  Repositioning via zero-duty step_response gotos: `33_` (44->21.9 %), `35_` (21.9->79.5 %).
- Duty ladder FAST (`38_duty_ladder.csv`, fwd from 15.1 %, d0 0.05 d1 0.70 step 0.05,
  step_ms 1000, pause 800): VERDICT WARN (endzone 632 ticks at the very end), tickhealth PASS.
  Per-rung result (position 15 %, fwd): duty <= 0.60 -> stall (|dpos| <= 0.24 %);
  0.65 -> creep +4.4 %/s; 0.70 -> +211 %/s, traversed 19.6 -> 97.8 % in <0.5 s, endzone cut,
  momentum carried the knob to 98.0 % INSIDE the end zone. Sharp breakaway between 0.60 and 0.65,
  violent 0.65->0.70 nonlinearity at this position — prime modeler input.
- STUCK: slider parked at 98.0 % (> kEndzoneHi 97). The endzone interlock cuts ALL torque there
  (slider_lab.cpp:376, direction-agnostic), so no non-endstop_safe program can move it —
  verified: `37_step_response.csv` (goto to 13) burned its 4 s timeout with zero movement.
  The established unstick (endstop_safe nudge, used earlier tonight: runs 12/25/26) is FORBIDDEN
  by the night-session rules ("No endstop_safe experiments"), so campaign-runner did not use it.
- PARKED (blocked, not attempted): duty ladder SLOW mode; step_response 0.6/0.7/0.8 fwd+rev;
  triangle fast/slow. NOTE: these would very likely re-park the slider in an end zone anyway
  (M5's end-zone cut is "part of the data" per firmware); the campaign design assumes an
  unstick capability that the night rules forbid. Needs a coordinator/Jan decision:
  authorize a minimal endstop_safe nudge (~150 ms, duty ~0.8, away from the end) or free it
  by hand in the morning.
- Anomaly for the modeler: breakaway at 15 % needed >= 0.65 duty; bench earlier saw 0.55 creep /
  0.7 fly at mid-travel — consistent with strong position-dependence of stiction.
- 2026-07-16 23:40 `40_dither.csv`: Block 1 UNSTICK #1 attempt 1: endstop_safe blip -0.78/120ms out of high end zone (98.0 %), per coordinator unstick protocol
- 2026-07-16 23:41 `41_step_response.csv`: Block 1: zero-duty goto reposition to 15 % (slow ladder runway)
- 2026-07-16 23:42 `42_step_response.csv`: Block 1: reposition retry with goto_duty 0.8 (0.72 goto stalled at 75 %)
- 2026-07-16 23:43 `43_duty_ladder.csv`: Block 1: duty ladder SLOW mode, fwd from ~15.8 %, rungs 0.05-0.70, VM 5.00 V
- 2026-07-16 23:43 `44_dither.csv`: Block 1 UNSTICK #2 attempt 1: endstop_safe blip -0.78/120ms out of high end zone (98.1 %) after slow ladder
- 2026-07-16 23:44 `45_step_response.csv`: Block 1 M5: step 0.6 fwd from 20 %, VM 5.00 V
- 2026-07-16 23:45 `46_step_response.csv`: Block 1 M5: step 0.6 rev from 80 %, VM 5.00 V
- 2026-07-16 23:46 `47_step_response.csv`: Block 1 M5: step 0.7 fwd from 20 %, VM 5.00 V
- 2026-07-16 23:46 `48_dither.csv`: Block 1 UNSTICK #3 attempt 1: endstop_safe blip -0.78/120ms out of high end zone (97.6 %) after 0.7 fwd step
- 2026-07-16 23:46 `49_step_response.csv`: Block 1 M5: step 0.7 rev from 80 %, VM 5.00 V
- 2026-07-16 23:47 `50_step_response.csv`: Block 1 M5: step 0.8 fwd from 20 %, VM 5.00 V
- 2026-07-16 23:48 `51_dither.csv`: Block 1 UNSTICK #4 attempt 1: endstop_safe blip -0.78/120ms out of high end zone (97.7 %) after 0.8 fwd step
- 2026-07-16 23:48 `52_step_response.csv`: Block 1 M5: step 0.8 rev from 80 %, VM 5.00 V
- 2026-07-16 23:48 `53_dither.csv`: Block 1 UNSTICK #5 attempt 1: endstop_safe blip +0.78/120ms out of LOW end zone (2.5 %) after 0.8 rev step
- 2026-07-16 23:49 `54_triangle.csv`: Block 1: triangle FAST mode, peak 0.8, ramp 3 s, fwd from 20 %, VM 5.00 V
- 2026-07-16 23:50 `55_dither.csv`: Block 1 UNSTICK #6 attempt 1: endstop_safe blip -0.78/120ms out of high end zone (98.1 %) after triangle fast
- 2026-07-16 23:51 `56_triangle.csv`: Block 1: triangle SLOW mode, peak 0.8, ramp 3 s, fwd from 20 %, VM 5.00 V
- 2026-07-16 23:51 `57_dither.csv`: Block 1 UNSTICK #7 attempt 1: endstop_safe blip -0.78/120ms out of high end zone (98.1 %) after triangle slow — leaves rig mid-travel for Block 2

## Block 1 COMPLETION (campaign-runner, after coordinator gate: unstick protocol authorized, 23:5x-00:2x, VM = 5.00 V)
- CORRECTION to the previous entry: the failed-goto evidence run is `39_step_response.csv`
  (not 37; `37_step_response.csv` was the successful reposition to 15.1 % before the fast ladder).
- UNSTICK PROTOCOL uses (all: dither endstop_safe, amp 0, |bias| 0.78, 120 ms, 1 attempt each,
  all landed in [5,95] = success; all other interlocks live, VERDICT PASS each):
  - #1 `40_dither.csv` 98.0 -> 75.0 % (frees Block 1 after fast ladder)
  - #2 `44_dither.csv` 98.1 -> 67.6 % (after slow ladder)
  - #3 `48_dither.csv` 97.6 -> 66.9 % (after 0.7 fwd step)
  - #4 `51_dither.csv` 97.7 -> 68.7 % (after 0.8 fwd step)
  - #5 `53_dither.csv` 2.5 -> 47.1 % (bias +0.78, out of LOW zone after 0.8 rev step)
  - #6 `55_dither.csv` 98.1 -> 66.9 % (after triangle fast)
  - #7 `57_dither.csv` 98.1 -> 66.8 % (after triangle slow; leaves rig mid-travel for Block 2)
- Goto anomaly: default goto duty 0.72 STALLED at 75 % (`41_step_response.csv`, 4 s, only 2.8 %
  twitch); retry with goto_duty 0.8 worked (`42_step_response.csv` -> 15.8 %). All later runs used
  goto_duty 0.8. Static friction at ~75 % exceeds 0.72 duty — position-dependence again.
- Duty ladder SLOW (`43_duty_ladder.csv`, fwd from 15.8 %, same rungs as 38): VERDICT WARN
  (endzone 570 ticks at end), tickhealth PASS. Per-rung: <= 0.60 stall (|dpos| <= 0.28 %),
  0.65 creep +3.5 %/s, 0.70 fly +180 %/s (fast mode was +211 %/s — slow decay brakes slightly).
- M5 step responses (hold 4 s, all tickhealth PASS, structurally clean):
  - `45_` 0.6 fwd from ~20: creep ~1 %/s, 20 -> 24.0 %. `46_` 0.6 rev from 80: full stall (79.9 %).
  - `47_` 0.7 fwd from 20: flies, endzone cut at 97, parked 97.6 (cut = data).
  - `49_` 0.7 rev from 80: -43 %/s for 0.5 s then RE-STUCK DEAD at 58.8 % against full 0.7 rev
    torque for 3.5 s. Mid-travel re-stick, direction-dependent (0.7 fwd sails through the same band).
  - `50_` 0.8 fwd from 20: flies, parked 97.7. `52_` 0.8 rev from 80: flies, 80 -> 2.5 % (low zone).
- Triangles peak 0.8, ramp 3 s, fwd from ~20 (both WARN endzone — up-ramp launches the knob into
  the end zone, down-ramp branch is lost there; onset data is the value):
  - `54_triangle.csv` FAST: sustained-motion onset at duty 0.677 (pos ~25 %).
  - `56_triangle.csv` SLOW: onset at duty 0.645 (pos ~20 %).
- Anomalies for the modeler (position/direction dependence is the story of the night):
  1. Breakaway fwd at 15-20 %: between 0.60 and 0.65 (ladders), triangle onset 0.65-0.68.
  2. 0.6 fwd creeps at 20 % but 0.6 rev stalls at 80 %; 0.7 rev re-sticks at ~59 % mid-travel;
     0.72 goto stalls at 75 %. High-friction band somewhere ~55-80 % in the rev direction?
  3. Kinetic friction tiny once flying: 180-211 %/s at 0.7, overshoot into the end zone every time.
- Rig end state: player idle, position 66.8 %, no reboot all block. Scope: still off USB
  (checked once at block start per rule — VID 0x0CE9 absent).
- 2026-07-16 23:57 `58_breakaway_grid.csv`: Block 2 M4: breakaway grid pos 10, both dirs x2, VM 5.00 V
