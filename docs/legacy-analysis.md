# Legacy Autopsy — `EsphomeZoo/components/motorized_slider`

Read-only analysis of the old component (state: 2026-07-16, single commit `3c4207d init`).
Purpose: extract *requirements, constants, and lessons* — NOT structure. Per the
clean-room rule, the new implementation never mirrors this code's control flow.

## What may be reused (the allowed list)

- **Hardware facts:** ALPS RS60N11M9 (5 kΩ linear, 60 mm), pot as divider with 1 kΩ
  series on the + side, 3.3 V; wiper → ADC1 (GPIO9 on the rig); LEDC PWM 19.5 kHz on
  GPIO10/11 into cheap H-bridge; 5 V motor supply. Raw endpoints observed ≈8000 (pos 0)
  and ≈100 (pos 100) — wiring is inverted, 13-bit ADC.
- **Empirical constants as starting hypotheses** (to be re-measured in M4/M5):
  breakaway duty ≈0.424 fwd / ≈0.402 bwd (single point, mid-travel); max useful
  slider speed ≈50 %/s; ADC read ≈250 µs for 4 oneshot samples inside the 1 kHz tick.
- **Feature list** (= requirements input): trajectory moves, Rastpunkte force
  feedback, manual-override detection, boot self-calibration of raw endpoints,
  HA position sensor.
- **Boot calibration idea** (full-torque to both stops, watch raw extreme stall) —
  worked reliably, keep the concept.
- HA-facing names for the eventual `kueche_mief` migration (entity names frozen).

## Architecture it had (for the record, not for reuse)

1 kHz `esp_timer` tick: read ADC (4× oneshot, 13-bit) → scalar "Kalman" position
filter → finite-difference velocity + PT1 (τ=150 ms) + hard-zero below 9 %/s →
state machine INITIALIZE / IDLE / MOVE2POS / FORCEFEEDBACK. MOVE2POS: trapezoidal
trajectory → position setpoint → PID (Kp 0.05, Ki 0.005, Kd 0) → `set_torque()`.
`set_torque()` adds a stiction offset (±0.42) keyed to commanded-torque sign below a
velocity threshold, plus optional square-wave dither (built, default off, never used).
FORCEFEEDBACK: constant-magnitude pull to nearest Rastpunkt with blend ramp +
velocity damping active only above 10 %/s.

## Why it wasn't smooth — the three enemies

### Friction → the compensation was a relay
Stiction FF switched on the *sign of commanded torque* (threshold ±0.02). Near target:
tiny error flips PID sign → ±0.42 torque kick → overshoot → flip back. Classic relay
limit cycle. Mitigated by giving up: ±2 % "allowed error" and torque hard-off 400 ms
after the setpoint stopped changing. Positioning accuracy was sacrificed to hide chatter.

### Noise → velocity blind exactly where it matters
Differentiating the quantized pot at 1 kHz multiplies noise ×1000; the 150 ms PT1
that tames it adds 150 ms phase lag (hence Kd=0, hence wobbling detent damping);
the <9 %/s → 0 deadband removes all velocity information in the stick-slip regime.
The "Kalman" filter was a random-walk scalar filter on position (adaptive EMA):
no motion model, no torque input, no velocity output.

### Backlash → known but never handled
`follow_trajectory()` contains the comment "initially we have to pretension the belt"
— and no implementation. No width measurement, no reversal kick, no approach-direction
policy. Reversals traverse the dead band open-loop while the integrator winds up,
then the belt engages with a thunk.

## Verified bugs (evidence the old code is not a safe template)

| Where | Bug |
|---|---|
| `move_to_position()` | `last_torque` is a non-static local reset to 0 each call → "reset integral on torque sign change" can never fire (dead code) |
| `monitor_manual_movements()` | `labs()` on a float difference → truncation to whole integers; threshold semantics off |
| `calculate_rastpunkt_force()` | asymmetric "already moving toward detent" thresholds (+60 vs −30) — unexplained |
| `set_target_position()` | expects 0–1 but YAML `target_position: 42` is 0–100 → clamps to 1.0 → plans a move to 100 % |
| `plan_trajectory()` | short-move branch computes `t_acc` two inconsistent ways (`√(d/a)` then `√(d/2a)`); float-rounding edge can produce an over-fast profile |
| `initialize_motor()` | `DETECT_FRICTION` state exists but is skipped (AUTOTUNE → MOVE_TO_100); friction constants effectively hardcoded |
| whole file | pervasive function-`static` state → single-instance only, untestable, hidden coupling between states |

## Lessons carried into the new design

1. Friction FF must key on **planned motion direction**, never on noisy feedback or
   commanded-torque sign.
2. A velocity estimate needs a **model with the torque input**, not a longer filter.
3. Backlash needs explicit treatment: measured width, reversal feedforward,
   unidirectional final approach.
4. The endgame (last ~2 %) is a different control problem than the cruise —
   calibrated impulses beat proportional creep on a stiction-dominated plant.
5. Dither exists as infrastructure; whether it's usable is an *acoustics* question
   (audio gear must stay silent) → measure, don't assume (M8).
6. Silence at rest (PWM hard-off in IDLE) was done right — keep as invariant A2.
7. No function-static state in the new code; everything observable, everything testable.
