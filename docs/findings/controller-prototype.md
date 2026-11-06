# Controller prototype in the validated sim (SIL) — 2026-07-17 day session

Phase-3 item "controller prototype tuned in sim" (plan.md). Cascade
feedforward-dominant controller + duty-driven EKF observer **in the loop**
(never sim truth), against plants perturbed within the measured uncertainty.
Code: `analysis/src/slideranalysis/controller.py` (controller),
`silharness.py` (closed loop + acceptance suite), invariants pinned in
`tests/test_controller.py`. Reproduce:

```
cd analysis && uv run acceptance --out ../docs/findings
```

Every run: perturbed plant (friction ±7 % per direction — spans the measured
±0.04-duty breakaway spread; m ±25 %, b ±15 %), sensor noise + measured PWM
spike tail ON, controller/observer on the NOMINAL identified plant only.

## Metrics (plan.md acceptance, v0 targets)

| # | Metric | Target | Sim result | Verdict |
|---|---|---|---|---|
| A1 | Landing, 40 randomized moves, perturbed plants | ≤0.5 % median, ≤1.0 % p95 | **median 0.25 %, p95 0.39 %, max 0.65 %**, 0 give-ups | **PASS** |
| A2 | Rest silence after settle | PWM identically 0 ≥2 s, no re-trigger | 40/40 silent (latched IDLE; exact zeros) | **PASS** |
| A3 | No limit cycle at rest | true position p-p ≤0.3 % | 0.000 % (stuck; measured p-p is sensor noise) | **PASS** |
| A4 | Cruise smoothness @150 %/s | σ/mean ≤0.15 | worst of 4: **0.037** | **PASS** |
| A4' | Cruise smoothness @50 %/s (plan value) | σ/mean ≤0.15 | 0.12–0.46 run-dependent | **NOT CREDIBLE** — 50 %/s is below the identified regime and below the cliff; the real plant will stick-slip there. Plan metric needs renegotiation or closed-loop synthesis from impulses |
| A5 | 50 %-travel move incl. settle (10 runs, half rev) | ≤1.2 s | median 0.65 s, **max 0.78 s** | **PASS** |
| A7 | Detent pull-in from 3 % (10 runs, both sides) | ≤400 ms, ≤1 overshoot | median 264 ms, **max 368 ms, 0 overshoots** | **PASS** |
| — | Stress: rev moves into the 55–80 % band, friction +7 % | completes, ≤1 % | max err 0.39 %, no give-ups | **PASS** |
| — | Grab during move (logic only) | detect, release, no fight, silent | detected 358 ms after clamp, torque ≡ 0 from the next tick, position adopted, silent | **PASS** (logic) |
| A6 | Reversal thunk | no audible thunk | — | **NOT TESTABLE** (no acoustics in sim; M6 says no positional dead band ≥1–2 ms though) |
| A8 | Manual grab, real dynamics | detect ≤100 ms, release ≤2 ticks | release ≤1 tick after detection ✓; detection 358–500 ms | **NOT CREDIBLE in sim** — the hand is a rigid clamp; and see failure mode 1: with pot-only sensing, grab is only distinguishable from a friction stall by torque escalation failing, which takes ~0.4 s by construction. The 100 ms budget likely needs current or accel sensing (Phase 7) |

Plots: `controller_a1_errors.png`, `controller_move_example.png`,
`controller_cruise.png`, `controller_band_move.png`, `controller_grab.png`.

## Controller structure (what actually ships in the numbers above)

torque = **stiction·sign(v_des)** (static map at p̂, kinetic map once moving,
keyed to DESIRED direction — never to a noisy sign) + **b̂·v_des + m̂·a_des**
+ **Kp·(p_ref − p̂) + Kd·(v_des − v̂)** with the reference integrated at v_des
and error-clamped; then: brake → **impulse endgame from the measured M7
table** (forward-only landing; overshoot triggers a rev re-approach below the
target); latched silent IDLE.

| Tuning value | Choice | Rationale |
|---|---|---|
| v_cruise / v_min_cmd | 180 / **120 %/s** | inside the validated flight regime; v_min_cmd is the cliff floor — the trajectory NEVER commands a slower nonzero velocity (pinned by test) |
| a_traj | 5000 %/s² | ~1/3 of what the plant can deliver; keeps FF ≤ u_max |
| Kp (on integrated reference) | 0.05 duty/%, error clamp ±3 % | ±0.15 duty authority = covers the ±0.04–0.06 friction spread with margin; crossover ~14 Hz, Kd adds phase |
| Kd | 2.5e-4 duty/(%/s) | damping without amplifying the ~30–50 %/s observer noise (0.01 duty) |
| approach margin / endgame zone | 1.5 % / 3.5 % | main move always stops short (fwd sense); short moves are impulse-only |
| impulse aim / tol | 0.90 of gap / 0.30 % | undershoot bias: a fwd trim is cheap, an overshoot costs a rev cycle |
| impulse settle | 120 ms (70 ms for ≤12 ms pulses) | SIM HAS NO POST-PULSE CREEP — on hardware these must grow (measured creep up to 1.3 % over 400 ms after big pulses) |
| stall detector | progress < 25 %/s over 100 ms of p̂ | see failure mode 1 — v̂ is blind here |
| kick escalation | +0.08 per 60 ms stalled | breaks the worst measured breakaway (0.80) from any FF level before grab declaration |
| grab declaration | u_max for 120 ms with zero progress, or 3 zero-travel pulses | worst breakaway incl. spread 0.80 < u_max 0.9 ⇒ full torque + no motion cannot be friction |

## Failure modes found (each one is a Phase-4 spec input)

1. **The duty-driven KF velocity is blind under an exogenous clamp.** Only
   position is measured; when a hand stops the slider mid-flight the model
   keeps predicting motion and v̂ climbs to ~350 %/s while the truth is 0
   (visible in `controller_grab.png`). Any logic keyed to v̂ (stall
   detection!) silently fails. Fix that shipped: stall/progress decisions use
   windowed p̂ displacement, never v̂. Spec consequence: the servo state
   machine must not trust model-dominated states under disturbance.
2. **Innovation-gate lockout.** The observer's rest spike gate (reject >5σ)
   latched a permanent 2.5 % estimate offset after the grab released — every
   honest sample looked like a spike. Fix: a gate may only reject ISOLATED
   outliers; >8 consecutive rejections = the offset is real, accept
   (`observer.py::gate_max_consecutive`). Textbook, but it bit within a day.
3. **Near-breakaway cruise is feedback-carried, not FF-carried.** With ±7 %
   friction error the FF alone either stalls or flies; the reference
   integrator (±0.15 duty authority) is what actually holds cruise — in the
   band-stress runs it did all the escalation work (0 kicks fired). Kicks
   remain as the saturation backstop and the grab discriminator.
4. **Grab vs. friction stall is fundamentally slow with pot-only sensing**
   (~0.4 s: escalate to u_max, wait 120 ms). A8's 100 ms needs a second
   signal (motor current / knob accel — Phase 7 candidates).
5. **A4 at 50 %/s is not a meaningful sim test** — flagged NOT CREDIBLE, and
   on the real plant the cliff says sustained 50 %/s open sliding may not
   exist at all. If A4@50 stays a product requirement, the trajectory layer
   must synthesize slow apparent motion from impulse trains (M7 table), or
   the requirement moves to ≥120 %/s.

## A4@50 via impulse trains: feasible as SURGES, not as a glide

Follow-up to gate question 1 (controller-spec §6). Trajectory mode: fixed
±0.8 pulses from the measured M7 table, fired by a deficit-triggered
scheduler against a 50 %/s reference ramp (self-pacing, KF in the loop).
20 → 70 % traverse, nominal plant and friction +7 %. Reproduce:
`uv run acceptance --impulse-a4`. Plot:
`controller_a4_impulse_trains.png`. Both smoothness numbers, as demanded:

| Schedule | v_mean | σ/mean (100 ms windows) | max step (rest-to-rest) |
|---|---|---|---|
| 3 ms, deficit-paced (bursts) | 50 %/s ✓ | **0.07** ✓ | **6.3–7.3 %** (burst surges) |
| 5 ms, deficit-paced | 50 %/s ✓ | **0.08** ✓ | 2.2–2.9 % |
| 8 ms, deficit-paced | 50 %/s ✓ | **0.05–0.07** ✓ | 2.4–3.2 % |
| 20 ms, deficit-paced | 51 %/s | 0.24–0.32 ✗ | 4.3–4.5 % |
| 5 ms, isolated (min gap 17 ms) | **16 %/s ✗** | 0.13–0.15 | **0.4 %** |
| 8 ms, isolated (min gap 20 ms) | **28–29 %/s ✗** | 0.14–0.17 | **1.0 %** |

Verdict, both halves stated:

- **Numerically: FEASIBLE.** Deficit-paced 5–8 ms trains hit 50 %/s with
  σ/mean 0.05–0.08 — comfortably inside A4's 0.15 — and are robust to the
  +7 % friction corner (the scheduler self-paces; the sim's warm-stiction
  regime applies since pulses arrive every 20–120 ms, far inside the D0/D1
  warm window — delivery ×1.0 up to ~8 min dwell, degradation only at
  ≥30 min, which concerns the FIRST pulse after a long rest, not trains).
- **Optically/haptically: it is stop-and-go, not a glide.** The velocity
  trace is 100–300 %/s spikes separated by rests — the cliff makes that
  physically unavoidable (no sustained sliding below ~100 %/s exists). At
  50 %/s the passing schedules move in visible 2–3 % surges at 15–30 Hz;
  window-σ/mean simply cannot see them at 100 ms scale.
- **The fine-step variant undershoots in sim, and this is exactly the sim's
  documented blind spot:** isolated 8 ms pulses yield ~0.8 %/pulse in sim vs
  the MEASURED 1.75 % (§4c under-prediction: post-pulse creep + m at the
  range top). On the measured table, isolated 8–10 ms pulses at 28–35 Hz
  deliver **45–60 %/s at ~1 % steps** — plausibly a passable A4 AND
  acceptable optics, but only HIL can settle it.

Recommendation into the gate: keep the three options of spec question 1,
now with numbers — (a) restate A4 at ≥120 %/s (glide exists there natively);
(b) accept surge-mode 50 %/s (passes A4-as-written, visible 2–3 % surges);
(c) HIL-test isolated 8–10 ms trains for ~50 %/s at ~1 % steps — the
measured impulse table says yes, the (conservatively biased) sim says 30 %/s.
No firmware work is blocked on this: the trajectory layer emits pulses
either way.

## Sim-credibility boundaries (explicit)

Credible: breakaway spread, position/direction friction structure, the
cliff, brake decay, impulse travel statistics, sensor noise incl. spike tail,
end-zone interlock. NOT credible: sub-30 %/s friction texture (stick-slip
creep unmodeled), post-pulse creep (settle waits optimistic), acoustics (A6),
hand dynamics (A8), thermal drift. Every PASS above is conditional on the
plant model being right within its stated envelope — the HIL phase (Phase 6)
re-runs exactly these scenarios on hardware via the lab-rig recorder.
