# Controller Spec — `slider_servo`

> **DRAFT / NICHT FREIGEGEBEN.** Phase-4 approval gate (plan.md): Jan
> approves the trigger graph (§2) BEFORE any implementation. Numbers cited
> from `docs/plant-model.md` (measurements), SIL evidence from
> `docs/findings/controller-prototype.md`, parameter sourcing from
> `docs/auto-ident-spec.md`. Prototype source: `analysis/src/slideranalysis/
> controller.py` + `silharness.py` (all acceptance metrics that the sim can
> credibly test: PASS, commit d55debd).

## 1. Layer architecture

Hard requirement (Phase 5): the core is **pure C++, zero IDF includes**,
natively compilable on macOS; hardware only via a thin HAL. One instance =
one object; no function-static state anywhere.

| Layer | Owns | Talks to |
|---|---|---|
| L5 Command (ESPHome/HA wrapper) | set_target, release, tolerance, param push, diagnostics out | L4; NVS param blob |
| L4 Trajectory | velocity schedule (cliff floor ≥ v_min_cmd), waypoint = target − approach_margin, endgame pulse planning from the per-unit M7 table | L3 |
| L3 Motion core (state machine §2) | FF cascade + reference-integrator feedback, stall escalation, grab declaration, settle latch | L2, L1 |
| L2 Observer | 2-state duty-driven EKF, R(duty) from sensor model, innovation gate with bounded lockout, **progress signal from p̂ window** | L1 (ADC via HAL) |
| L1 Safety | thermal duty budget, abort switch, **end-zone policy = Gate-Frage 8** (lab rig removed the cut 2026-07-17 — passive marker only; for the servo a *directional* clamp「only block torque INTO the nearby stop」would avoid the trap while keeping stop protection) | HAL (PWM) — clamps L3's output, reports the clamp |
| L0 HAL | `read_adc()`, `set_pwm(signed duty)`, `now_us()`, param load/store | hardware / SIL stubs |

Tick pipeline (1 kHz, same order in SIL and firmware): sample ADC → L2
predict(u[t−1])+update → L3/L4 evaluate → u[t] → L1 clamp → PWM + recorder.
The recorder taps the pipeline read-only; it can never stall or reorder it.

Cascade law (L3, duty units until the K/R refit; the seam is `PlantParams`):

    u = sign(v_des)·F(p̂)  +  b̂·v_des  +  m̂·a_des  +  Kp·(p_ref − p̂) + Kd·(v_des − v̂)

F = static map while progress-stalled (+ escalation kick), kinetic map once
moving; **keyed to sign(v_des) only** — never to torque sign (legacy bug) or
v̂ sign. p_ref integrates v_des and freezes beyond ±e_p_clamp.

## 2. State machine — trigger graph

States: **IDLE** (output ≡ 0, latched), **MOVE**, **BRAKE**, **PULSE**,
**WAIT**, **RELEASED** (torque-free, waiting for stillness; entered by grab,
command, or persistent interlock). Trigger sources: `CMD` (HA/API), `TICK`
(per-tick evaluation), `OBS` (observer-derived event), `ILK` (L1 reports a
clamp), `TIME` (timer expiry). Every transition below is exhaustive — a
(state, trigger) pair not listed is a no-op by definition, and the SIL
harness asserts exactly that.

| # | From | Trigger (who) | Guard | To | Action |
|---|---|---|---|---|---|
| 1 | IDLE | CMD set_target | \|err\| ≤ tol | IDLE | re-latch hold := p̂ |
| 2 | IDLE | CMD set_target | \|err\| ≤ endgame_zone | WAIT | plan first pulse next expiry |
| 3 | IDLE | CMD set_target | else | MOVE | plan waypoint = target − margin |
| 4 | IDLE | OBS hand-move (\|p̂−hold\| > adopt_err for adopt_ms) | — | IDLE | hold := p̂, **no torque** |
| 5 | MOVE | TICK remaining ≤ stop_dist(v̂) | — | BRAKE | u := 0 |
| 6 | MOVE | TICK progress-stall ≥ stall_ms (OBS p̂-window, **never v̂**) | escalation not saturated | MOVE | kick += kick_boost each kick_period |
| 7 | MOVE | TICK progress-stall AND \|u_raw\| ≥ u_max for grab_sat_ms | — | RELEASED | reason=grab; u := 0 next tick |
| 8 | MOVE/BRAKE/PULSE/WAIT | CMD set_target (new) | — | BRAKE | target := new; replan at stillness (via 9→10) |
| 9 | BRAKE | OBS still (progress < stall_v) | — | WAIT | settle timer (long) |
| 10 | WAIT | TIME expiry | \|err\| ≤ tol | IDLE | settle latch; hold := p̂ |
| 11 | WAIT | TIME expiry | pulses ≥ impulse_max or removes > 3 | IDLE | **gave_up diagnostic flag** (never silent) |
| 12 | WAIT | TIME expiry | ≥ grab_pulses pulses with zero travel | RELEASED | reason=grab (M7: 0.8-pulses always move a free slider) |
| 13 | WAIT | TIME expiry | \|err\| > endgame_zone + 2 | MOVE | removes += 1 (escalation-breakthrough re-approach) |
| 14 | WAIT | TIME expiry | err > 0 | PULSE | fwd width from table, aim·err |
| 15 | WAIT | TIME expiry | err < 0 | PULSE | rev width to target − reapproach; overshoots += 1 iff last pulse was fwd |
| 16 | PULSE | TICK width elapsed | — | WAIT | settle timer (short ≤12 ms pulses, long otherwise) |
| 17 | PULSE | CMD set_target / abort | — | BRAKE | pulse cut mid-width (allowed; endgame re-measures) |
| 18 | MOVE/PULSE | ILK clamp persistent > ilk_yield_ms | — | RELEASED | reason=interlock; do not fight L1 |
| 19 | any except IDLE | TIME watchdog (2× expected move time + 1 s) | — | RELEASED | reason=watchdog; diagnostic flag |
| 20 | RELEASED | OBS still ≥ release_still_ms | no pending target | IDLE | hold := p̂ (adopt) |
| 21 | RELEASED | OBS still ≥ release_still_ms | pending CMD target | MOVE/WAIT per rules 1–3 | execute pending |
| 22 | RELEASED | CMD set_target | — | RELEASED | store pending (cannot move against a hand) |
| 23 | any | CMD release/abort | — | RELEASED | u := 0 within ≤ 2 ticks |
| 24 | any | REC recorder full | — | — | **no transition, ever** (ring buffer; control is recorder-blind) |
| 25 | any | OBS fault (NaN/P divergence) | — | RELEASED | reason=fault; diagnostic; requires CMD to leave IDLE after |

## 3. Invariants as tripwires (each: SIL assertion + negative control)

Seeds T1–T7 exist as `tests/test_controller.py`; all get negative controls
(seeded bug must make the test fail) in Phase 5 per plan.md.

| # | Tripwire (falsifiable statement) | Negative control that must break it |
|---|---|---|
| T1 | In IDLE and RELEASED the PWM command is **exactly 0 every tick** (A2/A3: silence is latched, not regulated) | add a small P-hold term |
| T2 | No creep commands: v_des is 0 or ≥ v_min_cmd; during MOVE no duty plateau in (0.05, 0.45) | set v_min_cmd = 30 |
| T3 | Stiction FF sign == sign(v_des) on every active tick; FF never reads torque sign or v̂ sign | key FF to sign(u) |
| T4 | Stall/progress decisions read the p̂ window only — with the plant clamped and v̂ forced high (grab replay), stall is still detected ≤ stall_ms + window | revert to v̂-based stall |
| T5 | Innovation-gate lockout bounded: a real 5 % offset at rest is accepted within gate_max_consecutive + KF convergence ≤ 200 ms | remove the consecutive-reject escape |
| T6 | Grab declared ⇒ u ≡ 0 from the next tick until stillness; max one tick of residual torque | drop the release |
| T7 | The last motion-causing pulse before a settle latch is forward — both move directions | allow direct rev landings |
| T8 | Every terminal settle carries an honest flag: reached \| gave_up \| adopted — no silent failure | clear gave_up before latch |
| T9 | Bounded activity: every command reaches IDLE/RELEASED within the watchdog bound | command a target inside the end zone with watchdog off |
| T10 | Pulses only at ±impulse_duty with widths inside the identified table domain [2, 50] ms | emit a 200 ms “pulse” |
| T11 | \|u\| ≤ u_max on every tick incl. FF + kick + feedback sum | raise kick cap |
| T12 | Observer input independence: z comes from the ADC path only; with a frozen sensor and active duty, p̂ converges to z (post-gate), not to the model | blend commanded duty into z |
| T13 | Recorder independence: bit-identical control trace with recorder on/off/full | add recorder backpressure to the tick |
| T14 | Interlock supremacy: L1's clamp overrides L3 the same tick (endzone: applied duty 0 regardless of command) | apply clamp one tick late |
| T15 | (state, trigger) pairs outside §2 are no-ops — fuzz all triggers in all states | add an undocumented transition |

## 4. Forbidden list (drift lives in the gaps)

1. **No function-static state** — instance members only (multi-instance design rule).
2. **No friction FF keyed to torque sign or estimated-velocity sign** — the
   legacy ±0.42 relay bug. Desired-velocity sign only.
3. **No commanded cruise below the cliff** (measured: ≤1 %/s at 0.66,
   ≥100 %/s at 0.68): no v_des in (0, v_min_cmd), no sustained duty in
   (0.05, 0.45). Impulses (±0.8, ≤50 ms) are the only sub-cliff motion tool.
4. **No endgame from both directions** — forward landings only; overshoot
   costs a rev re-approach, never a rev landing.
5. **No filtering, averaging, or resampling in the recorder path**, and no
   controller decision ever reads recorder state (T13, trigger 24).
6. **The observer never consumes commanded duty as a measurement** — duty is
   model input, ADC is the only z (T12). No echo loops via HA state.
7. **goto stays in the lab firmware** — the servo has no bang-bang position
   mode; repositioning is a normal MOVE.
8. **No hidden integrators**: the reference integrator with clamp+freeze is
   the only I-action; adding a Ki anywhere needs a spec change.
9. **No torque at rest** — incl. no hold dither until Jan rules on M8
   audibility. IDLE means the H-bridge outputs are identically zero.
10. **No blocking calls, heap allocation, logging, or flash access in the
    tick path** (lab rig: tick p99 418 µs at 1 kHz — that budget is the law).
11. **No magic numbers in code** — every constant lives in the §5 table with
    a source; "tuned on the bench, forgot to write down" is how the legacy
    controller happened.
12. **No IDF includes in the core** — HAL only (Phase-5 SIL requirement).

## 5. Parameters: source of truth

Per unit (NVS, from auto-ident — see auto-ident-spec for methods/budget):
endpoint calibration; σ₀, σ_PWM; breakaway map F_s(p, dir) 7×2 + sustain
flags; F_k mid-level + b per direction; impulse table 2×6 widths→travel.
Jan's north star: **every unit identifies these itself, bounded time, no
scope, no host.**

Compile-time defaults (HA-overridable where marked):

| Param | Default | Basis | HA? |
|---|---|---|---|
| v_cruise / v_min_cmd | 180 / 120 %/s | validated flight regime / cliff floor + margin (sustained flight measured from ~104 %/s) | yes / no |
| a_traj | 5000 %/s² | ≈⅓ of plant capability | no |
| Kp, e_p_clamp, Kd | 0.05 /%, 3 %, 2.5e-4 | ±0.15 duty authority ≥ measured ±0.04–0.06 friction spread; SIL-tuned | no |
| u_max | 0.9 | above worst measured breakaway 0.80; grab discrimination needs the gap | no |
| tol | 0.30 % | A1 budget (0.5 % median) minus sensor σ | yes |
| approach_margin / endgame_zone / reapproach | 1.5 / 3.5 / 1.2 % | M7 table domain; SIL-tuned | no |
| impulse_duty / aim | 0.8 / 0.90 | the duty the table was measured at / undershoot bias | no |
| impulse settle (long/short) | 120 / 70 ms | **sim value — HIL must re-tune up** (measured post-pulse creep ≤1.3 % over 400 ms is unmodeled) | yes |
| stall (progress, window, delay) | 25 %/s, 100 ms, 80 ms | above sensor-noise progress floor | no |
| kick (+step, period) | 0.08, 60 ms | reaches u_max from any FF in ≤3 steps | no |
| grab (sat, zero-travel pulses, still) | 120 ms, 3, 300 ms | u_max > worst breakaway ⇒ discriminates; floor ~0.4 s total | no |
| gate (σ, max consecutive) | 5, 8 | spike tail measurement / lockout bound (T5) | no |

## 6. Explicit HIL items (Phase 6) and decisions for Jan

Sim could not credibly test — these are **hardware verification items**:
(a) A4@50 %/s cruise (sim not credible; see question 1), (b) A6 reversal
thunk (no acoustics; M6 found no positional dead band ≥1–2 ms), (c) A8 grab
timing with a real hand, (d) impulse settle times under real post-pulse
creep, (e) long-term drift/temperature.

**Questions to answer at this gate:**

1. **A4**: keep 50 %/s as a requirement (then v1 must synthesize slow motion
   as impulse trains — extra trajectory mode), or restate A4 at ≥120 %/s?
2. **A8**: accept the ~0.4 s pot-only grab floor for v1, or pull Phase-7
   sensing (shunt current / knob accel) forward now? This is a hardware
   decision that changes L2.
3. **Grab semantics**: released slider ADOPTS its position (fader UX,
   specced here) — or should the move resume after release?
4. **Detents/force-feedback**: still v1.1 (plan.md open question 2)? This
   spec has no detent layer; adding one later touches L4 only.
5. **Dither**: M8 says it buys base-duty headroom ≈ amplitude; forbidden at
   rest until you rule on audibility (forbidden list #9).
6. **tol = 0.3 %** default: consistent with A1/A3 — confirm as the UX default?
7. **The trigger graph (§2)**: approve, or rewrite states/transitions before
   any C++ exists.
8. **End-zone policy for the servo (L1)**: the lab rig dropped the
   direction-agnostic cut today (nutzlos + hinderlich, 24 traps in one
   night). For `slider_servo`: (a) no positional clamp at all (budget +
   thermal only, like the rig now), or (b) **directional clamp** — torque
   toward the nearby stop is cut outside [3, 97], torque away is always
   allowed (no trap by construction)? T14's negative control follows the
   choice. Spec currently assumes (b) pending your call.
