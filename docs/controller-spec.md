# Controller Spec — `slider_servo`

> **DRAFT / NICHT FREIGEGEBEN.** Phase-4 approval gate (plan.md): Jan
> approves the trigger graph (§2) BEFORE any implementation.
> **Gate-Fragen 1–8 beantwortet (Jan, 2026-07-17 abends)** — eingearbeitet:
> §2 um CRAWL erweitert (Antwort 1, Zeilen 26–29), Endzonen-Politik (a)
> (Antwort 8), §6 jetzt Antworten statt Fragen. Freigabe = Jans OK auf den
> revidierten Graphen; alles außer den CRAWL-Zeilen stand schon zur Abnahme.
> Numbers cited
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
| L4 Trajectory | velocity schedule (cliff floor ≥ v_min_cmd), waypoint = target − approach_margin, endgame pulse planning from the per-unit M7 table, **CRAWL pulse-train pacing for requested cruise < v_min_cmd** (§2 rows 26–29), target clamping to the calibrated span (dead-band edges from endpoint calibration; `target_clamped` diagnostic) | L3 |
| L3 Motion core (state machine §2) | FF cascade + reference-integrator feedback, stall escalation, grab declaration, settle latch | L2, L1 |
| L2 Observer | 2-state duty-driven EKF, R(duty) from sensor model, innovation gate with bounded lockout, **progress signal from p̂ window** | L1 (ADC via HAL) |
| L1 Safety | thermal duty budget, abort switch, **end-zone policy (a) — DECIDED 2026-07-17**: no positional torque clamp at all (Jan: hardware is very fault tolerant, 100 % duty for 10 s is fine; the rig's direction-agnostic cut caused 24 traps in one night). Budget + thermal interlocks are the only current protection. The real end-zone problem is the **sensor dead band** — handled in L4/L5 as target clamping, not in L1 as torque policy | HAL (PWM) — clamps L3's output, reports the clamp |
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
**WAIT**, **CRAWL** (paced sub-cliff pulse train toward a distant target —
Gate answer 1: slow motion is a v1 requirement, and the cliff means no
sustained duty can do it; a ratcheting reference advances at the requested
rate and each due pulse is dosed from the per-unit table for the current
ratchet error, so the train self-corrects against table optimism),
**RELEASED** (torque-free, waiting for stillness; entered by grab,
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
| 8 | MOVE/BRAKE/PULSE/WAIT/CRAWL | CMD set_target (new) | — | BRAKE | target := new; replan at stillness (via 9→10) |
| 9 | BRAKE | OBS still (progress < stall_v) | — | WAIT | settle timer (long) |
| 10 | WAIT | TIME expiry | \|err\| ≤ tol | IDLE | settle latch; hold := p̂ |
| 11 | WAIT | TIME expiry | pulses ≥ impulse_max or removes > 3 | IDLE | **gave_up diagnostic flag** (never silent) |
| 12 | WAIT | TIME expiry | ≥ grab_pulses pulses with zero travel | RELEASED | reason=grab (M7: 0.8-pulses always move a free slider) |
| 13 | WAIT | TIME expiry | \|err\| > endgame_zone + 2 | MOVE | removes += 1 (escalation-breakthrough re-approach) |
| 14 | WAIT | TIME expiry | err > 0 | PULSE | fwd width from table, aim·err |
| 15 | WAIT | TIME expiry | err < 0 | PULSE | rev width to target − reapproach; overshoots += 1 iff last pulse was fwd |
| 16 | PULSE | TICK width elapsed | — | WAIT or CRAWL (origin state) | settle timer (short ≤12 ms pulses, long otherwise) |
| 17 | PULSE | CMD set_target / abort | — | BRAKE | pulse cut mid-width (allowed; endgame re-measures) |
| 18 | MOVE/PULSE/CRAWL | ILK clamp persistent > ilk_yield_ms | — | RELEASED | reason=interlock; do not fight L1 |
| 19 | any except IDLE | TIME watchdog (2× expected move time + 1 s) | — | RELEASED | reason=watchdog; diagnostic flag |
| 20 | RELEASED | OBS still ≥ release_still_ms | no pending target | IDLE | hold := p̂ (adopt) |
| 21 | RELEASED | OBS still ≥ release_still_ms | pending CMD target | MOVE/WAIT per rules 1–3 | execute pending |
| 22 | RELEASED | CMD set_target | — | RELEASED | store pending (cannot move against a hand) |
| 23 | any | CMD release/abort | — | RELEASED | u := 0 within ≤ 2 ticks |
| 24 | any | REC recorder full | — | — | **no transition, ever** (ring buffer; control is recorder-blind) |
| 25 | any | OBS fault (NaN/P divergence) | — | RELEASED | reason=fault; diagnostic; requires CMD to leave IDLE after |
| 26 | IDLE | CMD set_target with requested cruise v_req < v_min_cmd | \|err\| > endgame_zone | CRAWL | ratchet := p̂; rate := clamp(v_req, crawl envelope) |
| 27 | CRAWL | TICK pulse due (settle elapsed, ratchet ahead of p̂) | remaining > endgame_zone | PULSE | width from per-unit table for (ratchet − p̂), domain-clamped; origin = CRAWL |
| 28 | CRAWL | TICK | remaining ≤ endgame_zone | WAIT | endgame handover — landing rules 10–15 unchanged (fwd-only) |
| 29 | CRAWL | TICK ≥ crawl_grab_pulses zero-travel pulses OR \|p̂ − ratchet\| > crawl_override_err | — | RELEASED | reason=grab (hand blocks or drags — Gate answer 3: never fight) |

CRAWL honesty (T8 family): persistent delivery shortfall (achieved rate
< ½·requested over 2 s despite domain-max widths) sets a `crawl_degraded`
diagnostic flag and continues — never a silent slow-down.

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
| T14 | Interlock supremacy: L1's clamp overrides L3 the same tick (thermal/budget: applied duty clamped regardless of command — no end-zone clamp exists, policy (a)) | apply clamp one tick late |
| T15 | (state, trigger) pairs outside §2 are no-ops — fuzz all triggers in all states | add an undocumented transition |
| T16 | CRAWL emits **zero duty between pulses** — no inter-pulse hold, ramp, or creep duty; every motion in CRAWL is a table-domain pulse (T2/T10 extended to CRAWL) | add a 0.1 inter-pulse hold duty |
| T17 | CRAWL override: hand blocking (crawl_grab_pulses zero-travel pulses) or dragging (\|p̂ − ratchet\| > crawl_override_err) reaches RELEASED with u ≡ 0 the next tick | drop the ratchet-error check |

## 4. Forbidden list (drift lives in the gaps)

1. **No function-static state** — instance members only (multi-instance design rule).
2. **No friction FF keyed to torque sign or estimated-velocity sign** — the
   legacy ±0.42 relay bug. Desired-velocity sign only.
3. **No commanded cruise below the cliff** (measured: ≤1 %/s at 0.66,
   ≥100 %/s at 0.68): no v_des in (0, v_min_cmd) to the L3 cascade, no
   sustained duty in (0.05, 0.45). Impulses (±0.8, ≤50 ms) are the only
   sub-cliff motion tool — isolated in the endgame, paced trains in CRAWL.
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
| v_crawl_max | 60 %/s | 20 ms pulse (+6.2 % med) per 120 ms short-settle; requested cruise in (v_crawl_max, v_min_cmd) rounds up to v_min_cmd | no |
| crawl period (min/max) | 120 / 600 ms | short-settle floor (**HIL re-tunes up**, same caveat as impulse settle) / pacing ceiling for the slowest useful crawl | no |
| crawl_grab_pulses / crawl_override_err | 3 / 3 % | mirrors WAIT grab rule 12 / adopt_err | no |

## 6. Gate answers (Jan, 2026-07-17 abends) and HIL items (Phase 6)

Sim could not credibly test — these are **hardware verification items**:
(a) slow cruise via CRAWL (pacing envelope + tracking), (b) A6 reversal
thunk (no acoustics; M6 found no positional dead band ≥1–2 ms), (c) A8 grab
timing with a real hand, (d) impulse settle times under real post-pulse
creep (drives crawl period floor too), (e) long-term drift/temperature,
(f) **CRAWL acoustics** — pulse-train clacking vs. the dither-assisted
variant, Jan's ear decides.

**Gate answers:**

1. **A4 — slow motion is required** („fast is easy"). 50 %/s stays; v1
   synthesizes sub-cliff cruise as CRAWL pulse trains (§2 rows 26–29).
   A4 restated in plan.md: fast metric unchanged, slow metric = ramp
   tracking + acoustics, both HIL.
2. **A8 — accepted**: the ~0.4 s pot-only grab floor is fine for v1
   („grabbing is relaxed"). A8 restated to ≤0.5 s detect, release ≤2 ticks
   unchanged. No Phase-7 sensing pulled forward; L2 unchanged.
3. **Grab semantics — adopt, never resume, never fight**: „the user is
   always right." Fighting a hand exists only as explicit force feedback,
   which is deferred (answer 4). v1 = slow controlled motion with override
   detection (rules 4, 7, 12, 29; T4/T6/T17).
4. **Detents/force feedback — v1.1** (also closes plan.md open question 2).
   No detent layer in v1; a later detent layer touches L4 only.
5. **Dither — implementer's choice** („keep if needed, remove if not";
   Jan doesn't care *how* slow motion is done, preferably silent). Plan:
   pulse-train CRAWL is primary; the M8 base+amplitude law makes a
   dither-assisted silent variant the fallback iff pulses are audible and
   dither is not (HIL item f). Forbidden list #9 (no torque at rest)
   stands regardless — dither never runs in IDLE.
6. **tol — Jan's UX bar is 2 %**. Spec default stays 0.3 % (A1/A3
   ambition, already met in SIL; HA-overridable). Consequence: a gave_up
   or adopted settle inside 2 % is acceptable UX, not a failure — the
   honest flags (T8) report it, nobody escalates hardware over it.
7. **Trigger graph — rewrite license granted** („rewrite what you need").
   Exercised for CRAWL (states list, rows 8/16/18 touched, 26–29 added).
   Everything else stands as drafted. Formal gate close = Jan's OK on the
   revised rows.
8. **End zones — policy (a), no positional torque clamp.** Jan: the dead
   band is the bad part, but „hw is very fault tolerant, 100 % motor for
   10 seconds no problem" → budget + thermal interlocks are sufficient
   current protection; no directional clamp, no trap by construction.
   The dead band is handled as a *sensor* problem in L4/L5: endpoint
   calibration defines the usable span, targets clamp into it with a
   `target_clamped` diagnostic. T14's negative control retargets to the
   thermal/budget clamp.
