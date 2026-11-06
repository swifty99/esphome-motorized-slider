# GLIDE mode spec — "go there as slow as possible, but never stop, nail it first shot"

> **FREIGEGEBEN 2026-07-17 — build authorized.** New motion mode. Grounded in the
> shipped firmware (`firmware/components/slider_servo/`), the plant model
> (`docs/plant-model.md`), the CRAWL work
> (`docs/findings/crawl-smoothness-2026-07-18.md`), and the controller spec it
> extends (`docs/controller-spec.md`). Written 2026-07-17.
>
> **Gate answers (Jan, 2026-07-17):** Q1 — **not-stopping is the priority**, run at
> the never-stop floor ("reasonable slow"); requests below the floor round **up**.
> Q2 — **new `State::GLIDE`** approved. Q3 — landing *tolerance* is accepted, but
> **judder on landing is not** → the coast landing must be a single forward seat,
> no hunt/limit-cycle/belt-flip. Q4 — self-learning **including NVS persistence**
> (bounded, validity-stamped, undershoot-safe). Mandate: build with subagents
> (code/test/measure/fit), **flashing allowed**, loop until satisfied.

## 0. The ask (Jan, verbatim)

> "Go to position, as slow as possible, BUT never STOP in between. Try to nail
> the target first shot. Self-learning parameters."

Four requirements, and **three of the four fight today's CRAWL directly**:

| Ask | Today's CRAWL | Gap |
|---|---|---|
| as slow as possible | 6–35 %/s exists | ✅ but see §1 — "never stop" removes the *slowest* rates |
| **never stop in between** | stop-start: `|v̂|>10 %/s` only **14.7 %** of the time; ~12 clacks/s (T16: zero duty between pulses) | ❌ the opposite of today |
| **nail target first shot** | hunt-and-peck endgame: pulse→settle→measure→correct, ~4 corrections | ❌ the opposite of today |
| self-learning params | `crawl_step` self-adjusts pulse *width* only (`crawl_w_scale_`, EMA) | ⚠️ partial — must also learn coast + cadence |

So this is a **new mode**, working name **GLIDE**, not a CRAWL tweak. It is the
"keep-rolling" idea flagged *designed-but-not-built* in the crawl findings §7,
**plus** a predictive **coast/kick-sized landing** that replaces the endgame hunt,
**plus** self-learning extended from width to cadence and stopping distance.

## 1. Physics: what "never stop" costs, and why slow buys first-shot

Two hard facts from `plant-model.md`, and the number that ties them together.

**(a) There is no continuous sub-breakaway drive (the cliff, §2b).** ≤1 %/s below
~0.66–0.68 duty, ≥100 %/s above, transition <0.02 duty. Dither doesn't soften it
(§4d). So slow motion is **impulse-only** — GLIDE is a pulse train, same as CRAWL.
Genuinely smooth *velocity* would need closed-loop kinetic-branch cruise (§10,
out of scope for v1).

**(b) Kinetic friction (~0.55) < static breakaway (~0.72) (§2 vs §3).** Once
moving, a kick skips breakaway *and* the "unstick cap" (`plant_sim.h:90-95`,
`observer.h:77-82`), so **a kick that arrives while still rolling is cheaper and
smaller than one from rest.** That is the entire mechanism of "never stop": keep
re-kicking before the slider re-sticks, the belt never unloads, the clacking goes
away, motion is continuous.

**(c) The re-stick clock — the finding that bounds the whole feature.** Inertia is
tiny (`m≈1.22e-5`), kinetic friction is large (`fk≈0.55`), so a drive-cut slider
decelerates at **fk/m ≈ 4.5·10⁴ %/s²**. Stop time and coast distance after a cut:

| v at cut [%/s] | 30 | 50 | 80 | 120 | 165 | 200 |
|---|---|---|---|---|---|---|
| stops in [ms] | 0.7 | 1.1 | 1.8 | 2.7 | **3.7** | 4.4 |
| coasts [%] | 0.01 | 0.03 | 0.07 | 0.16 | **0.30** | 0.44 |

(165 %/s is the measured mini-flight peak of a 10 ms/0.8 kick, findings §2.)

**Consequence 1 — "never stop" has a speed FLOOR.** To re-kick before re-stick,
the gap between one kick's cut and the next kick's start must be < the stop time.
The smallest kicks the plant permits are ~3–5 ms (a 3 ms fwd kick ≈ +0.2 %; 3 ms
is dead at 78 %, §4c). Working the cadence:

| kick | v_peak | re-kick within | cadence | travel/kick | never-stop avg |
|---|---|---|---|---|---|
| 3 ms | ~50 %/s | 1.1 ms | ~240/s | 0.2 % | **~50 %/s** |
| 5 ms | ~80 %/s | 1.8 ms | ~150/s | 0.4 % | ~60 %/s |
| 10 ms | ~165 %/s | 3.7 ms | ~73/s | 1.8 % | ~130 %/s |

So the **slowest speed at which this slider never stops is very roughly
40–60 %/s** (rig-measured, HIL). Below that, braking-between-kicks *cannot* keep it
rolling — it re-sticks between the smallest kicks you can fire. **Never-stop is
therefore FASTER than today's 6–35 %/s stop-start CRAWL.** "As slow as possible
*while never stopping*" = **run at this floor**, and the floor is a per-direction,
learnable optimum — arguably the truest reading of the ask. Rates slower than the
floor need §10's stretches (coast-between-kicks or closed-loop cruise) or accept
stopping (fall back to CRAWL).

**Consequence 2 — slow buys first-shot, for free.** Coast after the final cut is
`v·(v·m/fk)/2` — **0.03–0.3 % at glide speeds**, i.e. essentially zero. So the
landing is *not* a ballistic-coast problem; it is dominated by the **last kick's
own travel** (~0.2–0.4 %). Nailing first shot = **size the last kick(s) so
`kick_travel + tiny_coast = remaining`, biased to arrive just below target, then
one forward seat.** Slower glide → smaller final kick → tighter landing. The two
goals are synergistic at the landing even though they trade off at cruise.

**Consequence 3 — v̂ cannot drive this; p̂-progress must.** Velocity resolution at
rest is ±10–20 %/s and the observer *snaps v̂ to 0* below `obs_v_stick`=3 %/s
(re-stick rule `observer.h:89-97`, stuck latch `:67-72`). A 30–60 %/s glide sits
near that floor and v̂ degrades exactly in the creep/re-stick regime it would live
in (`plant-model.md:282-288`). So GLIDE's "still rolling?" test must be a
**windowed p̂-progress rate** (the exact pattern `progress_update` already uses for
stall, `controller.cpp:176-188`), **never v̂**. This copies invariant T4.

## 2. Design overview

GLIDE is a new controller state (§3 argues new-state vs CRAWL-submode) built from
three ingredients over the reused CRAWL pacing core:

1. **Keep-rolling cadence** — fire the next kick *before* p̂-progress decays to the
   re-stick floor, so position advances monotonically and never stalls. Between
   kicks the output is still exactly 0 (T16 preserved — it is the *cadence* that
   changes, not the between-kick drive). Runaway-guarded.
2. **As-slow-as-possible** — self-tune kick size / cadence *down* to the smallest
   that still keeps p̂-progress above the roll-floor. Clamped at the physics floor
   (§1); a request below the floor is handled per Gate Q1.
3. **Coast/kick-sized landing (nail first shot)** — as the target nears, ramp the
   kick size so the last kick's travel + coast lands just *below* target, then a
   single forward seat closes tol. Predictive, not hunt-and-peck. Undershoot-biased
   → preserves the forward-landing invariant (T7), no rev limit cycle.

**Self-learning** threads through all three (§7): the existing bounded-EMA width
learner (`crawl_w_scale_`) extended to also learn the **coast/stop distance** (=
auto-ident datum #7, `auto-ident-spec.md:49`) and the **cadence floor**, per
direction, optionally persisted to NVS `UnitCalib`.

## 3. Reuse map and the new-state decision

Reused as-is (all in `controller.cpp`, confirmed present):
- **Ratchet pacing** `ratchet_ += d·rate·DT` + lead clamp (`:499-506`).
- **PULSE machinery** `start_pulse(w, dir, origin, p_hat)` / `tick_pulse` with
  `pulse_origin_` return routing (`:283-299`, `:375-380`).
- **Self-adjust EMA** `crawl_w_scale_`, ratio-clamped [0.25,4], bounds
  [`crawl_w_scale_min`,`max`] (`:549-561`).
- **Grab/override guards** drag (`:510-514`) + zero-travel (`:533-544`), watchdog
  (`:110-117`), `crawl_degraded` honesty flag (`:516-524`).
- **Shrink-only landing scale** lesson (`:435-448`) — a landing kick may shrink,
  never grow (learned scale inflates near target and over-delivers).

New and not present today:
- **v̂ / p̂-delta into the tick.** `tick_crawl(float p_hat)` has *no* velocity input
  and no `p_prev` (`controller.h:85`). GLIDE needs a **windowed p̂-progress**
  signal in-tick (add a member ring like `ph_ring_`, or reuse it) — *not* v̂.
- **Keep-rolling fire rule** (fire before re-stick, §5) vs CRAWL's fire-after-settle.
- **Coast/kick-sized landing geometry** (§6) vs CRAWL's hand-off-at-`crawl_land_zone`.

**Recommendation: a new `State::GLIDE = 6`, reusing the cores above.** Rationale:
GLIDE's fire rule and landing are different transitions with their own invariants
(never-stop, coast-cut) that deserve their own tripwires; folding them into CRAWL as
another default-off flag (the `crawl_step` pattern) would entangle T16/T17's negative
controls. This new-state-vs-submode choice is **Gate Q2** — if Jan prefers the
lighter submode route, the rows below collapse into CRAWL rows 26–29 with guards.

## 4. Trigger-graph rows (proposed — needs Jan's OK)

New state **GLIDE** = paced *continuous* sub-cliff kick train toward a distant
target, re-kicking before re-stick. Same trigger sources as `controller-spec.md`
§2 (`CMD/TICK/OBS/ILK/TIME`). Rows numbered G-prefixed; each cites the CRAWL row it
parallels.

| # | From | Trigger | Guard | To | Action |
|---|---|---|---|---|---|
| G1 | IDLE | CMD set_target, `glide` requested | `|err| > coast_zone` | GLIDE | ratchet := p̂; rate := clamp(v_req, [glide_floor, v_crawl_max]); progress window reset (parallels row 26) |
| G2 | GLIDE | TICK kick due: p̂-progress ≤ `v_roll` OR gap ≥ `glide_max_gap_ms` | remaining > coast_zone | PULSE | width from table for the self-tuned step × `crawl_w_scale_`, domain-clamped; origin = GLIDE (parallels row 27, but fires *before* re-stick, not after settle) |
| G3 | PULSE | TICK width elapsed | origin = GLIDE | GLIDE | **keep-rolling settle** = min(`glide_settle_ms`, headroom to re-stick); `crawl_adapt_pending_` := true (parallels row 16) |
| G4 | GLIDE | TICK remaining ≤ `coast_zone` (= predicted last-kick travel + coast) | — | WAIT | **coast-cut**: emit the final sized kick if one is due, else u:=0; `crawl_landing_` := true; enter WAIT(1) → forward seat via `endgame_decide` (parallels row 28, but predictive) |
| G5 | GLIDE | TICK progress-dropout `glide_grab_pulses` kicks with p̂-progress ≈ 0, OR `|p̂−ratchet| > crawl_override_err` | — | RELEASED | reason=grab; u:=0 next tick (parallels row 29 / rows 4,7,12; hand blocks or drags — never fight) |
| G6 | GLIDE | ILK persistent > `ilk_yield_ms` | — | RELEASED | reason=interlock (row 18) |
| G7 | GLIDE | TIME watchdog (armed off the **achievable** rate, not requested — §8) | — | RELEASED | reason=watchdog; diagnostic (row 19) |
| G8 | MOVE/BRAKE/PULSE/WAIT/CRAWL/GLIDE | CMD set_target (new) / release / abort | — | BRAKE/RELEASED | as rows 8 / 23 (GLIDE is not special here) |

Honesty (T8 family, extends `crawl_degraded`): if the achieved rate stays
< `glide_degraded_frac`·requested over the window **despite** the cadence being at
its tightest (kicks back-to-back), set `glide_degraded` and continue — never a
silent slow-down, never a false grab from an honest slow crawl.

## 5. Keep-rolling fire rule (never stop) — detail

Per tick in GLIDE, after any active kick returns to GLIDE (G3):
- Maintain a short p̂-progress window `prog = |p̂_now − p̂_(N ago)|·FS/N` (the
  `progress_update` idiom, but a **short** N so it tracks the sawtooth, e.g. 8–20
  ticks — HIL-tuned; a longer N goes slower but adds landing latency, §1c).
- **Fire when** `prog ≤ v_roll` (about to re-stick) **or** `gap ≥ glide_max_gap_ms`
  (hard cadence cap, safety) — whichever first. This replaces CRAWL's
  "settle elapsed AND deficit ≥ frac·expected" (`:526-532`).
- **Runaway guard:** if `prog ≥ glide_runaway_v` (a kick over-delivered into a
  mini-flight), *skip* the next kick and let it coast down — bounded, and it feeds
  the self-tuner to shrink the step.
- Between kicks: **output is exactly 0** (reuse the `hold_duty≡0` return paths,
  `:485`,`:528`,`:568`). Keep-rolling changes *when* the next kick fires, not the
  zero-drive-between-kicks invariant. (T16 preserved — see §9.)

Why this is the whole game acoustically: today the slider re-sticks and the belt
unloads ~12×/s (the clack). Firing at `prog≈v_roll` means the next kick lands
mid-motion, the belt never fully unloads, and the kicks needed to *sustain* are
smaller than to *start* (§1b) → smaller, more regular, quieter. SIL can prove the
*geometry* (progress never hits the stuck floor); *quietness is Jan's ear* (HIL).

## 6. Coast/kick-sized landing (nail first shot) — detail

As `remaining = (target − p̂)·d` shrinks toward `coast_zone`:
- **Ramp the kick size down** so the final kicks aim `min(remaining − coast −
  seat_margin, glide_step)` rather than a fixed step — the last kick lands the
  slider **just below target** (approach-from-below, both directions).
- `coast = glide_coast_k · prog²` (or a learned table; §1 gives ~0.03–0.3 %), a
  **small correction**, self-learned per direction (§7), **undershoot-biased**
  (bias factor like `impulse_aim`=0.90) so a miss is always short.
- At `remaining ≤ coast_zone` (G4): stop driving; hand to WAIT with
  `crawl_landing_`=true. The existing **forward seat** (`endgame_decide` shrink-only,
  `:435-448`) closes the last ≤`glide_step` with one gentle pulse.
- **Direction reconciliation with T7/#4 (forward landings only):** always arrange
  to arrive *just below* target and seat *forward* onto it. For a fwd move this is
  natural (undershoot). For a rev move, aim the coast a hair *past* target (slightly
  below it) so the final motion is again a forward seat — the same "rev to
  target − reapproach, then forward" geometry rows 14/15 already use. **No invariant
  relaxation needed**; the reverse limit cycle that Option B fought
  (findings §8) is avoided by construction because the coarse hunt is replaced by a
  single sized seat.

"First shot" target: **coast + ≤1 forward seat** lands within tol. Pure-zero-pulse
landing (no seat) is riskier (no safety net on the coarse actuator) — that trade is
**Gate Q3**.

## 7. Self-learning parameters

Reuse the shipped bounded-EMA idiom (`crawl_w_scale_`, ratio-clamped, bounded,
noise-floor-skipped, `:549-561`) — the "online complement to auto-ident" the crawl
findings §5 already frame. Extend the *learned set*:

| Learned quantity | From | Update | Bounds/safety | Persists? |
|---|---|---|---|---|
| kick width scale (exists) | `|p̂−p_at_pulse_|` per kick | bounded EMA (as today) | [0.5, 2.0]×, shrink-only at landing | per-move today; Gate Q4 |
| **cadence floor / `v_roll`** | did it re-stick? (progress dropout after a kick) | shrink step until dropout, back off | never below physics floor; runaway guard | Gate Q4 |
| **coast distance `glide_coast_k`** | measured overshoot of each landing vs predicted | bounded EMA, per direction | undershoot-biased; = auto-ident #7 datum | Gate Q4 |

- **Draws from** NVS `UnitCalib` impulse table (#8) for the initial `pulse_width_for`.
- **Could feed back** a persisted per-direction step/coast scale (today
  `crawl_w_scale_` resets to 1.0 each move, `:348`) → the step from *per-move online
  correction* to *the unit tunes its own glide*, closing the loop with auto-ident
  Goal #5. Persistence is **opt-in and validity-stamped** (`auto-ident-spec.md:36`)
  — Gate Q4, because unbounded persisted learning on a drifting plant is a footgun
  (the shrink-only lesson writ large).
- Honesty: if learning cannot reach the requested slow rate, or the coast keeps
  missing tol, `glide_degraded` flags it (never faked).

## 8. What SIL can and cannot prove (and the watchdog fix)

The plant sim models the load-bearing structure — **kinetic≠static, re-stick,
moving-kick-skips-breakaway, the unstick cap** (`plant_sim.h:76-114`) — so GLIDE's
**logic/geometry is SIL-testable in the existing idiom**. It does **not** model
stick-slip creep, the 0.3–1.3 % post-pulse creep, acoustics, or a real coast
runway (fk dominates → sim coast is ~4 ms/0.4 %). So:

- **SIL proves (tripwires §9):** never hits the stuck floor mid-cruise; zero duty
  between kicks; monotone approach; lands within tol with ≤1 forward seat; no rev
  landing; grab/override release; learning bounded & stable; termination; honest
  degraded flag.
- **HIL proves (Jan + rig):** the actual never-stop *feel*, the acoustic win vs
  today's clack (score with `analysis/servo-hil/crawl_smoothness.py`, baseline
  roughness 6.65 / kept-rolling 14.7 %), the real coast distance, the first-shot
  hit rate, long-run drift. Same fast/slow split as A4s (`plan.md`).

**Watchdog:** arm G7's watchdog off the **achievable** rate (or scale it), not the
requested rate — the known bug where an honest slow crawl runs past a
requested-rate budget and self-releases (findings §2, §8 tail). GLIDE sidesteps it
partly by sustaining rate, but the watchdog must still be armed honestly.

## 9. Invariants — GLIDE honors the existing set, adds its own

GLIDE deliberately **breaks no existing invariant** (this is the low-risk story):
- **T16 (zero duty between pulses):** preserved — keep-rolling changes cadence, not
  the between-kick output (§5). New tripwire G-T2 extends T16 to GLIDE.
- **#3 / T2 (no sub-cliff cruise; impulses only):** preserved — GLIDE is a pulse
  train.
- **#4 / T7 (forward landings only; overshoot costs a rev re-approach, never a rev
  landing):** preserved by approach-from-below + forward seat (§6).
- **#9 (no torque at rest):** preserved — GLIDE acts only while commanded moving.
- **T4 (stall/progress from p̂ window, never v̂):** GLIDE's fire rule obeys it (§1c).

New tripwires (each: SIL assertion + seeded-bug negative control, `sil_bugs.h` +
`analysis/tests/test_sil.py` `_both` pattern):

| # | Tripwire | Negative control that must break it |
|---|---|---|
| G-T1 | In GLIDE cruise (remaining > coast_zone), a short p̂-progress window never falls to the stuck floor for > `glide_max_gap_ms` | revert fire to CRAWL's wait-full-settle → progress dips to stuck |
| G-T2 | GLIDE emits exactly 0 duty between kicks (T16 extended) | inject an inter-kick hold duty |
| G-T3 | GLIDE reaches tol with ≤1 forward seat, monotone approach, **no rev landing** | cut at a fixed zone ignoring coast, or allow a rev seat |
| G-T4 | Learned coast/step scale stays in bounds; landing scale shrink-only | uncap the landing scale → over-deliver → overshoot |
| G-T5 | `glide_degraded` set iff achieved < frac·requested at tightest cadence; never silent | clear the flag before latch |
| G-T6 | Progress-dropout with a hand, or drag past ratchet, → RELEASED, u≡0 next tick | drop the dropout/drag check |

## 10. Parameters (new, all default-off → SIL byte-identical until set)

Mirrors how `crawl_step`/`crawl_settle_ms`/`crawl_land_zone` shipped disabled.
Add to `ServoConfig` (`servo_params.h`), wire `config`-directives in
`sim_runner.cpp` (`:73-85` pattern), expose a subset as HA numbers
(`slider_servo.h:113-118` pattern).

| Param | Default (off) | Meaning | Basis | HA? |
|---|---|---|---|---|
| `glide` | 0 | enable GLIDE for this move (0 = ordinary CRAWL/MOVE) | mode gate, like `crawl_step` | yes |
| `glide_floor` | 40 %/s | min never-stop rate; requests below → Gate Q1 | §1 physics floor (rig-tune) | yes |
| `v_roll` | 30 %/s | fire when p̂-progress ≤ this (about to re-stick) | just above the 25 %/s progress noise floor (`stall_v`) | yes |
| `glide_max_gap_ms` | 8 | hard inter-kick cap (safety/runaway) | ~2× worst re-stick time (§1) | yes |
| `glide_settle_ms` | 0 | keep-rolling settle; 0 = auto (headroom-to-re-stick) | §1 stop-time table | yes |
| `glide_runaway_v` | 200 %/s | skip next kick above this (mini-flight guard) | > measured 165 %/s clack peak | no |
| `glide_step` | 0.4 % | target per-kick travel (reuses `crawl_step` semantics) | S2 crawl value that Jan's ear picked | yes |
| `coast_zone` | 0.8 % | begin coast/kick-sized landing within this | last-kick travel + coast (§1,§6); ~`crawl_land_zone` | yes |
| `glide_coast_k` | 0 | coast ≈ k·prog² (0 = learn from zero) | auto-ident #7 | no (learned) |
| `glide_coast_gain` / bounds | 0.3 / [0.5,2]× | coast-learn EMA gain + bounds | mirror `crawl_w_gain` / `crawl_w_scale_*` | no |
| `glide_coast_bias` | 0.9 | undershoot bias (arrive just below target) | `impulse_aim` | no |
| `glide_grab_pulses` | 3 | progress-dropout kicks → grab | mirror `crawl_grab_pulses` | no |
| `glide_degraded_frac` | 0.5 | achieved < this·requested → flag | mirror `crawl_degraded_frac` | no |
| `glide_persist` | 0 | persist learned coast/step to NVS across moves | Gate Q4 (off = per-move) | yes |

Reused unchanged: `crawl_impulse_duty` (0.80 — the reverse-breakaway-safe duty,
findings §2), `crawl_override_err`, `crawl_w_scale_*`, `pulse_min_ms`,
`crawl_max_width_ms`, watchdog, interlock.

## 11. Phased plan (each stage has a gate; every flash needs Jan's OK)

- **Stage 0 — spec (this doc).** Gate: Jan approves the §4 rows (Q1–Q4). Then
  `plan.md` gets a GLIDE phase entry and `controller-spec.md` §2 gains the rows.
- **Stage 1 — keep-rolling cadence (never stop).** New `State::GLIDE`, p̂-progress
  fire rule (§5), default-off. SIL: G-T1, G-T2 + negative controls; sim_runner
  `config glide …` + a GLIDE scenario; SIL stays green and default path
  byte-identical. **Flash gate → Jan's ear:** does it roll continuously? quieter
  than the clack? Score vs roughness 6.65.
- **Stage 2 — coast/kick-sized landing (nail first shot).** §6 geometry + forward
  seat; per-move coast learning. SIL: G-T3, G-T4. Rig: first-shot hit rate, rev
  has no limit cycle. Re-run `a1_sweep.py` to confirm A1 (fast moves) untouched.
- **Stage 3 — self-learning persistence.** Persist coast/step per direction to NVS
  `UnitCalib`, bounded + validity-stamped (Gate Q4); feed auto-ident #7/#8;
  long-run drift on the rig. SIL: G-T5 (honest degraded), stability of the learner.
- **Stretch (separate gate) — sub-floor continuous motion (§12).**

## 12. Out of scope for v1 (explicit stretches, if Jan wants sub-floor slow)

To go **slower than the ~40–60 %/s never-stop floor while still never stopping**,
braking-between-kicks is not enough (§1). Two options, both bigger and riskier,
deferred behind their own gates:

- **Coast-between-kicks (medium risk).** Between kicks, put the H-bridge in
  **high-Z/coast** instead of brake-low (shorted) so the slider glides on mechanical
  friction alone (no electrodynamic brake) and stays moving far longer → a much
  lower never-stop floor. Needs a coast PWM phase (L9110 both-inputs-low), is **not
  sim-modelled** (HIL), and nudges T16's meaning ("zero duty" vs "zero *torque*,
  coasting"). The cleanest way to genuinely go slow *and* smooth.
- **Closed-loop kinetic-branch cruise (higher risk).** Hold a modulated sustain
  duty on the kinetic branch (`u ≈ F_k + b·v_des` + observer damping) — truly
  continuous velocity, the thing `plant-model.md:383-384` calls "*why* the servo
  needs the observer." Rides on F_k's ±10 % spread (open-loop landing 96 % vs 52 %,
  §8a) and low-speed unobservability; breaks #3/T2/T16. A real research sub-project,
  not a v1 knob.

## 13. Gate questions — ANSWERED (Jan, 2026-07-17)

1. **How slow?** → **(a) run at the never-stop floor.** Not-stopping is the
   priority; "reasonable slow" = the floor. A request below the floor **rounds up**.
   No sub-floor stop-start fallback in v1 (§12 stretches remain deferred).
2. **New state or submode?** → **new `State::GLIDE`.**
3. **First-shot definition?** → **coast + ≤1 forward seat; landing tolerance
   accepted, judder is NOT.** The landing must not hunt, limit-cycle, or belt-flip
   — a single gentle forward seat, undershoot-biased (§6). Judder-freedom is the
   acceptance bar, tolerance residual within it is fine.
4. **Self-learning persistence?** → **persist to NVS**, across moves, per direction
   — bounded, validity-stamped, undershoot-safe / shrink-only at landing (§7).
