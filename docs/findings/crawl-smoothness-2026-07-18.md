# Crawl smoothness research — "the noise thing", round 2

Mandate (Jan, 2026-07-18): the baseline is excellent and the data is good, but
**the CRAWL is noisy** and there is **belt tensioning/untensioning** we can hear.
Goal: *smooth operation with self-adjusting parameters — no cooldown dwell.*
This is the research pass: the physics wall, what the bench actually does today
(fresh dumps + the new metric tool), the SIL-verified fixes, and a flash gate.

Tools: `analysis/servo-hil/crawl_smoothness.py` (objective roughness metric, new),
`crawl_probe.py` pattern for dumps. Loop + knobs: `docs/handover-tuning-2026-07-18.md`.
Physics: `docs/plant-model.md` (§2b cliff, §4c impulse table, §4d dither).

## 1. The physics wall — why "smooth = continuous" is impossible

The duty→velocity map is a **cliff, not a ramp** (plant-model §2b): ≤1 %/s creep
below ~0.66–0.68 duty, ≥100 %/s *flight* above it, transition <0.02 duty. **No
proportional slow-cruise regime exists** — the slider is stuck or flying, nothing
between. Dither does **not** help (§4d, M8): breakaway still needs peak duty to
reach the undithered threshold. So **slow motion must be synthesized from
impulses** — the CRAWL pulse train is not a hack to replace with a smooth drive,
it is the only tool the plant permits below ~120 %/s. "Smooth" can therefore only
mean **shaping the impulse train**: smaller, more regular, better-timed kicks.

The belt is positionally tight (M6 §4b: backlash ≤1–2 ms, below resolution), so
"belt tensioning/untensioning" is **not** slop — it is the elastic load/unload of
each kick: the motor breaks the slider away (belt tensions), it flies, the pulse
cuts and the motor shorts to brake (belt recoils/unloads), it re-sticks. That
load→unload→re-stick cycle, ~12×/s, is the sound.

## 2. What the bench does today (fresh dumps, firmware e28f965 + crawl 0.72)

Three CRAWL dumps, 2026-07-18, no reflash, scored by `crawl_smoothness.py`.
Roughness = clacks/s × median per-ms decel of |v̂| (jerk proxy); higher = worse.

| dump | dir | outcome | clacks/s | med step | peak\|v̂\| | roughness | kept-rolling |
|---|---|---|---|---|---|---|---|
| 15 %/s | fwd 48→88 | **reached** | 12.3 | 0.85 % | 165 %/s | **6.65** | 14.7 % |
| 6 %/s  | rev 88→12 | **FALSE-RELEASE @84 %** (grab) | — | — | ~0 (stuck) | — | 0 % |
| 35 %/s | rev 88→12 | **FALSE-RELEASE @28 %** (watchdog) | 5.8 | 1.19 % | 148 %/s | 3.22 | 7.6 % |

Three problems, and the acoustic one is the *least* urgent:

- **Reverse crawl is unreliable — this is the headline.** Both reverse dumps
  released with no hand on the rig. Root causes are *different* and both real:
  - **6 %/s (from the top):** 14 pulses at −0.72 duty produced **zero measured
    travel** → the slider looked stuck → **grab** false-positive. Cause: **0.72 is
    below reverse breakaway near the high end** (rev breakaway there ~0.67–0.75,
    plant-model §2). The acoustic softening 0.80→0.72 (commit e28f965) quietly
    **broke reverse crawl** at the extremes. Nobody had A/B'd reverse at 0.72.
  - **35 %/s:** the crawl under-delivers (~7 vs 35 %/s requested) because cadence
    is settle-limited (§3), so the move runs ~5× past its **watchdog** budget
    (armed for the *requested* rate) → watchdog release. A rate-honesty gap, not a hand.

- **Acoustic (the noise):** even the clean fwd crawl fires **~12 clacks/s**, each a
  ~10 ms/0.72 kick that breaks the slider into a **mini-flight peaking 165 %/s**
  (20× the crawl rate!) then re-sticks. Motion is stop-start: |v̂|>10 %/s only
  **14.7 %** of the time — stuck 85 % of the crawl. That is the clacking.

- **Irregularity:** per-clack travel spans 0.09→1.35 % (15×) because the fixed M7
  table width delivers different travel by position/direction/dwell — uneven
  cadence sounds rough even when the gap CV looks low.

**Key reframe: soften the cadence, not the shove.** Dropping the duty (0.72) to
quiet each shove hit the reverse-breakaway floor and broke reliability. Quietness
has to come from **smaller, more frequent, regular steps at a reliable duty**, not
from a weaker kick.

## 3. The governing identity: step ≈ rate × (settle + width)

Per clack the slider advances `step ≈ crawl_rate × period`, `period = settle +
width`. In the 15 %/s dump: 12.3 clacks/s → period 81 ms = **70 ms settle + ~10 ms
width**, step 0.85 % ✓. **The settle dominates the period**, so:

- `crawl_max_width_ms` only *caps* the step; it bites only at rates high enough to
  demand >cap-width pulses. At 15 %/s the crawl already self-selects ~10 ms pulses,
  so lowering the cap barely changes the sound. **The naïve "just cap the width" idea
  is weak.**
- Smaller quieter clacks at the same rate ⟺ **shorter settle**. The crawl settle is
  `impulse_settle_short_ms`, shared with the endgame — so the honest knob is a
  **crawl-exclusive settle** (added below, SIL-neutral).
- Free consequence: a **slower forward** rate already gives smaller clacks (same
  ~13/s cadence, smaller step). Worth Jan trying `Servo Speed` low forward today.

## 4. What was built this session (SIL-neutral, default-off, ships disabled)

All in the pure-C++ core + wrapper; **SIL stays 30/30 and the default path is
byte-identical** (proven by diffing sim_runner CSV fwd+rev vs the pre-change
build). Nothing changes on the rig until a YAML value is set.

- **C2 — self-adjusting step** (`crawl_step`, `crawl_w_gain`). When `crawl_step>0`,
  each pulse aims a constant small travel and the width self-corrects online from
  the *measured* travel of the previous pulse (bounded EMA). Regular quiet steps,
  and robust to the per-unit / per-direction impulse table being wrong — the
  *self-adjusting* the mandate asks for, and the online complement to auto-ident
  (§5). Default `crawl_step 0` = legacy dosing.
- **C2b — crawl-exclusive settle** (`crawl_settle_ms`). The §3 period lever: a
  shorter crawl settle tightens the cadence and shrinks the step at a given rate,
  **without** touching the endgame settle A1 depends on. Default `0` = the endgame
  short/long split (byte-identical).
- **sim_runner** gained `config crawl_step` / `config crawl_settle_ms` so SIL can
  A/B crawl configs without a recompile.

**SIL A/B (fwd crawl 20→60 @ 20 %/s), mechanism confirmed:**

| config | pulses | landed | crawl_degraded |
|---|---|---|---|
| legacy | 37 | 59.64 | 0 |
| `crawl_step 0.5` | 71 | 59.82 | **1** (small steps under-deliver at 70 ms settle) |
| `crawl_step 0.4` + `crawl_settle_ms 30` | 76 | 59.58 | **0** (tight settle restores rate) |

So C2 makes the steps small/regular and C2b's tight settle pays for the rate — the
pair delivers smaller regular steps that still keep up. (SIL proves it lands,
sustains rate, and stays green; it models neither acoustics nor stick-slip creep,
so *quietness* is the rig + Jan's ear, not SIL.)

## 5. Self-adjusting north star

`crawl_step`'s online width-correction is the always-on complement to **auto-ident
(axis F)**: instead of trusting a static impulse table it *learns each pulse's real
travel and corrects*. Feeding the learned per-pulse scale back into auto-ident's
NVS table is the path from hand-tuned crawl to a unit that tunes its own crawl.

## 6. Flash gate — staged for Jan (every `esphome upload` needs your OK)

Two candidates, incremental. Flash A first (proves reverse is un-broken), then B.

**A — reliability, YAML-only, no new code needed:**
```yaml
crawl_impulse_duty: 0.80   # back up from 0.72: 0.72 is below reverse breakaway
                           # near the ends -> stuck -> false grab (§2). Reliability first.
```
Test: crawl 6/15/35 %/s **forward and reverse** — every one must reach target, zero
release. Expect louder-per-shove than 0.72 but *reliable*. This alone fixes the
headline bug.

**B — smoothness on top (uses the new C2/C2b, still SIL-neutral until set):**
```yaml
crawl_impulse_duty: 0.80
crawl_step: 0.4            # constant ~0.4 % self-adjusting steps (regular, quiet cadence)
crawl_settle_ms: 30       # crawl-only tight settle -> sustains rate, no watchdog trip
```
Test: same 6/15/35 fwd&rev; then **score with `crawl_smoothness.py`** (roughness vs
the 6.65 baseline, kept-rolling should rise) and **Jan listens**. Tune `crawl_step`
(0.3–0.6) and `crawl_settle_ms` (20–40) by ear: smaller/tighter = higher-pitch buzz
vs slower clack. Watch thermal (many short 0.80 kicks; EMA should stay well under
0.3 — the sweep is self-throttled).

A1 is untouched by all of this (A1 runs at speed 0 = never CRAWL), so the endgame
scorecard cannot regress. Confirm with one `a1_sweep.py 40 1` after B anyway.

## 7. Designed-but-not-built (need rig feedback first)

- **C3 keep-rolling:** fire the next pulse while |v̂| is still above a floor (before
  full re-stick) so the slider never re-breaks-away — quasi-continuous, the belt
  never unloads. Biggest acoustic bet, needs v̂ in `tick_crawl` + a runaway guard;
  build after B tells us the re-stick impact is the remaining noise.
- **Position/direction-aware crawl duty:** floor the crawl duty to
  `F_s(pos,dir)+margin` so it stays as soft as breakaway allows but never below it —
  the principled version of "A", quiet where possible, reliable everywhere.
- **C4 endgame belt-thunk:** every fast move ends forward; an overshoot costs a rev
  re-approach = a belt load-reversal *thunk* (A1 rev overshoot 11 %). Trade a touch
  more `impulse_aim` undershoot for fewer reversals — normal A1 loop, rev-overshoot
  the gate. This is the *other* "belt tension/untension" Jan named.
- **Watchdog vs degraded rate:** arm the crawl watchdog off an achievable rate (or
  scale it) so an honest slow crawl never self-releases. Small core follow-up; B
  sidesteps it by actually sustaining the rate.
