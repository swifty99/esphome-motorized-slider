# Auto-Ident Spec (DRAFT) — every slider identifies itself

> „the ladder has one more step: get the parameters for model and control **on
> device**. each slider is different. (i have more and different)" (Jan,
> 2026-07-16)

Goal: a production `slider_servo` unit derives its own model/controller
parameters — **no scope, no host, bounded time** — and stores them per
exemplar in NVS. Scope-bound measurements (current spikes, PWM coupling
waveforms, ADC INL) are design-time knowledge and must never be required in
the field. This draft is grounded in what the 2026-07-16/17 fits actually
needed; per parameter it names the bench method used tonight, the on-device
method, and the storage.

## Principles (learned the hard way tonight)

1. **Movement detection must beat the noise floor.** Per-tick wiper noise is
   ~1.2 % p-p idle and ~3 % p-p with PWM (spikes). Run 58 (thr 0.3 %) and run
   01/07-17 (thr 1.5 %) false-triggered instantly; **threshold 5 %** with
   step 0.006/20 ms and hold 50 ms worked for all 34 valid M4 trials. Any
   on-device detector: threshold ≥ 5 % raw, or ≥ 1.5 % on a ≥ 8-sample median.
2. **Log the onset, not the termination.** The ramp-termination duty is up to
   +0.05 late. On device: remember duty at the *last quiet tick* before the
   movement threshold tripped (a 32-entry ring of (duty, pos) is enough) —
   same refit the offline pipeline does.
3. **Prefer fits that are sums, not matrices.** Everything below reduces to
   medians/linear 2×2 solves — no least squares library on the ESP32 needed.
4. **The duty→velocity map is a cliff, exploit it.** Fine ladders
   (plant-model §2b) show ≤1 %/s below breakaway and ≥100 %/s one 0.02-step
   above — there is no creep curve to measure. Breakaway ident is therefore a
   BINARY search problem: "did it move ≥5 % within 300 ms?" is unambiguous,
   noise-proof, and needs no velocity estimation at all.
5. **Respect the duty budget**: ≤ 0.5 average |duty| per 10 s (thermal
   interlock stays live during ident). Full self-ident below fits in
   ~60–90 s including pauses.
6. **Every stored parameter carries a validity stamp** (fw version, VM,
   endpoint calibration id, date counter) — a re-cal invalidates dependents.

## Parameter table

| # | Parameter | Bench method (tonight) | On-device method | Time | NVS (bytes) | Tonight's fit on-device-capable? |
|---|---|---|---|---|---|---|
| 1 | Endpoint calibration (raw@0 %, raw@100 %) | provisional constants | endstop_safe slow drive to both rails, median of 200 ticks each | 6 s | 8 | YES (existing plan) |
| 2 | Sensor σ₀ (quiet) | M1 noise_floor, 3 positions | 2 s idle recording, per-tick |x−median|, store MAD·1.48 | 2 s | 4 | **YES** — same math |
| 3 | Sensor σ under PWM + spike rate | stalled pwm_coupling + ladder rungs | 2 s stall at duty 0.30 (below any observed breakaway... use 0.45 cap), same MAD + count of >5σ ticks | 2 s | 8 | **YES** |
| 4 | Breakaway map F_s(p, dir), 5–7 positions × 2 dir × 2 reps | M4 breakaway_grid ramps + offline onset refit; fine ladders 45/48 established the cliff | **coarse bracket + step probes, NOT a fine ramp** (see below): discrete 300 ms holds from 0.60 upward in 0.03 steps until the binary moved-≥5 % classifier fires; first hit = F_s within +0.03; 2 reps beat finer steps (trial spread is ±0.04 anyway) | ~25 s incl. pauses (was ~40 s as ramps) | 2×7×2 = 28 | **YES** — this is the flagship on-device ident |
| 5 | Sustain flag per point (Stribeck/band map) | M4 hold_dpos analysis | after breakaway, hold 50 ms, store "sustained ≥ 3 %?" bit; bands with re-stick get the FF "no marginal cruise" flag | free (same runs) | 2 | **YES** |
| 6 | Kinetic level + damping (F_k mid, b) per direction | 13k-sample LSQ over all runs | 2 steady flights per direction mid-travel (e.g. 0.72 and 0.80, 25→75 %): terminal v from last-40-ms slope → 2×2 solve: u = F_k + b·v | 4 runs ≈ 8 s | 16 | **PARTIAL** — device gets mid-travel level + b; the 11-node position map stays an offline product |
| 7 | Inertia m | sim-domain refit, located only ×3 | **skip on device.** τ = m/b ≈ 15–40 ms ≪ any move; FF/observer tolerate the offline default. If ever needed: coast-stop distance after duty cut at known v | — | 4 (default) | NO — and doesn't need to be |
| 8 | Impulse table (travel per pulse duration @ 0.8, per direction) | M7 impulse_ladder runs 39/41/42: measured table 2–50 ms, ±10–15 % repeatable (plant-model §4c) | the M7 experiment IS the on-device method: fire 3/5/10/20/30/50 ms pulses from mid-travel, 2–3 reps, store medians; sim under-predicts by 25–35 % → measure, don't model | ~15 s | 2×6×2 = 24 | **YES — proven tonight**, the flagship FF parameter |
| 9 | Backlash width | M6 reversal/control pairs: Δ dead time −1..+2 ms at 0.75 → no resolvable dead band (plant-model §4b) | **not needed per unit** at this belt tension; if a future unit thunks: same reversal-pair experiment, flag if Δ > 5 ms | (5 s if run) | 1 flag | YES (as a cheap health check, not a parameter) |
| 10 | Torque constant K, motor R | **MEASURED** (M3 2026-07-17: R = 8.2 ± 0.8 Ω, K = 0.36 ± 0.20 mV/(%/s), plant-model §7) — needed shunt + scope + waveform-level reasoning (the module-GND shunt only sees the ON phase) | **NOT identifiable on device**: no current sense exists on the unit (L9110 has no sense pins; the shunt is bench equipment), and even with one, the DCM curvature needs waveform access. Consequence: K/R are **per-design constants in the firmware image**, refreshed on the bench per hardware revision. This is acceptable because every control-relevant quantity (breakaway, kinetic level, b, impulse table) is identified per unit in duty units directly — K/R only bridge to physical units and never enter the loop. If Phase-7 current sensing lands, R becomes on-device measurable (stall + ADC on the sense resistor); K still needs a velocity source (the pot suffices: same flight-vs-stall split, on-device feasible THEN) | — | in firmware image | NO for v1, by hardware |
| 11 | PWM coupling waveform / ADC INL | scope | design-time only | — | — | NO by design |

| 12 | Dither headroom (optional) | M8 dither_grid: Δbase ≈ amplitude, law holds at 100/200 Hz both directions (plant-model §4d) | law appears universal → nothing to identify per unit; if the controller uses dither, one confirmation probe (dithered bracket at one amp) validates the stored F_s map instead | (3 s if run) | 0 | YES (as a check; audibility decision is Jan's, not the device's) |

Total per-unit NVS: **≈ 90 bytes + stamps** — trivial.

### Why step probes instead of a ramp (learned from Block 4)

The continuous ramp (+0.005/20 ms) was designed to catch a creep onset — but
the plant has no creep regime (cliff, principle 4). A ramp therefore (a)
wastes time crawling through duties that provably do nothing, (b) logs a
termination duty biased late by the detector latency (+0.05 observed), and
(c) needs the onset refit of principle 2 to undo that bias. Discrete
settled probes with a binary classifier have none of these problems: each
probe is an independent, unbiased sample of "F_s ≤ d?", the resolution is the
step size (0.03 ≈ the physical trial spread — finer is pointless), and only
the LAST probe moves the slider (one flight + one reposition per point).
Bisection would save ~1 probe over the linear bracket but costs extra flights
when probing above threshold; the linear upward bracket is the cheapest
in moves, which is what wears the motor.

## Suggested sequence (one `self_ident` firmware program)

1. Endpoint cal (needs endstop_safe) → 2. σ₀ quiet → 3. σ_PWM stall →
4. breakaway step-probe bracket 7×2×2 (binary moved-≥5 % classifier, 0.03
steps from 0.60) → 5. two-duty flights fwd/rev (→ F_k, b) → 6. impulse table
→ write NVS + stamp. Budget ~75 s, ~25 moves, all inside interlocks. Abort rule: any step failing its own sanity check
(e.g. breakaway onset < 0.3 or > 0.9) marks the whole set invalid and keeps
the previous stamp.

## Acceptance (falsifiable)

- Repeatability: two consecutive self-idents on the same unit agree within
  ±0.03 duty on every breakaway point, ±15 % on b, ±10 % on impulse travels.
- Cross-check: offline pipeline (`plantreport`) on a recording of the
  self-ident run reproduces the device's stored values within the same bands
  (the device math is a subset of the offline math, so disagreement = bug).
- The FF demo protocol (plant-model §7) run purely from NVS parameters lands
  ±3 % — the same bar as the host-computed version.

## Open questions for Jan

1. Is duty 0.45 an acceptable stall level for the σ_PWM step (needs to stay
   below the *lowest* per-unit breakaway with margin — tonight's lowest
   observed was 0.56 fwd @ 90 %)? Alternative: mechanical block requirement
   at ident time (bench-only), or measure σ during the breakaway ramps' quiet
   phase instead (free, slightly less controlled).
2. May self-ident use endstop_safe (endpoint cal + rail-adjacent grid points
   need it)? If not, the grid shrinks to 10–90 %.
3. Where does the impulse table live: NVS raw, or folded into the FF profile
   generator's coefficients?
