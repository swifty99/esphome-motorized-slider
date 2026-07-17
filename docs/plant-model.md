# Plant Model — motorslider lab unit (fitted 2026-07-17, night session)

Data basis: 127 runs (final campaign state incl. Block 4), `data/2026-07-16/`
+ `data/2026-07-17/` (M1 noise, M2-CSV coupling grid in both decay modes,
M3-degraded coarse + fine duty ladders, M4 breakaway grid 34 valid trials,
M5 steps + triangles, M6 reversal pairs, M7 impulse ladders, M8 dither grid,
unstick blips as bonus impulses; M3 current ladders added in the day session,
§7). VM = 5.00 V constant, 1 kHz tick, ADC 13-bit ×4. Everything in §§1–6 is
in **duty units**: "force" means *stall-equivalent duty*. Since the M3 pass,
K and R are measured (§7) and the physical seam lives in
`plant-params.json meta.physical`; the duty-units frame remains the
controller's working frame. Force ∝ duty stays an assumption below ~0.5 duty
(§7 limits).

Reproduce everything:

```
cd analysis && uv run plantreport ../data/2026-07-16 ../data/2026-07-17 --out ../docs/findings
```

Machine-readable parameters: `docs/findings/plant-params.json` (consumed by
`slideranalysis.plantsim` / `slideranalysis.observer`). Re-run the command when
new CSVs land (M6 reversals, M7 impulse ladder, M8 dither) — every fit rescans
the directories.

Position convention: % of travel from the provisional endpoint calibration
(raw 8000 → 0 %, raw 100 → 100 %; `loadrun.py`). +duty = forward = position up.

## 1. Sensor model (ADC on the wiper)

| Quantity | Value | Source |
|---|---|---|
| σ per raw 13-bit sample | 18.9 counts ≈ 0.24 % | noise_floor runs 32/34/36 (s0..s3) |
| σ of `adc_raw` (mean of 4), motor off | **8.5 counts ≈ 0.108 %** (segment spread 4.9–12.5) | 30+ unpowered stalled segments |
| whiteness | mean-of-4 = sample σ/2 holds → white at 1 kHz; PSD flat | noise.py, M1 |
| σ of `adc_raw` under PWM (stalled) | **11.3 counts ≈ 0.143 %** pooled | pwm_coupling grid + all stalled rungs |
| coupling model σ²(d) = σ₀² + c·\|d\| | c = 121 ± 27 counts²/duty (2σ) | 60+ stalled-PWM segments, 0.05 ≤ d ≤ 0.75 |
| spike tail under PWM | p-p ≈ 130–415 counts (~1.5–5 % singles) vs 74 unpowered; the dominant PWM effect | same segments |
| quantization | 1 LSB raw / 0.25 counts adc_raw — negligible vs σ₀ | — |

**Decay mode matters (M2-CSV grid, runs 2026-07-17/57–66, stalled at 59.7 %,
duty 0.1–0.5 both modes):** wiper p-p spread FAST 220/218/200/224/308 counts
vs SLOW 253/249/286/326/415 counts for duty 0.1→0.5. Over 17 duty- and
position-matched segment pairs across the whole session: **SLOW couples 1.19×
the p-p and 1.07× the σ of FAST, and the gap grows with duty** (at 0.5:
SLOW ≈ 415 vs FAST ≈ 308 counts p-p). Since both L9110 modes are brake-type
(no discontinuous-conduction linearity argument for SLOW without M3 current
data) and the coarse ladders showed no drive advantage for SLOW (0.70 rung:
FAST +211 %/s vs SLOW +180 %/s), **recommendation: FAST (brake-low) as the
servo drive mode** — it is the quieter one on the only channel the servo has.
The spike tail (not broadband σ) is the dominant coupling artifact in both
modes. The spike waveform is now measured and the RC decision is FINAL — see
§7: 100 nF at the ADC pin returns the sensor to its unpowered floor.

Consequences: any threshold on per-tick position (movement detectors!) must
sit above ~1.5 % or be median-filtered — the firmware breakaway detector
false-triggered at 0.3 % and 1.5 % (junk runs 2026-07-16/58, 2026-07-17/01,
kept for exactly this characterization). The observer rejects >5σ innovations
at rest for the same reason.

## 2. Static (breakaway) friction map F_s(p, dir)

Assembled from 242 evidence items (held-without-motion = lower bound;
breakaway/creep onset = upper bound; flights/traverses = range upper bounds).
M4 grid onsets are **refit from the raw ramp traces** (last quiet tick before
0.8 % displacement) because the firmware logs the ramp-termination duty,
biased late by up to +0.05. Grid step 5 %, evidence window ±8 %; table below
at every 10 %:

| pos % | 0 | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| fwd lb | — | 0.60 | 0.65 | 0.60 | 0.50 | 0.50 | 0.30 | 0.30 | 0.30 | 0.50 | 0.50 |
| fwd ub | 0.70 | 0.60 | 0.59 | 0.63 | 0.61 | 0.67 | 0.65 | 0.61 | 0.61 | 0.56 | — |
| rev lb | — | — | 0.30 | 0.30 | 0.55 | 0.55 | 0.70 | 0.72 | 0.72 | 0.75 | 0.75 |
| rev ub | — | 0.72 | 0.70 | 0.70 | 0.65 | 0.67 | 0.65 | 0.64 | 0.64 | 0.67 | 0.70 |

Plot: `docs/findings/friction_static_map.png`.

- **Reverse breakaway sits 0.02–0.06 above forward nearly everywhere** (M4).
- **The numbers are trial-to-trial variable**: 13 grid points have lb > ub
  conflicts (e.g. rev@75: held 0.72 in one trial, broke at 0.64 in another;
  M4 rev@75 scatter 0.678..0.750). Breakaway here is a *distribution* with
  ~±0.04 spread, possibly dwell-time/hysteresis dependent — a single map value
  cannot be "the" truth. The simulator uses the conservative estimate
  (max of midpoint and lower bound), so it under-predicts motion for marginal
  drives (see overlay 49 below).
- Global safe planning values: **fwd breakaway ≤ 0.72 everywhere; rev ≤ 0.79
  everywhere** (max observed + margin). Legacy 0.42 is confirmed dead wrong.

### 2b. The duty→velocity CLIFF (fine ladders, Block 4)

0.02-duty ladders (`2026-07-17/45` fwd from ~21 %, `48` rev from ~80 %,
800 ms rungs, FAST mode), pipeline-verified rung velocities:

| duty | 0.60 | 0.62 | 0.64 | 0.66 | 0.68 | 0.70 |
|---|---|---|---|---|---|---|
| fwd v [%/s] | +0.7* | +0.3 | +0.2 | +0.5 | **+120 (flies)** | (zone-parked) |
| rev v [%/s] | −0.0 | −0.0 | −0.0 | +0.0 | −0.0 | **−104 (flies)** |

(*first rung right after a goto: crept +2.7 % — breakaway is lower for
~1 s after recent motion. Recency/dwell dependence, consistent with the
lb>ub conflicts above.)

**There is no proportional creep regime.** The stall→fly transition spans
≤ 0.02 duty and jumps ~200× in velocity (≤1 %/s micro-creep below, ≥100 %/s
above). Consequences: (1) velocity-mode control by duty modulation below
~120 %/s is physically unavailable open-loop — slow motion must be
synthesized from impulses (§4c) or closed-loop torque with the observer;
(2) any identification scheme that hopes to measure a creep velocity vs duty
curve is measuring noise; a binary moved/not-moved classifier at a ~5 %
threshold is the correct primitive (auto-ident-spec #4).

## 3. Kinetic friction map, damping, inertia

Fitted jointly (linear LSQ, honest covariance) on ~15k sliding samples
(|v| > 30 %/s, zero-phase 40 Hz velocity/acceleration) from 70+ runs:

u = m·a + b·v + sign(v)·F_k(p, dir), node grid every 10 %:

| pos % | 0 | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| F_k fwd | 0.39 | 0.61 | 0.57 | 0.57 | 0.55 | 0.51 | 0.57 | 0.54 | 0.53 | 0.53 | 0.60 |
| ±2σ | .03 | .02 | .01 | .01 | .01 | .01 | .01 | .01 | .01 | .02 | .03 |
| F_k rev | 0.51 | 0.62 | 0.54 | 0.59 | 0.55 | 0.56 | 0.59 | 0.60 | 0.58 | 0.72 | 0.38 |
| ±2σ | .03 | .02 | .02 | .02 | .02 | .01 | .01 | .01 | .01 | .01 | .02 |

| Parameter | Value | Uncertainty | Notes |
|---|---|---|---|
| b (viscous + brake damping) | **5.5e-4 duty/(%/s)** | ±0.2e-4 (2σ statistical); drifted 5.1→5.5e-4 as data grew — treat as ±10 % | includes electrodynamic braking — L9110 duty 0 = motor shorted; b is NOT pure mechanics |
| m (inertia equivalent), LSQ | 1.67e-5 duty/(%/s²) | ±0.02e-5 (2σ) **plus systematic** | 40 Hz smoothing smears transients → biased |
| m, sim-refined (position-domain, first 150 ms of 39 transients) | **1.22e-5 duty/(%/s²)** | objective-within-20 % range **[0.4, 1.8]e-5** | the honest statement: τ = m/b ≈ 10–35 ms; M7 short-pulse travels suggest the true value sits in the lower half |
| residual RMS | **0.14 duty** | — | model error incl. stick-slip texture; this is the number the observer's process noise is built from |
| corr(m, b) | −0.01 | — | not degenerate thanks to coast/brake samples |

Statistical ±2σ on the F_k nodes is small (many samples); the **real
uncertainty is the 0.12-duty residual**: friction texture varies faster than
the 10 %-node map can represent, and run-to-run variability is of the same
order as the position dependence.

**Known tension (unresolved, honestly):** run 49 (0.7 rev from 80 %) broke
away, decelerated and re-stuck dead at 58.8 % — implying friction ≥ 0.70 there
at low speed — while run 52 (0.8 rev) and the −0.78 blips flew through the
same band at 250–400 %/s implying ~0.55–0.60. A constant-F_k + b·v model
cannot hold both; a Stribeck term was tried and fits *negative* (artifact:
low-|v| samples are transient-dominated and the smoothed acceleration is
biased there — see `frictionfit.fit_kinetic` docstring). Low-speed (<30 %/s)
friction is effectively **unobservable with this sensor**; the 55–80 % rev
band's "poor sustain" (M4: re-sticks within −0.5..−1.6 % in half the trials)
is real but only qualitatively modeled. M7 impulse data + slow-deceleration
experiments would pin it down.

**Decay mode:** FAST vs SLOW ladders differ by ~15 % in terminal speed at 0.70
(+211 vs +180 %/s) and slightly in onset (0.677 vs 0.645 triangle). The fit
pools both (mode is not a model input yet) — a mode factor is a one-line
addition once M3 current data exists.

## 4. Simulator and validation

`slideranalysis.plantsim`: stick-slip state machine (stuck until |u| >
F_s(p, dir); sliding per the kinetic fit; re-stick on zero crossing under
F_s; unstick-tick friction capped at 0.95·|u| which reproduces M4's
"breaks-then-immediately-re-sticks" sustain failures), hard rails at 0/100,
end-zone interlock emulation, ADC noise model. **Backlash: stub (width 0)** —
no M6 reversal-at-speed data yet. Stick-slip *creep* (the 0.65-duty
+4 %/s regime) is not modeled: the sim either stalls or slides.

Validation = overlay: recorded duty trace in, simulated vs recorded position
(plots `docs/findings/overlay_*.png`). All ~40 moving runs scored; summary:

| Class | Runs | Position RMSE | Verdict |
|---|---|---|---|
| Step responses 0.6–0.8 (45/46/47/52) | 4 | **0.6–3.0 %** | GOOD |
| Coarse duty ladders (38/43, 26 s each) | 2 | **1.6–2.6 %** | GOOD incl. 0.70 flight + end-zone cut |
| Triangles (54/56) | 2 | 3.5–4.3 % (max 22–27 % during onset) | onset duty matches; creep phase off by design |
| Unstick blips + M4/M6 recovery dithers | 25 | **0.3–8 %**, median ≈ 1.7 % | GOOD; sim travels ~10 % farther than measured on the 98 %-start blips |
| Impulse ladders (41/42 fine+rev) | 2 | 0.8–1.3 % | GOOD; per-pulse comparison in the M7 table below |
| **Failures, explained** | 49: RMSE 19.8 and fine ladder 48: 44.9 (sim refuses marginal rev drives 0.70 from 80 — conservative F_s est vs. the real ±0.04 trial spread); fine ladder 45: 7.1 (sim breaks 1–2 rungs before the real 0.68 cliff, same spread); 50: 12–16 (0.8 fwd real flies ~500 %/s, sim ~430 → cut arrives early); 39: 13.1 (endzone parking mid-run); 07/26: 21–26 (early endstop_safe runs at the extreme rails, outside the map's supported region); 09: max 53 (early run, unclear state) | | |

**Stated tolerance: for drives ≥ 0.75 |duty| and starts inside 5–95 % the
model predicts position to ~1–3 % RMSE over full moves; for marginal drives
(0.65–0.72, exactly the breakaway spread) prediction degrades to
"may-or-may-not-move" — matching the physical trial-to-trial spread.**

## 4b. M6 backlash: no resolvable dead band

Reversal-vs-control dead-time pairs at duty 0.75 (arrive from one side, then
drive against vs. with the arrival direction; `slideranalysis.backlash`):

| pos % | 25 | 50 | 65 | 75 |
|---|---|---|---|---|
| Δ dead time (reversal − control) | +2 ms | −1 ms | −1 ms | −1 ms |

Baseline dead time is 6–9 ms in BOTH cases (breakaway delay, not backlash).
**Backlash ≤ the 1–2 ms measurement resolution at this duty → the simulator's
width-0 stub is justified.** Whether reversals *thunk* audibly (A6) remains
an M8/HIL question; positionally the belt is tight.

## 4c. M7 impulse table (measured — the FF primitive)

Travel per pulse at |duty| = 0.8 from mid-travel, settle-to-settle (includes
the 0.3–1.3 % post-pulse creep the sim does not model), 3 reps each
(`slideranalysis.impulses`; runs 39/41/42 + later):

| width | fwd travel med (range) | rev travel med (range) | sim |
|---|---|---|---|
| 3 ms | +0.20 (+0.13..+0.33) | −0.28 (−0.29..−0.20) | +0.15 / −0.12 |
| 5 ms | +0.56 (+0.42..+0.57) | −0.62 (−0.67..−0.52) | +0.39 / −0.32 |
| 10 ms | +2.64 (+2.17..+2.75) | −1.02 (−1.28..−1.01) | +1.37 / −1.13 |
| 20 ms | +6.21 (+6.10..+7.97) | −4.48 (−5.49..−3.87) | +4.00 / −3.68 |
| 30 ms | +10.3 (+10.2..+13.1) | −8.15 (−10.4..−7.5) | +7.5 / −6.8 |
| 50 ms | +21.7 (+20.2..+23.2) | −20.1 (−20.2..−19.8) | +16.2 / −13.3 |

Repeatability within triples: ~±10–15 % of travel. The sim under-predicts by
~25–35 % (post-pulse creep + m sitting at the upper end of its range) — the
**measured table, not the sim, is the FF lookup**, and it is exactly what the
on-device auto-ident can reproduce (auto-ident-spec #8). Note the fwd/rev
asymmetry (rev needs ~1.3× the width for the same travel below 30 ms).

**Activity-context caveat (paired probes, runs 148–155):** this table was
measured in dense ladders (pulses every ~0.4–4 s plus recenter gotos). An
**isolated** pulse — the endgame's actual use case — delivers only ~×0.7–0.75
of these values (measured: warm isolated +1.38..+2.00 vs table +2.64 at the
same positions; train pulses 400 ms apart recover ×1.3 immediately). True
first-pulse-after-rest penalty beyond that is ≤10 % (pair ratios 0.99/0.91/0.94
at 43–47 min dwell) — the earlier "×0.5–0.7 cold malus" reading of D1–D3
conflated position and context with dwell. FF consequence: with this table the
endgame systematically undershoots isolated pulses (safe direction; escalation
covers it; settle takes longer than the SIL suggests).

**Position resolution (runs 144–147, midday session):** ladders at 22/65/78 %
plus a rev-only recheck at 50 %. **Fwd is position-independent** (10 ms med
+2.33/+2.44/+2.44 vs +2.64 mid-travel — within pulse scatter). **Rev is
position-dependent by ×1.8**: 10 ms med −1.87/−1.76/−1.84 at 22/65/78 but
−1.06 at 50 (reproduces the night value −1.02 exactly; 12 vs 6 shots, disjoint
ranges). Mid-travel is a genuine rev low-delivery spot — the table's rev
column above is valid **only near mid-travel**; the ~1.3× width asymmetry
statement holds there but shrinks to ~1.15× elsewhere. Side finding: 3 ms fwd
at 78 % is essentially dead (+0.04). Consequence: auto-ident #8 must sample
rev at ≥2 positions; the endgame lands unidirectionally fwd, so the risk is
confined to re-approach dosing.

## 4d. M8 dither: breakaway law (objective part)

Dither grid (square dither on a slowly ramped base duty, own undithered
baseline per run; Block-2 detector recipe; `2026-07-17/52–56`):

| point | baseline breakaway | dithered base at breakaway | Δbase | base+amp |
|---|---|---|---|---|
| 20 % fwd, 100 Hz / 0.10 | 0.648 | 0.576 | 0.072 | 0.676 |
| 20 % fwd, 100 Hz / 0.20 | 0.696 | 0.474 | 0.222 | 0.674 |
| 20 % fwd, 200 Hz / 0.10 | 0.672 | 0.576 | 0.096 | 0.676 |
| 65 % rev, 100 Hz / 0.10 | 0.690 | 0.624 | 0.066 | 0.724 |
| 65 % rev, 100 Hz / 0.20 | 0.666 | 0.510 | 0.156 | 0.666 |

**Law: breakaway occurs when base + amplitude reaches the undithered
threshold** (peak duties 0.666–0.724 vs baselines 0.648–0.696, within the
±0.04 trial spread). Dither buys base-duty headroom ≈ its amplitude — no
super-additive "vibration lubrication" effect at 100/200 Hz. Implications:
dither does NOT soften the cliff (the peak still crosses the same threshold);
what it offers is a *lower average torque* while waiting at the edge, i.e. a
knob for the impulse endgame to sit closer to breakaway without moving.
Whether it is audible (the reason M8 exists) is Jan's morning item — until
then dither stays optional in the controller design.

## 5. Observer (Phase-3 item 3)

2-state EKF (position, velocity), prediction = this plant model driven by
applied duty, correction = raw ADC sample, R from the sensor model
(duty-dependent), Q from the fit residual whitened over a 100 ms correlation
time, >5σ innovation gate at rest. Strictly causal, 1 kHz. Reference =
offline zero-phase (filtfilt) velocity. Legacy baseline = finite difference +
PT1 150 ms (what the old controller had). Plots `docs/findings/observer_*.png`.

| Run | KF RMS (moving) | KF lag | legacy RMS | legacy lag |
|---|---|---|---|---|
| 47 (0.7 fwd flight, ~300 %/s) | **33 %/s** | **+2 ms** | 100 %/s | +91 ms |
| 52 (0.8 rev flight, ~400 %/s) | **51 %/s** | **+3 ms** | 175 %/s | +85 ms |
| 54 (triangle ramp) | **33 %/s** | −1 ms | 100 %/s | +84 ms |
| 45 (0.6 creep — model says stuck) | 31 %/s | −13 ms | 28 %/s | −10 ms |
| 49 (re-stick run — model diverges) | 78 %/s | −12 ms | 57 %/s | +79 ms |

Verdict: **where the plant model holds, the duty-driven KF is 3–5× more
accurate than the legacy estimator with ~zero lag instead of ~85 ms** — this
is the enabler for damping at low speed. Where the model diverges (creep,
re-stick) it degrades to roughly legacy level, not worse. Velocity resolution
at rest ≈ ±10–20 %/s (set by σ₀ and Q); the legacy 9 %/s hard zero threshold
was below the honest noise floor of its own estimator.

## 6. Validity & open gaps

Valid: position 3–97 %, |duty| ≤ 0.85, VM = 5.00 V, this exemplar, this night
(no temperature/wear model). Fitted from |v| > 30 %/s; below that the model
is qualitative.

1. **K, R**: MEASURED (M3, §7). The duty-units model remains the controller's
   working frame; the physical seam is filled in `plant-params.json
   meta.physical`. Still open: force∝duty linearity below ~0.5 duty (the
   shunt cannot see the off-phase motor current, §7).
2. **Backlash**: bounded ≤ 1–2 ms dead time at 0.75 duty (§4b) — no width
   measurable positionally; audible-thunk question (A6) still open for HIL.
3. **Low-speed friction / creep / dwell dependence**: unobservable with the
   current sensor + thresholds; slow-decel experiments are the next lever.
   The 55–80 % rev sustain failure is mapped but only qualitatively modeled.
   The sim also omits the measured 0.3–1.3 % post-pulse creep (§4c).
4. **Decay-mode factor**: FAST/SLOW pooled in the kinetic fit (~15 % speed
   difference at 0.70). Mode DECISION is made on sensor-noise grounds (§1:
   FAST); per-mode torque linearity remains an M3/scope item.
5. **Position calibration**: provisional endpoints (raw 8000/100); device
   self-cal will shift the % scale slightly; maps should be refit after.
6. **ADC spike source**: 2–3 % single-tick spikes under PWM — coupling
   mechanism unknown without the scope (M2 incomplete).
7. **m**: located only within a factor ~3 ([0.6, 2.0]e-5). Fine for FF
   design (τ ≪ move duration); would matter for aggressive trajectory shaping.

## 7. Motor electrical parameters (M3, 2026-07-17 day session)

Source: 14 scope/CSV ladder pairs (`data/2026-07-17/99..132`, Ch B = 0.2 Ω
shunt in the module GND return), spike captures 133–140. Analysis:
`slideranalysis.m3current`, reproduce with
`cd analysis && uv run m3fit ../data/2026-07-17 --out ../docs/findings`.
Plots: `m3_stall_lines.png`, `m3_k_regression.png`, `m3_rc_filter.png`.
Machine-readable: `docs/findings/motor-params.json` (also merged into
`plant-params.json` as `meta.physical` — the controller never reads it; it
is the seam for physical units).

**Circuit insight that shapes everything:** the shunt sees only the PWM
ON-phase current — off-phase recirculation is bridge-internal (both L9110
modes are brake-type). Hence `I_shunt ≈ d·I_motor`, the stall line is
intrinsically curved, and the folded 10 MS/s waveform confirms it (square
0→~250 mA at duty 0.3: the motor sees the full 5 V minus ~3 V of darlington
drops during ON).

| Quantity | Value (2σ) | Method / validity |
|---|---|---|
| R (motor+shunt+wiring, ON path) | **FAST 8.5 ± 0.8 Ω, SLOW 8.0 ± 0.7 Ω** → pooled **8.2 ± 0.8 Ω** | CCM model `I_shunt = d(d·VM − V_eff)/R + c`, linear LSQ, **valid d ≥ 0.5 only** (below: DCM/recirculation, line flattens to a switching-artifact floor — R deliberately not fitted there); resid 0.8/0.9 mA on 12/14 points incl. at-stop 0.68–0.80 rungs |
| V_eff (bridge saturation + brush) | 3.1–3.5 V | same fit; **partially degenerate with c** over the narrow duty window — quote as "the L9110 eats ~⅔ of VM", not as a precision number |
| c (off-phase artifact + base drive) | 145 (FAST) / 175 (SLOW) mA | the +30 mA SLOW offset matches the campaign's "+5–12 mA at low duty, converging by 0.6": brake-HIGH freewheel routes its base/bias current through the supply/GND (shunt-visible), brake-LOW's loop is internal — consistent hypothesis, not proven |
| K (back-EMF per % travel) | **0.36 ± 0.20 mV/(%/s)** | within-rung I-vs-v regression on the 0.68 flight phases (4 runs: 0.29/0.49/0.45/0.29 — wide spread, honest CI); `K/R = 44 µA/(%/s)` per duty is the regression-direct, R-independent number |
| Position independence of stall I | **±1.42 % max** (0.1–0.6, 5 positions, both modes) | confirms the campaign's ±1.5 % claim → the drive train is electrically homogeneous; ALL position dependence in the friction maps is mechanical |
| Consistency with b | b_electro ≈ **1.8e-4 duty/(%/s)** ≈ ⅓ of the fitted b = 5.5e-4 → mechanical remainder 3.7e-4 > 0 ✓ | electrodynamic braking from K/R and the ~250 mA ON-level; consistent, no contradiction |

Limits, stated: (a) force ∝ duty below ~0.5 is UNVERIFIED — the shunt cannot
see the off-phase motor current that also carries torque; (b) K's ±55 %
spread means torque-per-amp bookkeeping is rough; for the servo this does
not matter — all FF quantities are identified directly in duty units.
(c) **Tooling flag for the campaign:** Ch B levels in the `spikes` preset
read ~3.1× LOWER than the `ladder` preset at the same operating point
(folded ON-levels vs. stall table). The FAST-vs-SLOW spike comparison
(554 vs 190 mV) is same-preset and stands; the absolute mV values are
suspect until the range-dependent scaling in the capture path is checked.

### RC filter — final recommendation (with the decay-mode decision)

The unclipped wiper capture (139, AC-settle transient excluded) identifies
the ADC "spike tail" mechanism: **~0.1–0.3 µs switching bursts (~380 kHz
ringing, singles to ±0.2 V)** on the wiper node; an ADC sample landing
inside a burst (~5 %/tick odds) shifts that tick by up to ~±50 mV ≈ 140
counts — exactly the measured tail. Emulating the firmware's 4×-per-tick
sampling on the measured trace: **rms 11 counts / p-p 69** (matches the live
ADC under PWM), and after a first-order RC 1.5 kΩ(wiper source)×**100 nF**
(fc ≈ 1.06 kHz): **rms 8.8 counts ≈ the unpowered floor (8.5), p-p 32**.
Cost: τ = 150 µs → ~0.15 ms lag at trajectory bandwidth, ~1.4 ms settle to
13-bit after a (non-physical) full-scale step — negligible at 1 kHz control.

**Decision: fit 100 nF at the ADC pin; drive mode stays FAST** (drive: +15 %
speed at 0.70; sensor: FAST is cleaner without the RC and equal with it;
SLOW's cleaner current path is an EMC nicety, not a servo argument).

## 8. Open-loop feedforward recipe (demo input) — with the honest caveat

Sim-checked recipes (`plantsim`, parameters above). Two regimes:

**(a) Kick + cruise (the "obvious" profile) — DO NOT USE OPEN-LOOP.**
Kick 0.80/40 ms breaks away reliably, but cruising at 0.64–0.68 rides on
(u − F_k) ≈ 0.06–0.10 duty, and F_k's run-to-run spread is ±10 %≈0.06:
sim sensitivity shows landing 96 % vs 52 % for ±10 % friction. The Block-4
fine ladders make this final: the duty→velocity map is a CLIFF (§2b — ≤1 %/s
through 0.66, ≥100 %/s at 0.68), so there is no duty value that produces a
moderate open-loop cruise speed at all. Near-breakaway cruise is
open-loop-hopeless on this plant — this is *why* the servo needs the
observer, not a finding against the plan.

**(b) Calibrated impulse moves — GO, now on MEASURED data.**
Fixed-amplitude pulses at |duty| = 0.8, torque 0 after: travel is set by
pulse duration; brake decay stops the slider in ~10–20 ms. The M7 table
(§4c) is the lookup — measured, 3 reps, ±10–15 % repeatability; the 7
unstick blips (120 ms @ 0.78) repeat at σ ≈ 1 % from an identical start.

Demo protocol (existing player API only — pulse = `impulse_ladder` with a
single width, or `dither` amp 0 / bias ±0.8; recording as proof):

1. `goto` (duty 0.8) near the start point; settle 500 ms; read position.
2. Main pulse: width from the measured M7 table for ~80 % of the remaining
   distance (interpolate; fwd and rev columns differ ~1.3×). Keep ≥ 10 %
   clear of the end zones; for rev inside 55–80 % expect early re-stick
   (which *helps* trims).
3. Read landing from the recording (host-side, between runs — still no
   controller in firmware); trim with pulses from the fine end of the table
   (3–10 ms ≈ 0.2–2.6 %), 1–2 rounds.
4. Acceptance per plan: smooth trace, no mid-travel stall (pulses always at
   0.8 — above every observed breakaway), landing ±3 %.

Expected outcome per the measured spreads: main pulse lands within ~±15 % of
the *commanded travel* (i.e. ±3–5 % of full scale for a 30 % move), each trim
round divides the error by ~3–5 → **±1–2 % after 1–2 trims**. GO for the
demo in this iterative-impulse form; NO-GO for one-shot kick+cruise.
