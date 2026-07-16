# Measurement Campaign M0–M8

Instruments: **picoscope** (analog truth), **`slider_lab` firmware** (1 kHz digital
truth, scripted torque, CSV dump), optionally a **0.5 Ω shunt** in one motor lead
(current = torque, the signal the system otherwise doesn't have).

Every experiment states: purpose → setup → procedure → artifact → analysis output.
All lab-rig recordings land in `data/<date>/<experiment>/`, picoscope exports next to
them, one `notes.md` per session. The scope-sync GPIO (see lab-rig spec) makes the two
time bases alignable; record it on a scope channel whenever both instruments run.

General safety: experiments run with the end-zone interlock active (torque hard-cut
outside position [3, 97] unless the experiment is explicitly endstop-flagged) and a
per-experiment duty/duration budget. The motor is a tiny DC can — average |duty| over
any 10 s window stays ≤0.5 except during M5 full-speed steps.

---

## M0 — Rig documentation (15 min, no scope)

**Purpose:** pin down what we're actually measuring; unblock decay-mode planning.
**Procedure:** photograph the rig; identify the H-bridge module (marking on chip),
board variant (S2/S3, PSRAM), wiring incl. grounds; note motor supply source and
whether ADC/pot ground and motor ground share a return path (star point or not).
**Output:** `docs/findings/rig.md` with photos, the H-bridge truth table from its
datasheet, and the correct pin recipe for fast- vs slow-decay drive.

## M1 — Sensor noise floor, motor unpowered (30 min)

**Purpose:** baseline noise budget in counts and mV; is the noise white or structured?
**Setup:** motor leads disconnected or H-bridge disabled. Scope on wiper, AC-coupled,
mV range. Lab rig recording raw ADC at 1 kHz (and one run at max oneshot rate).
**Procedure:** knob parked at 0/25/50/75/100 %, 10 s recording each. Repeat the 50 %
point twice: WiFi connected vs. WiFi off (masterclass datapoint: what does the radio
cost us?). One run with a slow manual full glide for pot-track anomalies (scratchy
spots show as spikes).
**Artifacts:** 6+ CSV runs, scope screenshots/exports at 50 %.
**Analysis:** RMS + p-p noise in counts and % of travel; FFT of the ADC series (peaks
at mains, WiFi bursts, or DC-DC frequencies?); monotonicity check of the glide (S2 ADC
INL is notoriously bad at DB_12 — endpoint self-cal absorbs offset/gain, not curvature;
we need to know how much curvature there is).
**Decision fed:** how much averaging/filtering the servo needs at all; whether ADC
attenuation/divider should change.

## M2 — PWM coupling into the wiper, motor stalled (45 min)

**Purpose:** separate "sensor noise" from "self-inflicted EMI". Suspicion: a large
share of the legacy noise was 19.5 kHz H-bridge coupling.
**Setup:** slider held (mechanically block the knob, or use duty ≤0.35 — below
breakaway it won't move, legacy breakaway ≈0.42). Scope ch A on wiper (AC),
ch B on the PWM pin — trigger on PWM edge so coupling shows phase-locked.
**Procedure:** duty sweep 0 / 0.10 / 0.20 / 0.30, both directions, 10 s ADC recording
each + scope capture. Repeat the 0.20 point in the alternative decay mode (if M0
found a recipe) and, if a shunt is installed, record motor current shape.
**Artifacts:** ADC CSVs + phase-locked scope captures per duty.
**Analysis:** wiper disturbance amplitude vs duty; is it synchronous spikes (capacitive
coupling → RC at the pin helps) or baseline shift (ground/IR drop → wiring/star ground
helps)? Compare decay modes.
**Decision fed:** RC filter (recipe: ~100 nF directly at the ADC pin against the
~1.5 kΩ max wiper source impedance → fc ≈ 1 kHz, also fixes the ADC sampling-cap
error) vs. wiring fix vs. ignore. This decision changes what "measurement noise"
means for every later experiment — that's why M2 is early.

## M3 — Torque vs duty linearity, per decay mode (45 min, wants the shunt)

**Purpose:** with coast (fast) decay a small DC motor at 19.5 kHz runs in discontinuous
conduction at low duty → torque ≈ duty², exactly where fine control happens. Slow decay
linearizes. Quantify, then pick the drive mode.
**Setup:** slider blocked mid-travel (stalled motor: current ∝ torque). Shunt + scope
measuring motor current; without a shunt this experiment degrades to a velocity-based
proxy in M5 (fit quality suffers — hence open question 3).
**Procedure:** duty ladder 0.05…0.6 in 0.05 steps, 2 s each, both directions × both
decay modes. Keep stall periods short (motor heating).
**Analysis:** mean current vs duty per mode; pick the mode with the straightest curve
through the low-duty region.
**Decision fed:** drive mode for `slider_servo`; the torque-constant scaling used by
the observer and all feedforward terms.

## M4 — Breakaway / stiction map (60 min, automated)

**Purpose:** replace the legacy single-point friction constants with a map. ALPS fader
friction is position- and direction-dependent.
**Setup:** lab rig only. Uses the rig's crude repositioning helper between trials.
**Procedure (scripted as one experiment):** grid of 10 positions × 2 directions ×
3 repeats. At each point: settle 500 ms, then ramp duty +0.005 per 20 ms until the
raw position moves >0.3 %, log the breakaway duty, torque off. After breakaway,
hold the last duty for 300 ms and log whether motion *sustains* (kinetic < static:
the Stribeck drop).
**Analysis:** breakaway map d_break(x, dir) with spread; kinetic-friction estimate;
Stribeck-drop magnitude (this number decides how aggressive the impulse endgame must
be — a big drop means position "escapes" after breakaway).
**Decision fed:** feedforward stiction table for the servo; realism of the plant sim.

## M5 — Step responses / dynamics ID (45 min)

**Purpose:** inertia (effective, incl. belt+knob), viscous drag, torque constant —
the observer's model.
**Procedure (scripted):** from standstill at 20 % (resp. 80 % for reverse runs):
duty steps 0.45 / 0.55 / 0.7 / 0.85 / 1.0, recording until the end-zone interlock
cuts torque. 3 repeats each. Also 3 slow triangular duty ramps (0 → 0.6 → 0) for
quasi-static behavior.
**Analysis:** fit m·v̇ = K·d − F_c·sign(v) − b·v over the full recorded trajectories
(scipy least squares, all runs jointly; K partially known from M3). Report parameter
covariance — if b and F_c are degenerate, say so rather than pretending.
**Decision fed:** observer gains; trajectory limits (real max velocity/acceleration
instead of the legacy guesses 50 %/s, 600 %/s²).

## M6 — Backlash width (45 min, shunt makes it direct)

**Purpose:** measure the belt dead band — the quantity the legacy code never had.
**Procedure A (current signature, with shunt):** drive slowly forward, stop, settle.
Command reverse at fixed duty just above breakaway. On the scope: motor current is
low/unloaded while traversing the gap, steps up when the belt engages; pot stays flat
until engagement. Gap traversal time at known duty → gap size. 5 repeats × 3 positions
× both reversal directions.
**Procedure B (behavioral, no shunt):** same commands, measure only the pot dead time
and the position transient at engagement (the thunk, visible as velocity step).
**Analysis:** backlash width + variance; engagement velocity step vs approach duty
(→ the A6 thunk threshold gets its number here).
**Decision fed:** reversal feedforward pulse size; whether unidirectional final
approach is mandatory or merely nice.

## M7 — Impulse calibration ladder (60 min, automated)

**Purpose:** design data for the endgame. From stick, a short pulse moves the slider a
small, hopefully repeatable distance — if displacement vs pulse energy is monotonic
with usable variance, the last-2 % controller becomes a lookup table plus a couple of
corrective shots.
**Procedure (scripted):** at 25/50/75 %: pulses of duty 0.6, durations 1/2/3/5/8/12/20 ms,
10 repeats each, both directions, 300 ms settle + measure between shots. Variant B:
each pulse fired immediately after a small move in the *opposite* direction (cold
reversal) — quantifies how much pulse energy the backlash gap eats.
**Analysis:** displacement vs duration curves with error bars, per position/direction;
same-direction vs cold-reversal comparison.
**Decision fed:** endgame nudge table; expected landing accuracy (A1 feasibility check
happens HERE, in Phase 2 — not after implementation).

## M8 — Dither efficacy AND audibility (30 min)

**Purpose:** dither lowers effective stiction, but this is audio equipment — an
audible buzz disqualifies it regardless of control benefit.
**Procedure:** grid of frequency {50, 100, 200, 300, 400 Hz} × amplitude
{0.05, 0.1, 0.2, 0.3}: repeat the M4 breakaway ramp at 50 % with dither active.
In a quiet room, note audibility per point (ear test is binding; phone spectrogram
as backup). Scope on the wiper: does dither alias into the position signal?
**Analysis:** breakaway reduction vs (f, A) with an audibility mask over it.
**Decision fed:** dither operating point, or dropping dither from the design
(impulse endgame may make it unnecessary anyway).

---

## Session plan

Realistically two bench sessions:

- **Session 1 (scope-heavy, Jan at the bench):** M0, M1, M2, M3 — plus shunt
  soldering if approved. Decisions: RC filter, decay mode.
- **Session 2 (mostly automated, scope for M6):** M4, M5, M6, M7, M8 — the scripted
  experiments run back-to-back; the operator swaps setups and takes notes.

After Session 2: write `docs/plant-model.md`, then Phase 3 starts with zero hardware
dependency.
