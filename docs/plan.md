# Master Plan

Status tracker at the bottom. Each phase has an **exit gate** — measurable, not vibes.
Nothing proceeds to implementation of the servo controller before the plant is
identified and the controller spec is approved.

## Goal

A motorized-fader servo that is:

1. **Smooth** — trajectory moves without stick-slip judder, audible thunks, or creep.
2. **Precise** — lands on target reliably (see metrics), no limit cycle at rest.
3. **Silent at rest** — PWM hard-off when idle/settled (audio equipment).
4. **Touchable** — manual grab detected fast, motor releases immediately,
   force-feedback detents (Rastpunkte) feel like detents, not like a fight.
5. **Self-identifying** — „the ladder has one more step: get the parameters for
   model and control **on device**. each slider is different. (i have more and
   different)" (Jan, 2026-07-16). Every unit identifies its own model/controller
   parameters itself — no scope, no host, bounded time — and stores them per
   exemplar (NVS). Scope-bound measurements (spikes, coupling, INL) are
   design-time knowledge; production auto-ident must not need them. Deliverable:
   `docs/auto-ident-spec.md` — per parameter: bench method vs. on-device method
   vs. storage.

## Non-goals (v1)

- Multi-instance support (design for it — no function-static state — but don't test it).
- Arduino framework support (ESP-IDF only).
- Backwards compatibility with the old YAML schema.
- Accelerometer in the control loop (escalation path only, Phase 7).

## Why the old controller wasn't smooth (short form; full autopsy in legacy-analysis.md)

- **Friction feedforward as a relay:** ±0.42 full-scale stiction kick keyed to the
  *sign of the commanded torque* → chatter/limit cycle near target, hidden by a ±2 %
  tolerance stop.
- **Velocity estimate blind at low speed:** finite difference + PT1 (τ=150 ms) +
  hard zero below 9 %/s. No damping possible exactly where stick-slip lives.
- **Backlash unmodeled:** belt dead band never measured, never compensated;
  reversals thunk.
- The plant was never identified; all constants were hand-tuned under pressure.

## Strategy (the "magic", in dependency order)

1. **Identify the plant first.** Stribeck friction map (position- and
   direction-dependent), inertia + viscous drag, backlash width, actuator
   torque-vs-duty linearity per decay mode, sensor noise budget incl. PWM coupling.
   Instruments: picoscope + a dedicated experiment firmware (`slider_lab`).
2. **Model-based observer.** 2-state Kalman/Luenberger (position, velocity) driven by
   commanded torque through the identified model → low-lag, low-noise velocity down
   to zero speed. This unlocks damping, Kd, and detent stability.
3. **Feedforward-dominant cascade.** Torque = stiction·sign(v_des) + Kv·v_des +
   Ka·a_des + feedback(residual). Friction FF keyed to *desired* velocity sign
   (clean, constant per segment) — never to noisy torque sign. Trajectories never
   command creep below the stick-slip threshold.
4. **Endgame tricks instead of gain:** calibrated impulse nudging inside the last
   ~2 %, and unidirectional final approach (always land from the same direction)
   to cancel backlash. Optional dither, only if inaudible (M8 decides).
5. **Actuator linearization:** slow-decay PWM drive if the H-bridge supports it
   (M3 decides).

## Phases

### Phase 0 — Scaffold ✅ (this session)
Repo, plan, legacy autopsy, lab-rig spec, measurement protocols.
**Gate:** Jan has read the plan; open questions below answered.

### Phase 1 — Lab-rig firmware (`slider_lab`)
The identification instrument, NOT the controller. Spec: `docs/lab-rig-spec.md`.
Open-loop experiment player + 1 kHz recorder + HTTP CSV dump + scope-sync GPIO +
safety interlocks. Flash needs Jan's OK (house rule).
**Gate:** a torque-step run recorded on the rig, loaded in Python, and the same event
visible time-aligned on a picoscope capture.

### Phase 2 — Measurement campaign (M0–M8)
Protocols: `docs/protocols/measurement-campaign.md`. Produces `docs/plant-model.md`
with fitted parameters + confidence, and the three hardware decisions:
RC filter yes/no, decay mode, dither yes/no.
**Gate:** plant simulator (Phase 3) reproduces recorded step/reversal runs within
stated tolerance — parameters are real, not decorative.

### Phase 3 — Offline design (Python, `analysis/`)
- Plant simulator: Stribeck + backlash + inertia + quantized/noisy ADC model,
  validated against recorded runs (overlay plots committed to `docs/findings/`).
- Observer prototype tuned on *recorded* data; velocity validated against offline
  zero-phase differentiation as ground truth.
- Auto-ident spec draft (`docs/auto-ident-spec.md`): for every fitted parameter,
  prefer identification procedures a device can run on itself (breakaway ramp,
  impulse ladder, step fits from the on-board recorder) over scope-bound ones.
- Controller prototype tuned in sim: cascade FF+PID, impulse endgame,
  backlash-aware approach, override handling.
**Gate:** sim meets the acceptance metrics below; observer velocity RMS error and lag
documented.

### Phase 4 — Controller spec + approval gate
`docs/controller-spec.md`: layer architecture, complete state machine as a
**trigger graph** (every state/transition → who fires it), invariants phrased as
falsifiable tripwires, and an explicit **forbidden list** (what the controller must
NOT do — drift lives in the gaps specs leave open).
**Gate:** Jan approves the trigger graph BEFORE any implementation. (Anti-anchoring
template: spec is the only source; old code only for names/pins/timing.)

### Phase 5 — Clean-room implementation (`slider_servo`)
New ESPHome external component, ESP-IDF. Hard architectural requirement:
**controller core is pure C++ with zero IDF includes** — HAL (ADC, PWM, clock)
behind a thin interface. This makes the core natively compilable on macOS.
SIL harness: a `sim_runner` CLI links core + plant simulator, plays scenario files,
emits CSV; pytest asserts the tripwires. Every invariant from Phase 4 gets a test,
each proven non-vacuous with a negative control (test must fail against a seeded bug).
`slider_lab`'s experiment player stays available behind a config flag for regression
measurements on hardware.
**Gate:** all tripwires green in SIL, incl. negative controls documented.

### Phase 6 — HIL verification & tuning
Flash to rig only after Jan's OK. Run the acceptance experiments (same recorder as
Phase 1 — the lab infrastructure measures the servo). Before/after comparison plots
vs. the legacy recordings from Phase 2.
**Gate:** acceptance metrics met on hardware, documented with plots in
`docs/findings/`.

### Phase 7 — Optional escalations (only if Phase 6 misses targets)
- Knob accelerometer (ICM-42688 class) — first as instrumentation, fused into the
  observer only if needed.
- Motor back-EMF sensing during PWM off-phase → motor-side velocity; combined with
  the slider-side pot this makes the backlash state observable.

### Phase 8 — Migration
Port `kueche_mief` (the deployed user of the old component) to `slider_servo`.
HA entity names stay identical (EsphomeZoo house rule). Old component retired.

## Acceptance metrics (v0 — refine with real numbers after Phase 2)

| # | Metric | Target |
|---|---|---|
| A1 | Landing accuracy, 40 randomized moves | ≤0.5 % median, ≤1.0 % p95 |
| A2 | Rest silence after settle | both PWM outputs identically 0 for ≥2 s, no re-trigger |
| A3 | No limit cycle | position p-p ≤0.3 % after settle |
| A4 | Cruise smoothness (50 %/s, mid-travel) | velocity ripple σ/mean ≤0.15 |
| A5 | 50 %-travel move duration | ≤1.2 s incl. settle |
| A6 | Reversal | no audible thunk; quantified threshold set after M6 |
| A7 | Detent pull-in from 3 % offset | settled ≤400 ms, ≤1 overshoot |
| A8 | Manual grab during move | detected ≤100 ms, torque released ≤2 ticks after |
| A9 | Loop health | ISR p99 <500 µs, zero missed 1 kHz ticks in 60 s |

## Standing decisions (defaults — say the word to change)

- Host: ESPHome external component on ESP-IDF (fits the zoo + HA tooling).
- Loop rate 1 kHz.
- No function-static state anywhere in the new code (instance members only).
- Data dump via on-device HTTP CSV endpoint (curl-able, script-friendly).
- Python side: uv-managed project in `analysis/`.
- Repo local-only for now; Forgejo remote when Jan wants one.

## Answered (2026-07-16)

1. H-bridge: ~~DRV8833~~ → **L9110** (Jans Korrektur an der Bench, 2026-07-16
   abends). Kein Coast-Zustand, kein Strom-Limit/ITRIP. Firmware-„FAST" =
   brake-low (= slow decay!), „SLOW" = brake-high — Rezepte korrigiert in
   lab-rig-spec R3.
2. Board: **ESP32-S2 Mini** (2 MB PSRAM) → recorder buffer in PSRAM.
3. Scope: **PicoScope 3203D** (2 ch + EXT + AWG) with the automation toolkit in
   `/Users/jan/dev/repos/pico-bench` (Rosetta x86 venv, block capture proven).
   Connection plan: `docs/protocols/probe-connections.md`. Shunt as built:
   **0.2 Ω in der GND-Rückleitung des L9110-Moduls** (führt den gesamten
   Brückenstrom, single-ended; der L9110 hat keine Sense-Pins).

## Open questions for Jan

1. ~~Shunt-Position~~ → final (2026-07-16, nach L9110-Korrektur): **0.2 Ω in der
   GND-Rückleitung des Moduls** (GND—Shunt—GND-Brücke), single-ended vs TP-GND.
   Kein ITRIP-Schutz (existiert am L9110 nicht) — Interlocks der Firmware sind
   der einzige Stromschutz. M3/M6 messen den Gesamt-Brückenstrom.
2. Is **force feedback / Rastpunkte** a v1 requirement, or is smooth positioning the
   priority and FF comes as v1.1? (Affects Phase 4 spec scope, not Phases 1–3.)

## Status tracker

| Phase | Status | Notes |
|---|---|---|
| 0 Scaffold | ✅ done | commit 2612d29 |
| 1 Lab rig | ✅ bis auf Scope-Alignment | v3 geflasht+verifiziert (9c1928c): 52k Intervalle 0 strukturelle Gaps, max Jitter 115µs, Dump 3.5s abort-fest; tickhealth PASS (6d13e31); Alignment-Gate = erster Block der Nacht-Session 2026-07-16→17 |
| 2 Measurements | 🔶 läuft | autonome Nacht-Kampagne 2026-07-16→17 (Handover: docs/handover-night-2026-07-16.md); Nacht-Zuschnitt: A=Wiper/B=Shunt fest, M4/M7 nach goto-Fix, M8 nur objektiver Teil |
| 3 Offline design | 🔶 startet mit M3-Daten | modeler offline; Deliverables plant-model.md + auto-ident-spec.md (Draft) |
| 4 Controller spec | ⬜ | approval gate |
| 5 Implementation + SIL | ⬜ | |
| 6 HIL verification | ⬜ | flash gate |
| 7 Escalations | ⬜ | only if needed |
| 8 Migration kueche_mief | ⬜ | entity names frozen |
