# Probe & Bench Connections — PicoScope 3203D via pico-bench

Scope facts (from `/Users/jan/dev/repos/pico-bench`): 3203D, **2 channels** (A/B),
EXT trigger input, AWG output, 8-bit ADC (analog offset available — use it),
64 MS memory, ps3000a driver under Rosetta (`arch -x86_64 .venv-x86/bin/python`).
Close the PicoScope 7 app before scripting; USB-3 port preferred.

## One-time rig prep (Jan, soldering iron, ~30 min)

Add permanent test points (pin headers or wire loops) so sessions don't start with
clip archaeology:

| TP | Net | Notes |
|---|---|---|
| TP-GND | ground star point | at the pot's GND terminal; **all** scope ground clips land here, nowhere else |
| TP-WIPER | pot wiper = GPIO9 | the feedback signal |
| TP-3V3 | 3.3 V rail | noise reference for M1 |
| TP-PWM-A | GPIO11 / L9110 IA | forward drive side |
| TP-PWM-B | GPIO10 / L9110 IB | needed for decay-mode work |
| TP-SYNC | GPIO14 (Jans Verdrahtung) | → EXT trigger, standing connection |
| TP-VM | 5 V motor supply | occasional sanity checks |
| TP-ISEN | L9110-Modul GND-Rückleitung via 0.2 Ω | **the shunt — see below** |

**Shunt as built (Jan, 2026-07-16 — replaces the AISEN proposal, which assumed a
DRV8833; the real bridge is an L9110 without sense pins):** **0.2 Ω in the GND
return of the L9110 module** (GND — Shunt — GND-Brücke). V(TP-ISEN) =
0.2 · I_bridge, **single-ended vs TP-GND** — no differential measurement, no
channel wasted. It carries the **full** bridge current including switching
spikes: live spikes >500 mV were observed, so use **±2 V/±5 V ranges for
spike/transient work** and **±200..500 mV for averaged staircase levels** — the
old ±50 mV recipe clips hard. There is **no ITRIP-style hardware current chop**
on the L9110; the firmware interlocks are the only current protection.

## Standing connections (Jans Probe-Dressing, 2026-07-16)

**TP-SYNC (GPIO14) → EXT trigger. Always.** Every capture triggers off the
experiment program (rising = program start, segment boundaries blip LOW for 1 ms).
This is what makes scope captures and firmware CSV sample-exact alignable without
eating a channel.

Probes as dressed at the bench: **probe 1 ↔ wiper or shunt** (re-clip per
experiment), **probe 2 ↔ PWM pins only**. This covers every essential capture in
the table below; the two deviations from the original plan are minor: M1's 3.3 V
rail comparison needs a one-off re-clip of probe 2, and M6 drops the simultaneous
scope-wiper (the firmware records the pot at 1 kHz anyway — engagement detection
lives in the current signature on probe 1).

## Per-experiment channel map

| Exp | Ch A | Ch B | Trigger | Sampling |
|---|---|---|---|---|
| M1 noise floor | TP-WIPER, AC, ±50 mV | TP-3V3, AC, ±50 mV | free-run + EXT marks | 1 MS/s |
| M1b ADC-INL (optional, see below) | AWG signal at ADC pin, DC, ±5 V | — | EXT | 1 MS/s |
| M2 PWM coupling | TP-WIPER, AC, ±50 mV | TP-PWM-A, DC, ±5 V | **Ch B rising** (phase-locked to PWM) | 10 MS/s |
| M3 torque linearity | TP-ISEN, ±0.5 V (Mittel) / ±2 V (Spikes) | TP-PWM-A, DC, ±5 V | Ch B rising | 10 MS/s |
| M4 breakaway map | firmware CSV is primary | optional TP-WIPER DC ±5 V | EXT | 1 MS/s |
| M5 step responses | firmware CSV primary; TP-ISEN adds current transients | TP-WIPER, DC, ±5 V | EXT | 1 MS/s |
| M6 backlash | TP-ISEN, ±0.5 V (Mittel) / ±2 V (Spikes) | TP-WIPER, DC, ±5 V | EXT | 1 MS/s |
| M7 impulse ladder | firmware CSV primary | optional TP-ISEN | EXT | 1 MS/s |
| M8 dither | TP-WIPER, AC, ±50 mV | TP-ISEN | EXT | 1 MS/s |

Block-mode memory budget: 64 MS ÷ 2 ch → ~6.4 s at 10 MS/s (plenty for PWM-edge
work), ~32 s at 1 MS/s (covers every scripted program).

M6 is where the shunt earns its keep: during backlash traversal the motor runs
unloaded (current low, pot flat), at belt engagement the current steps up — the
gap becomes directly visible as a time × current signature instead of being
inferred from pot dead time.

## M1b — AWG bonus experiment (recommended, ~15 min in session 1)

The 3203D AWG separates **ESP32-S2 ADC nonlinearity** from **pot track
nonlinearity** — two error sources the self-calibrating endpoints can't tell apart:
disconnect the wiper wire from GPIO9, feed the AWG directly into the ADC pin
(0→2.75 V slow ramp, 30 s), record with the lab firmware, monitor the same node on
Ch A. Firmware CSV vs known ramp = the S2's INL + noise, isolated. Legacy suspicion:
S2 ADC at DB_12 attenuation is visibly nonlinear — this measures how much.

## Grounding rules — OTA-only when the scope is on (decided 2026-07-16)

The Mac shares earth with the scope; USB-powering the rig from the Mac would close
a ground loop straight through the ground clips — into exactly the mV-level
measurements the campaign depends on (M1 noise floor, M2 coupling, M3/M6 ISEN).
The rig is wireless-first by design (HA service start, HTTP CSV dump), so:

- **Simple rule: scope clipped ⇒ USB unplugged.** The rig floats on a **USB power
  bank** (preferred: battery = no switching hash, no earth path; a Class-II
  double-insulated wall charger is the fallback). All control and data over WiFi.
  The scope's clip at TP-GND is then the single ground reference by construction.
- USB-to-Mac is fine for flashing and bring-up — scope disconnected.
- **Power-bank caveats:** (1) pick one that doesn't auto-off at light load —
  S2 idle can drop under many banks' keep-alive threshold; if runs die mid-session,
  that's the first suspect. (2) **VM sags as the bank drains, and torque constant
  K ∝ VM** — note VM (multimeter at TP-VM, or a scope spot-check) at the start of
  every run block in `notes.md`; the M3/M5 fits need it.
- One star point (TP-GND). All scope ground clips go there and only there;
  clip ground first, then probes. Never clip a ground lead to a non-GND net
  (clips are earth-bonded).
- Same pattern pico-bench already validated on the ESP32-C6 (OTA current
  measurement, USB off).

## Data landing convention

Scope captures saved as `.npz` (pico-bench archive format) next to the lab CSVs:
`data/<date>/<seq>_<experiment>.csv` + `<seq>_<experiment>_scope.npz` + session
`notes.md`. Recipe/limit YAMLs specific to this project live here (pico-bench
README: consuming repo owns recipes), under `analysis/scope-recipes/` when we get
to them.
