# Motorized Slider for ESPHome

Position control for a motorized audio fader (ALPS RS60N11M9 class: belt-driven DC
motor, pot feedback, strong stiction) on ESP32-S2/S3 with ESPHome.

## Two versions

| Version | Where | Status |
|---|---|---|
| v1 (2025, hand-written) | `components/motorized_slider`, tag `v1-legacy`, [README-v1](README-v1.md) | Works, but chatters at the target. Why: [docs/legacy-analysis.md](docs/legacy-analysis.md) |
| v2 (2026, clean-room) | `firmware/components/slider_servo` | Measurement-driven rewrite. SIL gate green, first HIL stage measured on the rig, not yet deployed in the field |

Use v2 in your ESPHome config (full example: [firmware/slider_lab_rig.yaml](firmware/slider_lab_rig.yaml)):

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/swifty99/esphome-motorized-slider
      ref: main
      path: firmware/components
    components: [slider_servo]
```

## How v2 was built

Every control decision is backed by an identified plant parameter or a recorded
experiment (PicoScope + lab firmware `slider_lab`), and every invariant became a
falsifiable tripwire test in software-in-the-loop before hardware saw the code.
v1 was used for hardware facts and constants only, never for control flow.

## Documents

| Doc | What |
|---|---|
| [docs/plan.md](docs/plan.md) | Master plan: phases, exit gates, acceptance metrics, status tracker |
| [docs/legacy-analysis.md](docs/legacy-analysis.md) | Autopsy of the old component: what it did, why it wasn't smooth, known bugs |
| [docs/lab-rig-spec.md](docs/lab-rig-spec.md) | Spec of the Phase-1 experiment firmware (`slider_lab`) — the identification instrument |
| [docs/protocols/measurement-campaign.md](docs/protocols/measurement-campaign.md) | Picoscope + lab-rig measurement protocols M0–M8 |

## Layout (grows over time)

```
docs/            plans, specs, protocols, findings
firmware/        slider_lab (Phase 1), later slider_servo (Phase 5)
analysis/        Python: loaders, identification fits, plant simulator, controller sim
data/            recorded runs (lab CSV + picoscope exports), committed selectively
```

## House rules

- Clean room: the controller spec is the only source for the new implementation.
  Old code may be opened for names, pinning, and timing constants — never for
  control-flow structure.
- Nothing gets flashed to a device without Jan's explicit OK.
- The deployed device (`kueche_mief`) keeps its HA entity names at migration time.
