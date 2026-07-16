# motorslider — clean-room servo controller for a motorized audio fader

Clean-room redesign of the motorized-slider controller (ALPS RS60N11M9 class fader,
belt-driven DC motor, pot feedback, ESP32-S2/S3). The old ESPHome component in
`EsphomeZoo/components/motorized_slider` is **reference material only** — nothing has
to stay. Goal: genuinely smooth, silent, precise motion despite dominant stiction,
noisy feedback, and belt backlash.

This is run as a measurement-driven project ("masterclass" style): every control
decision is backed by an identified plant parameter or a recorded experiment, and
every invariant becomes a falsifiable tripwire test before hardware sees the code.

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
