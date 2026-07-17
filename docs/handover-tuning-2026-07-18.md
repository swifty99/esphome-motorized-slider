# Handover — slider_servo Deep-Dive Tuning-Loop

Auftrag von Jan (2026-07-17 nachts, wörtlich): „do handover for a deep dive
tuning loop. this is a perfect start, we shall try different strategies maybe
but this is the baseline." **Heißt: der Servo lebt und funktioniert (Baseline
steht). Diese Loop probiert systematisch Tuning-Strategien und misst jede gegen
die Baseline.** Kein Bring-up mehr — reines, messgetriebenes Verbessern.

Bring-up-Story + alle Zahlen: `docs/findings/hil-2026-07-17.md`.
Servo-Architektur/Verkabelung: [[servo-alive-on-rig]] (Memory) bzw. der Commit
`6fbb134`. Tools: `analysis/servo-hil/` (+ dessen README).

## Baseline — der Maßstab (nicht verschlechtern)

Firmware = Commit **e28f965** (`firmware/slider_lab_rig.yaml`). Servo-Werte:
`impulse_aim 0.80`, `impulse_max 12`, `impulse_settle_ms 150`,
`crawl_impulse_duty 0.72`, `recorder 12000`; Rest = Spec-Default.

**A1-Scorecard (seed 1, 40 Moves) — die Zahlen zum Schlagen:**

```
all  gave_up 0   overshoot 5%   max|err| 0.43%   out-of-tol 12%   max pulses 3   settle med 0.7s
fwd  overshoot 0%
rev  overshoot 11%
```

Dazu: Boot drehmomentfrei, A9 top (p99 ~525 µs, ~1 missed/250 k, 0 ADC-Fehler),
beide Richtungen immer `reached`. **Die 12 % out-of-tol sind der p̂-vs-z-Offset
(~0.3 %), NICHT Endgame-Bounce** — mit Endgame-Knöpfen nicht wegzukriegen (Achse
B unten).

## Die Loop (bewährter Zyklus)

1. **Tune** — ein YAML-Knopf (unten) ODER eine Strategie-Änderung.
2. **Flash** — `cd firmware && esphome compile slider_lab_rig.yaml && esphome upload slider_lab_rig.yaml --device 192.168.178.115` (Jans OK nötig; keine Allow-Rules → prompt). Boot-Verify: `servo-hil/servo_status.py`.
3. **Measure** — `cd analysis && uv run python servo-hil/a1_sweep.py 40 1` (gleicher Seed = identische 40 Moves wie die Baseline).
4. **Compare** — gegen die Scorecard oben. Für CRAWL zusätzlich `crawl_analyze.py` auf einem `/servo/dump.csv`.
5. **Commit** wenn die Ziel-Achse strikt besser ist OHNE die anderen zu
   regredieren; sonst verwerfen, nächste Strategie. Klein/oft/deutsch, kein Push.

Regel: **immer gegen Seed 1 vergleichen** (A/B auf denselben Moves), sonst
vergleicht man Rauschen. Für Streuung ggf. 2–3 Seeds mitlaufen lassen.

## Verfügbare Knöpfe (YAML, KEIN Core-Recompile)

Alle in `slider_servo:`; Default = Spec, aktueller Rig-Wert in Klammern wenn
gesetzt. Core bleibt unangetastet, SIL bleibt 30/30.

| Knopf | Default | Wirkt auf |
|---|---|---|
| `impulse_aim` | 0.90 (**0.80**) | Endgame-Undershoot; kleiner = weniger Overshoot, mehr Pulse |
| `impulse_max` | 8 (**12**) | Endgame-Pulsbudget vor `gave_up` |
| `impulse_settle_ms` | 120 (**150**) | Settle nach langem Puls (>12 ms) vor Re-Decide |
| `impulse_settle_short_ms` | 70 | dito für kurze Pulse (≤12 ms) |
| `crawl_impulse_duty` | 0.80 (**0.72**) | CRAWL-Kick-Härte (Akustik); Endgame bleibt 0.80 |
| `crawl_max_width_ms` | 20 | max CRAWL-Puls-Breite → Ruck pro Klack |
| `tol` | 0.30 | Landetoleranz (Latch auf p̂) |
| `v_cruise` | 180 | Marschgeschwindigkeit MOVE |
| `v_min_cmd` | 120 | Cliff-Floor (nie kriechen im MOVE) |
| `u_max` | 0.90 | L1-Clamp / Grab-Diskriminierung |
| `thermal_avg_max` | 0.50 | L1-Thermal-Trip |
| `raw_at_0` / `raw_at_100` | 8000 / 100 | Endpoint-Kalibrierung Counts→% (Achse B!) |

## Tuning-Achsen & Strategien (Vorschlag, grob nach Wert)

**A — Endgame-Präzision** (1 Runde durch, weiter möglich). Rev-Overshoot 11 %
ist ok, aber Backlash-Reversal ist die Wurzel. Levern: `impulse_aim` weiter
runter (mehr Pulse, weniger Overshoot — Kurve suchen), `impulse_settle_ms`.
Noch NICHT exponiert, aber leicht nachrüstbar: `reapproach`, `endgame_zone`,
`approach_margin`. Sauberste Strategie: **direction-aware Endgame** (Rev bekommt
eigenes aim/settle) oder **backlash-aware First-Pulse** nach Richtungswechsel —
Core-Change (Muster unten).

**B — Observer/Sensor-Offset ~0.3 %** (= das out-of-tol-12-%-Floor, die größte
verbleibende *Genauigkeits*-Grenze). p̂ liest am Settle konsistent ~0.3 % neben
dem Roh-z. Kandidaten: echte Endpoints auf dem Rig vermessen und
`raw_at_0/raw_at_100` setzen (bisher nominal 8000/100); oder Observer-Bias. Am
saubersten löst das **Auto-Ident** (Achse F). Endgame-Knöpfe helfen hier NICHT.

**C — CRAWL-Akustik** (Jans-Ohr-Item, Kandidat 0.72 geflasht). Sweep
`crawl_impulse_duty` 0.72↔0.80 (Trade-off: leiser vs Rate — 0.72 lieferte ~6.6
statt 10 %/s), `crawl_max_width_ms`. Wenn der Puls-Zug grundsätzlich zu klackig:
**kontinuierlicher/Dither-Antrieb knapp unter Breakaway** (stille CRAWL-Variante
§6 f) — Core-Change. Metrik: `crawl_analyze.py` (Klacks/s, Ruck/Klack, Duty) +
Jans Ohr für „leiser".

**D — Move-Feel / Speed**: `v_cruise`, `v_min_cmd`, plus `a_traj` (nicht
exponiert) — schnellere/weichere Moves vs Sauberkeit. `peak|v|` und settle-Zeit
im Sweep beobachten.

**E — MOVE-Gains**: `kp`, `kd` (nicht exponiert) — MOVE-Tracking/Overshoot vor
dem Endgame. Nur anfassen wenn A/D die MOVE-Phase als Quelle zeigen.

**F — Auto-Ident On-Device** (Block 6, `docs/auto-ident-spec.md`). Der
Wurzel-Fix für B (Endpoints) UND den Rev-Tabellen-Teil von A. Schreibt UnitCalib
nach NVS über `save_calib()` (Blob 'SVRC' v1, Pfad steht im Wrapper). Ersetzt
Handtuning durch per-Unit-Messung — irgendwann die eigentliche „Strategie".

## Muster für Core-Änderungen (SIL heilig halten)

Jede größere Strategie (direction-aware Endgame, backlash-First-Pulse,
Dither-CRAWL) als **SIL-neutralen Core-Eingriff**: neuer `ServoConfig`-Param mit
Default = bisheriges Verhalten (Spec), sodass `uv run pytest tests/test_sil.py
tests/test_controller.py` grün bleibt (30/30); Rig-Verhalten NUR per YAML-Override.
So wurde `crawl_impulse_duty` gemacht (Commit `e28f965`) — Vorlage. Vergleich
gegen SIL: `sil/build/sim_runner_prod <scenario> --out x.csv` (Szenario-Format
im Kopf von `sil/sim_runner.cpp`).

## Bench-Fakten & Leitplanken

- Rig **slider-lab @ 192.168.178.115**, ESP32-S2 (kein FPU — Tick-Budget
  respektieren, p99 aktuell 525 µs). **.187 = fremdes Gerät, Finger weg.**
- Firmware = `slider_lab_rig.yaml` (slider_lab `enabled: false` = dormant, Servo
  = alleiniger Bridge-Owner; geteilte Pins via `allow_other_uses`). safe_mode +
  OTA = jederzeit reflashbar; Jan kann physisch unbricken.
- **Flash NUR mit Jans OK** (keine Allow-Rules → jeder esphome-Aufruf promptet;
  KEINE eigenen Allow-Rules schreiben, siehe [[classifier-blocks-self-permissions]]).
- **30 s Pausen zwischen Aktuations-Chunks** (L9110 ~800 mA). Der A1-Sweep ist
  durch seine Move-Gaps + Analyse selbst-gedrosselt; Thermal-EMA blieb <0.03.
  Thermal beobachten, kein absichtliches Grinding. Anschläge erlaubt.
- **SIL muss grün bleiben**: `cd analysis && uv run pytest tests/test_sil.py tests/test_controller.py` → 30/30. Bei Core-Änderung immer prüfen.
- **Der eine Reboot** (einmalig, unbestätigt, mit einem 40-s-Slow-Dump
  korreliert): jetzt diagnostizierbar über den „Servo Reset Reason"-Sensor
  (aktuell „sw/ota") + Dump-Yield. Wenn er wiederkehrt und `task_wdt` zeigt →
  Dump-Pfad nochmal härten. Stress-Test (4 Dumps am Stück) war sauber.
- Abends: Findings-Doc updaten, Rig idle, Torque 0.

## Für Jan

- **Ohr-Item offen**: CRAWL @ 0.72 leise genug? Zu langsam (Richtung 0.80)? Oder
  gleich die Dither/kontinuierliche stille Variante bauen?
- **Priorisierung**: welche Achse zuerst — A (Endgame perfektionieren), B/F
  (Genauigkeit via Kalibrierung/Auto-Ident), C (Noise), D (Speed-Feel)?
- Noch offene Hand-Items (nicht in der Loop): A6-Thunk, A8-Grab mit echter Hand,
  Endzonen-Dead-Band (`span_lo/hi`).
