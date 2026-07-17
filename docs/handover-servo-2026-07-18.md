# Handover — slider_servo ALIVE (orchestrierte Opus-Session)

Auftrag von Jan (2026-07-17 abends, wörtlich): „the slider servo in pure C++
was not my command. the target is ESPHome. get the convenience if you like."
**Heißt: das Produkt ist die ESPHome-Komponente auf dem Rig.** Der pure-C++-
Core aus Phase 5 bleibt als Innenleben und SIL-Testbett (die „convenience") —
er wird GEWRAPPT, nicht neu geschrieben. Diese Session macht den Servo
real: Wrapper, Compile, Flash (Gate: Jans OK), HIL-Bring-up.

## Stand bei Übergabe (2026-07-17 spätabends)

- **Phase 4 freigegeben** (8 Gate-Antworten eingearbeitet, inkl. CRAWL-Modus
  §2 Zeilen 26–29, Endzonen-Politik (a), A8 ≤0.5 s, FF→v1.1).
- **Phase-5-SIL-Gate GRÜN**: alle 17 Tripwires + Negative Controls
  (`docs/findings/sil-phase5-2026-07-17.md` — dort auch die 3 gefundenen
  latenten Fehler inkl. Observer-Re-Stick-Fix). 30/30 Tests:
  `cd analysis && uv run pytest tests/test_sil.py tests/test_controller.py`.
- **Code-Karte**: Core `firmware/components/slider_servo/core/*`
  (servo.h = Tick-Pipeline/Fassade, controller.* = §2-Graph, observer.h =
  EKF, safety.h = L1, servo_params.h = §5 + Nominal-Kalibrierung,
  sil_bugs.h = Seeded Bugs, ohne `SLIDER_SIL_BUGS` wegkompiliert).
  SIL: `sil/` (CMake nativ, sim_runner). Es gibt noch KEINEN Wrapper —
  `slider_servo` hat kein `__init__.py`, kein ESPHome-`.h/.cpp`.
- **Spec ist Gesetz**: `docs/controller-spec.md` (freigegeben). Verbote §4
  (kein Function-static, kein Heap/Blocking/Logging im Tick, keine Magic
  Numbers, goto bleibt im Lab-FW). Params §5. HIL-Items §6 a–f.

## Bench-Fakten (erspart Suchen; bei Zweifel Repo-Doku schlägt Memory)

- Rig **slider-lab @ 192.168.178.115 (fest)**, ESP32-S2 Mini (2 MB PSRAM,
  KEIN FPU — float ist Software, Tick-Budget respektieren; slider_lab
  schafft p99 418 µs @ 1 kHz). Auf **.187 wohnt ein FREMDES Gerät — Finger
  weg.** Steuerung: `cd analysis && uv run labctl status|preflight|run|flash|cycle`.
- Verkabelung: Pot-Wiper → **GPIO9 (ADC1 ch8)**, 13-bit, ATTEN_DB_12,
  4 Samples/Tick. Brücke ist ein **L9110** (YAML-Kommentar „DRV8833" ist
  stale): **AIN1=GPIO11 (fwd), AIN2=GPIO10, LEDC 19531 Hz**. Achtung
  Decay-Semantik: Firmware-„FAST" = brake-low = physikalisch slow decay —
  Rezept aus `slider_lab` 1:1 übernehmen, Entscheidung war decay FAST.
  Sync GPIO14. Raw-Skala invertiert: ~8000 Counts @ 0 %, ~100 @ 100 %.
- Arbeitswerte: Breakaway-Cliff ~0.66–0.70 (±0.04 Spread), goto_duty 0.8.
  Endzonen: nur passiver Marker, kein Cut (Politik (a), Spec §6 Antwort 8).
- Wiedervorlage: **ein ADC-Vorfall** (raw flach 0, nur OTA-Reboot half);
  slider_lab zählt adc_oneshot_read-Fehler als Sensor — der Servo-Wrapper
  muss den Fehlerzähler ebenfalls führen (und der Tick darf auf Fehlercodes
  nicht mit Arbeit reagieren, nur zählen; Observer-Fault-Pfad Row 25
  existiert im Core).
- Scope (nur falls HIL-Traces nötig): PicoScope 3203D, Single Owner,
  Trigger NUR Ch B (EXT-Pfad tot), Toolkit in `~/dev/repos/pico-bench`.

## Orchestrierung (bewährtes Muster der bisherigen Sessions)

Opus-Orchestrator hält Commits (atomar, deutsch, Befund im Text, KEIN Push)
und Phasen-Gates; Agenten tauschen NUR über das Repo. Vorschlag:
**coder** (Wrapper, Blöcke 1–2), **hil-runner** (Blöcke 4–5,
hardware-seriell — nie zwei Agenten gleichzeitig an der Bench).
API-529: 10 min Backoff, Agent per SendMessage fortsetzen.

## Blöcke

1. **ESPHome-Wrapper `slider_servo`** (coder; Vorbild: `slider_lab`-Layout
   mit `__init__.py`-Codegen + `.h/.cpp` unter `USE_ESP_IDF`):
   - HAL-Bindung an den Core (`core/servo.h` Fassade): adc_oneshot-Sampling
     (4×/Tick, Median/Mittel wie Lab), Counts→% über Endpoint-Kalibrierung
     (Defaults 8000/100, NVS-überschreibbar), LEDC-Drive mit dem
     L9110-Rezept aus slider_lab, gptimer-1-kHz-Tick (Struktur aus
     slider_lab übernehmen — bewährt).
   - Config-Schema: Pins wie oben, Optionen für §5-„HA?"-Params (tol,
     v_cruise, impulse settle). **Beide Komponenten in einer Firmware**:
     slider_lab bleibt hinter Config-Flag verfügbar (plan.md Phase 5),
     Motor-Ownership exklusiv (Servo default; Lab-Player nur wenn Servo
     disabled — ein einfacher Arbiter, kein doppeltes PWM-Setup).
   - HA-API: number „Target" (0–100 %), optional number „Speed" (<120 → 
     CRAWL), buttons release/abort, Sensoren: state, settle_flag,
     release_reason, pulses/kicks/overshoots, thermal_avg, ADC-Fehler,
     **tick p99 µs + missed ticks (A9!)**.
   - Recorder: Core-Ring (PSRAM, Alloc im Setup) + HTTP-CSV-Dump nach
     slider_lab-Muster — Phase 6 misst den Servo mit derselben Infra.
   - NVS-Param-Blob (UnitCalib) laden/speichern; Bring-up läuft mit den
     kompilierten Nominal-Defaults, Auto-Ident schreibt später (Block 6).
2. **Compile** (`uv run labctl` nutzt esphome; `esphome compile
   firmware/slider_lab_rig.yaml`-Äquivalent für die erweiterte YAML).
   Binärgröße + RAM prüfen. SIL-Suite muss weiter grün sein (Core
   unangetastet oder Änderungen rückgespiegelt).
3. **FLASH-GATE: Jans OK einholen. Ohne OK kein Flash.** Danach OTA nur auf
   slider-lab, `labctl cycle`, Preflight.
4. **HIL-Bring-up** (hil-runner, klein anfangen):
   a. A9: Tick-p99/Misses aus den Diagnose-Countern, ADC-Fehler beobachten.
   b. Erste kleine Moves mid-travel, Recorder-Trace ziehen, gegen
      SIL-Erwartung legen (Landung, Pulszahl, Settle).
   c. **Settle-Zeiten RAUF-tunen** (Spec §5: Sim-Wert, post-pulse creep
      ≤1.3 %/400 ms ist unmodelliert — impulse_settle/short + crawl period
      floor am Gerät nachziehen).
   d. Akzeptanz am Rig: A1 (40 randomisierte Moves), A2/A3 (Stille nach
      Settle), A5, A7; Vergleichsplots gegen die Legacy-Recordings aus
      Phase 2 → `docs/findings/`.
   e. CRAWL: Pacing über mehrere Raten (5/15/30 %/s), degraded-Flag,
      Override-Checks vorsichtig mechanisch (kein Dauergrinding).
5. **Jan-gebundene HIL-Items** (abends einsammeln, nicht blockierend):
   CRAWL-Akustik (Klackern? Dither-Fallback nötig? Spec §6 f), A6-Thunk,
   A8-Grab mit echter Hand, Endzonen-Dead-Band-Vermessung (span_lo/hi).
6. **STRETCH — Auto-Ident On-Device** (`docs/auto-ident-spec.md`):
   Breakaway-Rampe, Impuls-Leiter → UnitCalib → NVS. Erst wenn 4 stabil.

## Leitplanken

- Flash NUR slider-lab, NUR mit Jans OK (Allow-Rules bestehen fürs Lab-FW).
- 30 s Pausen zwischen Aktuations-Chunks (L9110 ~800 mA cont.); Anschläge
  erlaubt, kein absichtliches Grinding. Thermal-EMA des Servos beobachten.
- Commits klein/oft/deutsch beim Orchestrator, kein Push. Tracker aktuell.
- Rig zickt (ADC flach 0): OTA-Reboot versuchen, dokumentieren; hilft er
  nicht → stoppen, nicht stundenlang retryn.
- Abends: `docs/findings/hil-<datum>.md` (TL;DR: lebt er? A-Metriken?
  Retunes? Überraschungen?), Rig idle, Torque 0.

## Für Jan

- **Flash-OK** ist das einzige harte Gate vor „alive".
- Ohr + Hand: CRAWL-Klackern, A6, A8-Grab, Dither-Entscheid (§6 f).
- Wenn der Servo läuft: Phase-8-Frage (kueche_mief-Migration) wird real.
