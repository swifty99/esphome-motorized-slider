# Motorslider — Entwicklungsbericht (Juli bis November 2026)

Zum Nachschlagen: Was wurde gebaut, wie, warum, was kam heraus. Details stehen in den verlinkten Dokumenten.

## 1. Ziel

Der alte Controller (`EsphomeZoo/components/motorized_slider`, ALPS-RS60-Fader, Riemenantrieb, DC-Motor, Poti-Rückmeldung)
war laut, ungenau und verlor sich in Regelschleifen. Neubau von Grund auf („clean room"), mit dem Endziel:
**Der Küche-Mief-Controller setzt die Slider per HA auf einen Wert, und die Ansteuerung kommt mit den Unzulänglichkeiten
der Hardware klar** (Haftreibung, verrauschtes Poti, unbekannte Polarität, Sensor-Sättigung).

## 2. Der Weg

| Schritt | Was | Warum | Ergebnis |
|---|---|---|---|
| Autopsie (07-16) | Alt-Code analysiert (`legacy-analysis.md`) | Anforderungen und Fehler verstehen, nicht kopieren | 3 Hauptfeinde: Reibungs-Feedforward als Relais, blinde Geschwindigkeit unter 9 %/s, unmodellierter Backlash; 7 verifizierte Bugs |
| Lab-Rig + Messkampagne (07-16/17) | Experiment-Firmware `slider_lab`, 127 Messläufe, PicoScope | Erst die Strecke messen, dann regeln | **Die Strecke ist eine Klippe**: darunter (≈0,66–0,70 Duty) steht der Slider, darüber fliegt er mit >100 %/s; kein Schleichbereich, Dither hilft nicht (`plant-model.md`) |
| Controller-Spec (07-17) | Zustandsmaschine mit 29 Zeilen, Invarianten als Tripwires | Spec zuerst, Jan gibt den Graphen frei | Langsame Fahrt nur als Impulszug (CRAWL), Landung nur vorwärts, „Hand hat immer recht" (adoptieren, nie kämpfen) |
| Core + SIL (07-17) | Reiner C++-Kern ohne IDF, Simulator, pytest, Negativ-Kontrollen | Jede Invariante muss beweisbar brechen können | 17 Tripwires grün; 3 latente Fehler gefunden, bevor Hardware lief |
| ESPHome-Komponente + Rig-HIL (07-17/18) | `slider_servo` als Wrapper um den Kern, auf der Rig vermessen | Ziel-Deliverable ist die ESPHome-Komponente | Rig-Baseline: Median-Fehler klein, ~0,4 % max, Boot drehmomentfrei, Tick-p99 ~525 µs |
| Feinschliff (07-18) | CRAWL-Akustik, GLIDE (Dauerfahrt statt Klacken) Stufe 1 | Jans Ohr: Klackern und Riemenlast | GLIDE nur auf der Rig, in der Küche aus (`glide: 0`) |
| Küche-Spec (11-07) | `kueche-mief-spec.md`: 2 Slider, Soll-Number, Ident auf dem Gerät | Jede Einheit ist anders, kein Oszi im Feld | Freigegeben am gleichen Tag |
| Küche-Umsetzung (11-07) | Multi-Instanz-Wrapper, Ident-Programm L6 (32 Zeilen), Plant-Schar im SIL, dann Flash und Ident am echten Gerät | Offline absichern, dann an die Hardware | siehe Abschnitt 4 |

Arbeitsweise: Spec zuerst und von Jan freigegeben, dann Implementierung durch Subagenten (Core und Wrapper getrennt, je eigener
Worktree), Hardwarezeit nur seriell durch mich, jeder Flash nur mit Jans OK, jede Änderung committet.

## 3. Wichtigste Erkenntnisse

- **Rig-Konstanten sind Einheiten-Eigenschaften.** Breakaway, Impulsweg, Polarität und Poti-Orientierung unterscheiden sich pro Slider.
  Deshalb misst der Ident alles selbst und leitet `impulse_duty`, `u_max` und `v_min_cmd` pro Einheit ab.
- **Annahmen an der Hardware prüfen, nicht schätzen.** Die geratene Polarität von Slider 2 war falsch; sein Wiper sättigt oben
  am ADC (raw 8191, σ = 0), was die erste Plausibilitätsprüfung für defekt hielt.
- **Ein Crash ist nicht immer ein Crash.** ESPHome meldet einen Task-Watchdog als `IllegalInstruction`. Erst der Reset-Grund
  (`Küche Mief Reset Grund`) zeigte es; Ursache war ein zu voller Tick (zweite Instanz sampelte den ADC bei jedem Tick).
- **Der SIL fängt Logik, nicht Hardware.** Alle drei Feldfehler (Sättigung, σ0 in der Klemme, Boot-Transient der Versorgung)
  wurden danach im SIL reproduziert und mit Negativ-Kontrolle abgesichert.
- **Kein Phantom-Fahren.** Boot, Reconnect und OTA bewegen nie (kein Restore auf dem Soll-Number).

## 4. Ergebnis (Küche, 2026-11-07)

- Beide Slider identifiziert (Button, 118–140 s, NVS-persistent, überlebt Reboots). Slider 1: `dir +1, Fs95 0,74, impulse_duty 0,84`;
  Slider 2: `dir −1, Fs95 0,67, impulse_duty 0,77`, oben geklippt.
- Landetest (12 Zufallsmoves je Slider): alle `reached`, Median-Fehler 0,10 % (Slider 1) und 0,15 % (Slider 2), Maximum 0,31 % / 0,33 %.
  Ohne Ident landete Slider 1 ca. 2 % vor dem Ziel.
- Per HA abgenommen (`number.set_value`, Knöpfe Ident/Abbruch, Status); manuelles Spielen in der HA-Oberfläche lief sauber.
- Soak: 7,7 h ohne Reset, 0 ADC-Fehler, Dunst-Automatik unverändert.
- SIL: 194 Tests grün (1 `xfail`: Wiederholbarkeit K9 bei realer Streuung, ausgewiesen).
- Thermal-Schutz auf Jans Wunsch realistisch (τ 120 s, Trip bei 0,95 nach ca. 6 min Vollgas) statt Trip nach 7 s.

## 5. Entscheidungen von Jan (Auszug)

| Datum | Entscheidung |
|---|---|
| 07-16 | Clean-room Neubau, „nothing has to stay"; Flash nur mit OK |
| 07-17 | CRAWL (Impulszug) für Langsamfahrt; keine Endzonen-Klemme (Hardware verträgt Vollgas); Rastpunkte erst v1.1; Toleranz-Latte 2 % |
| 11-07 | Beide Slider, Soll-Number statt Mirror, Ident per Button mit Anschlägen, kein Touch-Grab, alle Spec-Vorschläge ok |
| 11-07 | Thermal-Trip realistisch machen; Flash auf kueche-mief bis 18:00 erlaubt |
| 11-07 | Legacy-Skala (`sensor.kuche_licht_ui`) bleibt vorerst unverändert, der Slider zeigt „ungefähr 0–100 %", reicht |
| 11-07 | Repo sichern (Forgejo), Entwicklungsbericht schreiben |

## 6. Reproduzieren (die wichtigsten Handgriffe)

- **SIL/Tests:** `cd analysis && uv run pytest tests/ -q` (Erwartung: 194 passed, 1 xfailed).
- **Küche bauen/flashen:** `esphome compile|upload kueche_mief_ESP32.yaml --device 192.168.178.57` im Zoo (Komponente aus `../motorslider`, Flash nur mit OK).
- **Ident:** HA-Button „… Ident" (oder `button.press`); Abbruch: Button „… Abbruch". Status/Bericht: `sensor.…_status`, `sensor.…_ident_bericht`.
- **Trace:** `curl http://192.168.178.57/servo/licht/dump.csv` bzw. `/servo/dunst_slider/dump.csv` (nur im Leerlauf, ~8 s Ring).
- **Diagnose:** `Küche Mief Reset Grund` zuerst lesen, dann `Slider Tick p99`, `Ticks verpasst`, `Heap frei`.
- Betriebsanleitung: `EsphomeZoo/docs/kueche-mief-slider.md`; Messungen: `docs/findings/kueche-hil-2026-11-07.md`.

## 7. Offen

- Watchdog beobachten (2 Resets vor dem Idle-Teiler, seitdem keiner in 5 Läufen); Test tagsüber mit weiteren Idents.
- Tick-Budget unter Ident knapp (p99 875–975 µs, Gate 900 µs).
- Rig beim nächsten Flash kurz prüfen (LEDC/httpd/Thermal haben sich geändert).
- Optional: Ident auf ca. 113 s kürzen, Entity-IDs ohne doppeltes Präfix, Mirror auf den Dunst-Sollwert, Rastpunkte (v1.1).

## 8. Wo liegt was

| Was | Wo |
|---|---|
| Spec Küche, Controller, GLIDE, Auto-Ident | `docs/kueche-mief-spec.md`, `controller-spec.md`, `glide-mode-spec.md`, `auto-ident-spec.md` |
| Messungen und Findings | `docs/findings/` (u. a. `kueche-hil-2026-11-07.md`, `ident-sil-2026-11-07.md`) und `docs/plant-model.md` |
| Repo (Sicherung, öffentlich) | GitHub `swifty99/esphome-motorized-slider`, Branches `main` = `v2`; Push mit `git pushgh` (Alias nutzt das swifty99-Token, `kipp-ing` hat dort nur Lesezugriff) |
| Komponente (Wrapper + Kern) | `firmware/components/slider_servo/` |
| SIL und Tests | `sil/`, `analysis/tests/` |
| Küche-Config und Betrieb | `EsphomeZoo/kueche_mief_ESP32.yaml`, `docs/kueche-mief-slider.md`, `docs/maintenance-log.md` |
