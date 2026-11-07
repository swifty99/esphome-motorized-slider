# Küche-Mief-Spec — echte Hardware, Ident auf dem Gerät, Position per HA

> **FREIGEGEBEN 2026-11-07 (Jan).** Graph §3, R1–R8 und alle Vorschläge aus §9
> sind bestätigt („na klar, Hauptsache es geht"); Antworten zu Q1/Q2 stehen in
> §9. Die Flash-Gates bleiben bestehen: **jeder Flash braucht Jans OK.**
> (Muster aus `feedback_spec_reimpl_template`: Graph zuerst, Invarianten als
> falsifizierbare Tripwires, Alt-Code nur für Namen/Timing.)
> Baut auf: `controller-spec.md` (freigegeben, Core bleibt Gesetz),
> `auto-ident-spec.md` (Draft, wird hier produktiv gemacht und korrigiert),
> `glide-mode-spec.md`, `plant-model.md`. Phase 8 aus `plan.md`.

## 0. Ziel und Jans Entscheidungen (2026-11-07)

**Ziel (Jan):** Der Küche-Mief-Controller setzt die Position der Slider per HA
auf einen Wert. Die Ansteuerung kommt mit den massiven Unzulänglichkeiten der
Hardware klar. Das echte Gerät, ohne Oszi: die Systembeschreibung (Plant) wird
**auf dem Gerät gelernt und eingetragen** (NVS), nicht am Scope gemessen.

| # | Entscheidung | Folge |
|---|---|---|
| E1 | Scope: **beide Slider**, Licht UI zuerst | Wrapper braucht 2 Instanzen (§2 R1) |
| E2 | Semantik: **explizites Soll-Number je Slider**, kein Mirror | Bestehende Dunst-Logik bleibt unberührt; Mirror = späterer, separater Schritt |
| E3 | Ident per **HA-Button, Anschläge erlaubt**, nie automatisch beim Boot | Ohne gültigen NVS-Stand: konservative Defaults + Flag `nicht identifiziert` |
| E4 | **Kein Touch-Grab**, nur Poti-Grab wie auf der Rig (~0,4 s Floor) | Touch bleibt reiner HA-Sensor; Controller-Graph unverändert |

Nicht-Ziele: Mirror auf `Dunstabzug Effektiver Sollwert`, Rastpunkte/Force
Feedback (v1.1), Strommessung (Phase 7), Scope-Messungen.

## 1. Ist-Aufnahme: Rig ↔ Küche Mief

Quelle: `kueche_mief_ESP32.yaml` (Produktion), `kueche_mief_ESP32_motsliddf.yaml`
(alter Versuch mit `motorized_slider`), HA-Abfrage 2026-11-07.

| Aspekt | Rig (alles gemessen) | Küche Mief (Stand der Config) |
|---|---|---|
| Chip | ESP32-S2 Mini | **ESP32-S2 Mini** (gleich, `lolin_s2_mini`, kein FPU) |
| Last | nur WiFi + Servo | WiFi, 2× BME280 (195 ms), UART-PM-Sensor, UDP-Transport, esp32_touch, LEDC, Regler v2 jede 1 s |
| Slider 1 „Licht UI" | — | Wiper **GPIO5**, Motor GPIO10/11 (heute plain `gpio`-Switch), Touch GPIO7 |
| Slider 2 „Dunstabzug" | — | Wiper **GPIO6**, Motor GPIO12/16 (LEDC 19531 Hz), Touch GPIO8 |
| Pot-Orientierung | invertiert (raw ≈ 8000 @ 0 %, ≈ 100 @ 100 %) | Slider 1 **invertiert** (Legacy: 0,079 V → 100 %, 2,57 V → 0 %); Slider 2 **normal** (0,155 V → 0 %, 2,70 V → 100 %) |
| Motorpolarität | AIN1 = vorwärts = Position hoch | **unbekannt je Slider** (motsliddf: `pwm_forward: motor_1_l`) |
| Breakaway | Cliff 0,66–0,70, ±0,04 | **unbekannt.** Legacy-Config hatte `static_friction 0.43` (Hinweis, unbewiesen: anderes Duty-Mapping oder andere Mechanik) |
| Treiber / Motorspannung | L9110, Powerbank | **unbekannt.** HA zeigt Board-Rail `kueche_mief_24v_versorgung` ≈ 22 V; ein L9110 verträgt ≤ 12 V → Treiber/Regler klären (§9 Q1) |
| ADC | `adc_oneshot` ADC1 ch8 allein | ESPHome-`adc` auf GPIO1 (Versorgung), GPIO5/6 (Wiper) → **ADC1-Unit-Konflikt** (§2 R2) |
| Sync-Pin | GPIO14 (Scope) | GPIO14 = **I2C SCL** → Sync-Pin muss optional werden |
| HA-Altlasten | — | keine: weder `slider_sollwert` noch `motor_1`/`dither` in der Registry → Namen frei |

Aus HA (live): `sensor.kuche_licht_ui` = 31,6 %, `sensor.dunstabzug_sollwert_ui`
= 0,08 %, Touch-Binärsensoren beide `off`, Versorgung 22,1 V.

**Die Lehre daraus:** Die Rig-Konstanten (Impuls-Duty 0,8, `v_min_cmd` 120,
Bracket ab 0,60, σ_PWM-Stall bei 0,45) sind **Eigenschaften einer Einheit,
keine Naturkonstanten**. Auf einer Einheit mit Breakaway ≈ 0,43 würde der
0,45-Stall den Slider bewegen und ein 0,8-Impuls ihn weit überschießen. Der
Ident muss deshalb nicht nur Werte füllen, sondern den **Suchbereich selbst
bestimmen**, und der Core muss die Impuls-Duty aus UnitCalib lesen (R7).

## 2. Anforderungen an Wrapper und Integration

**R1 — Zwei Instanzen, ein Motor zur Zeit.** `slider_servo` wird multi-instanz-
fähig (ID je Slider). Ein **Arbiter** erlaubt höchstens eine Instanz mit
Drehmoment ≠ 0; ein zweites Kommando wird in der Reihenfolge des Eintreffens
**gequeued** (nicht verworfen, nicht parallel). Grund: Tick-Budget (Rig p99
525 µs bei 1 ms Takt, S2 ohne FPU: zwei aktive Observer sprengen es) und
L9110-Strom (~800 mA je Kanal). Eine **IDLE-Instanz** darf nur ADC-Sampling +
Zustandsupdate kosten (Ziel ≤ 100 µs, per A9 zu belegen); wo das nicht reicht,
fällt sie auf reduziertes Sampling (≥ 100 Hz) zurück, solange sie IDLE ist.
Je Instanz: eigener Timer-Slot oder gemeinsamer Tick (Entscheidung in §3-Review),
eigener NVS-Key (`SVRC` + Instanz-Index), HTTP-Dump unter `/servo/<id>/dump.csv`
auf **einem** httpd.

**R2 — ADC-Koexistenz.** Zwei Besitzer von ADC1 gehen nicht (`adc_oneshot_new_unit`
schlägt beim zweiten Aufruf fehl). Vorschlag: **`slider_servo` besitzt ADC1**
und liefert alle ADC1-Kanäle der Karte (Wiper 1, Wiper 2, Versorgung GPIO1) als
Sensoren aus; die ESPHome-`adc`-Plattform entfällt für diese drei Pins. Die
Entitäten `Küche Mief 24V Versorgung` und `Küche Licht UI` behalten ID, Namen,
Einheit, device_class, Filter und Werte-Semantik (R3). Jede Alternative
(geteiltes Unit-Handle) braucht Jans OK, weil sie ADC-Locking im Tick riskiert.

**R3 — Legacy-Konsumenten bleiben unverändert.** `position_slider_1`
(→ `sensor.kuche_licht_ui`, geht per `packet_transport` an nordlichter) und
`position_slider_2` (speist `position_slider_2_ext`, `Dunstabzug Effektiver
Sollwert`) behalten ID, Name, Einheit (%) und Filter (delta/throttle). Der
Servo liefert die kalibrierte Position als Quelle dahinter; die bisherigen
`calibrate_linear`-Datapoints werden durch die per Ident gelernten Endpunkte
ersetzt (Jan bestätigt den YAML-Diff, CLAUDE.md: nur nach Bestätigung).
Bewegt der Motor Slider 2, laufen die Konsumenten mit (bei „HA-Sollwert aus"
folgt der Lüfter dem fahrenden Slider; bei „HA-Sollwert an" wird er
ignoriert). Das ist gewollt und im Status-Text sichtbar.

**R4 — Neue Entitäten (Vorschlag, Jan bestätigt, danach eingefroren).**

| Slider | Entität | Typ | Zweck |
|---|---|---|---|
| Licht UI | `Küche Licht UI Soll` | number 0–100 %, step 0,5, `restore_value: false` | Position setzen |
| Licht UI | `Küche Licht UI Status` | text_sensor | idle / moving / crawl / released(grab\|cmd\|interlock\|watchdog\|fault) / ident(Phase) / nicht identifiziert |
| Licht UI | `Küche Licht UI Ident` | button | Ident starten |
| Licht UI | `Küche Licht UI Abbruch` | button | Abbruch, Drehmoment 0 ≤ 2 Ticks |
| Licht UI | `Küche Licht UI Ident gültig` | binary_sensor (diagnostic) | NVS-Stand gültig + Stempel passt |
| Licht UI | `Küche Licht UI Ident Bericht` | text_sensor (diagnostic) | Kurzbericht der letzten Läufe (Werte + Ursache bei FAIL) |
| Dunstabzug | `Dunstabzug Slider Soll`, `… Status`, `… Ident`, `… Abbruch`, `… Ident gültig`, `… Ident Bericht` | analog | analog |
| Gerät | `Slider Tick p99`, `Slider Ticks verpasst`, `Slider ADC Fehler` | sensor (diagnostic) | A9 im Betrieb |

Vorhandene Entitäten (`Küche Licht UI`, `Dunstabzug Sollwert UI`,
`Touch Slider 1/2`, …) werden **nicht** umbenannt.

**R5 — Boot und Reconnect bewegen nie.** Weder Boot noch HA-Reconnect noch
OTA noch Restore eines Number-Werts darf eine Fahrt auslösen (Lehre aus
`project_reconnect_phantom_ha_automations`; die motsliddf-Variante hatte
`restore_value: true` + `on_value` und wäre beim Booten losgefahren). Eine
Fahrt entsteht **nur** aus einem aktiven `set`-Aufruf nach abgeschlossenem Boot.
Boot-Zustand ist IDLE mit Drehmoment 0, Hold := gemessene Position.

**R6 — Polarität wird gelernt, nicht konfiguriert.** Verdrahtung von Motor und
Pot-Orientierung sind je Slider unbekannt und dürfen nicht als YAML-Glaubenssatz
einfließen. Ident-Phase I2 bestimmt `dir_sign ∈ {+1, −1}` (Duty-Vorzeichen →
Positionsrichtung) per Probe-Impuls und speichert es in UnitCalib. `invert_direction`
aus dem YAML wird zum bloßen Startwert für Phase I2.

**R7 — Per-Unit-Ableitungen im Core (SIL-neutral).** Drei Konstanten werden aus
UnitCalib abgeleitet statt fest zu sein, jeweils mit Default = bisheriges
Verhalten, sodass SIL 30/30 grün bleibt (Muster `crawl_impulse_duty`, Commit
`e28f965`):
(a) `impulse_duty = clamp(F_s_p95 + 0,10, 0,60, 0,95)` (Tabelle wird dann bei
genau diesem Duty gemessen, I8); (b) `u_max = clamp(F_s_p95 + 0,15, …, 1,0)`
(Grab-Diskriminierung braucht u_max > schlimmster Breakaway, sonst nie Grab);
(c) `v_min_cmd` folgt aus gemessener Flug-Geschwindigkeit eine Stufe über dem
Cliff statt 120 %/s fix. Änderungen an Core-Konstanten ohne Spec-Zeile sind
verboten (controller-spec §4 #11).

**R8 — Unidentifiziert heißt gedrosselt, nicht gesperrt.** Ohne gültigen
NVS-Stand fährt der Servo mit sicheren Nominalwerten, **begrenzt**: nur
Endgame-/Impuls-Pfad mit kleinen Breiten, kein MOVE-Cruise, Status
`nicht identifiziert`. Begründung: lieber ein langsamer, ehrlich markierter
Slider als ein 0,8-Duty-Impuls auf einer Einheit, die bei 0,43 losgeht.

## 3. Ident-Programm: Trigger-Graph (zur Freigabe)

Neue Ebene **L6 Ident** neben L0–L5. Sie besitzt den Motor exklusiv (Controller-
Zustand während Ident: `IDENT`, `set_target` wird nicht ausgeführt, sondern mit
Status `ident_läuft` abgelehnt). Zustände: `I0 OFF`, `I1 PREFLIGHT`,
`I2 POLARITY`, `I3 ENDPOINTS`, `I4 QUIET`, `I5 BRACKET`, `I6 STALL`,
`I7 FLIGHTS`, `I8 IMPULSES`, `I9 COMMIT`, `FAIL(reason)`, `ABORT`.
Trigger: `CMD` (Button), `TICK`, `OBS`, `ILK` (L1), `TIME`. Nicht aufgeführte
(Zustand, Trigger)-Paare sind No-Ops; die SIL-Harness prüft das (T15-Muster).

| # | Von | Trigger | Guard | Nach | Aktion |
|---|---|---|---|---|---|
| 1 | I0 | CMD Ident | Controller IDLE, keine andere Instanz Drehmoment ≠ 0 | I1 | Instanz sperrt Arbiter; Status `ident(preflight)` |
| 2 | I0 | CMD Ident | sonst (Instanz aktiv, RELEASED, Fault) | I0 | Ablehnung mit Grund im Status |
| 3 | I1 | TICK Preflight ok | ADC nicht flach 0 / nicht an Schiene, σ-Probe 0,5 s plausibel, Thermal-EMA < 0,1, Versorgungs-ADC (wenn vorhanden) im Fenster | I2 | VM-Wert stempeln |
| 4 | I1 | TICK Preflight fail | — | FAIL | reason ∈ `adc_flat`, `adc_rail`, `thermal`, `supply` |
| 5 | I2 | TICK | Position 15–85 % | I2 | Probe-Impulse 0,8 × {5, 10, 20, 40} ms, Duty-Vorzeichen +, jeweils 300 ms Ruhe; Bewegung ≥ 1 % (Median über 8 Samples) beendet die Probe |
| 6 | I2 | TICK Position < 15 % oder > 85 % | — | I2 | erste Probe in Richtung Mitte (geschätzt über Poti-Skala), `dir_sign` danach aus beobachteter Richtung |
| 7 | I2 | OBS Bewegung gesehen | — | I3 | `dir_sign` := sign(Δpos)·sign(Duty) |
| 8 | I2 | TICK nach Wiederholung × 2 ohne Bewegung ≥ 1 % | — | FAIL | `no_motion` (nicht verdrahtet / blockiert / Treiber / zu niedrige Spannung) |
| 9 | I3 | TICK | — | I3 | Impuls-Zug (20 ms @ 0,8 alle 150 ms) Richtung Schiene A; Stopp, wenn 3 Impulse in Folge < 0,3 % Fortschritt |
| 10 | I3 | OBS Schiene A erreicht | — | I3 | 0,5 s Ruhe, Median 200 Ticks → `raw_A`; dasselbe Richtung B → `raw_B` |
| 11 | I3 | OBS beide Schienen | \|raw_A − raw_B\| ≥ `min_span` (4000 counts), Ruhe-σ plausibel | I4 | `raw_at_0`, `raw_at_100` aus Orientierung; nutzbare Spanne [`span_lo`, `span_hi`] |
| 12 | I3 | OBS beide Schienen | Spanne zu klein / gleiche Seite | FAIL | `span` |
| 13 | I4 | TIME 2 s | — | I5 | σ₀ := MAD·1,48 über Ruhe-Ticks |
| 14 | I5 | TICK | — | I5 | **adaptiver Bracket**: Haltephasen 300 ms ab Duty 0,15 in Schritten 0,05 aufwärts bis zur ersten Bewegung ≥ 5 % (Median ≥ 8 Samples); danach Feinschritt 0,03 abwärts-bestätigt; 7 Positionen × 2 Richtungen × 2 Wdh., zwischen den Punkten Repositionierung per Impulsen |
| 15 | I5 | OBS Bewegung | — | I5 | sofort Duty 0; Onset-Duty := Duty am letzten ruhigen Tick (32er-Ring); Sustain-Bit nach 50 ms |
| 16 | I5 | TICK Duty > 0,95 ohne Bewegung | — | FAIL | `no_breakaway` |
| 17 | I5 | OBS alle Punkte | F_s_p95 in [0,25; 0,92] | I6 | Impuls-/u_max-Ableitungen R7 berechnen |
| 18 | I6 | TIME 2 s | Stall-Duty := max(0,10; F_s_min − 0,10) | I7 | σ_PWM + Spike-Rate (>5σ-Ticks) |
| 19 | I7 | TICK | — | I7 | je Richtung 2 stationäre Flüge (Duty F_s_p95+0,12 und +0,20, 25→75 %) → 2×2-Lösung u = F_k + b·v aus letzten 40 ms |
| 20 | I7 | OBS 4 Flüge pro Richtung | F_k < F_s, b > 0 | I8 | — |
| 21 | I7 | OBS | F_k ≥ F_s oder b ≤ 0 | FAIL | `plant_implausible` |
| 22 | I8 | TICK | — | I8 | bei `impulse_duty` (R7a) Impulse 3/5/10/20/30/50 ms je Richtung, ≥ 2 Wdh. aus Mitte, Median je Breite → Tabelle; nach jeder Gruppe zurück in die Mitte |
| 23 | I8 | OBS Tabelle vollständig | monoton in Breite, > 0 | I9 | — |
| 24 | I8 | OBS Tabelle unplausibel | — | FAIL | `table_implausible` |
| 25 | I9 | TICK | — | I0 | UnitCalib + Stempel (fw, VM, Endpunkt-ID, Zählerdatum) nach NVS **atomar** (Blob schreiben, dann Commit-Zähler); Status `ident gültig`; Bericht |
| 26 | beliebig I1…I9 | CMD Abbruch | — | ABORT | Duty 0 ≤ 2 Ticks; alter NVS-Stand bleibt |
| 27 | beliebig I2…I8 | OBS Bewegung bei Duty = 0 | > 3 % über 100 ms | ABORT | reason=`hand`; nie gegen eine Hand fahren |
| 28 | beliebig I2…I8 | ILK Thermal/Budget | — | ABORT | reason=`interlock` |
| 29 | beliebig I2…I8 | TIME Phasen-Watchdog | 2 × erwartete Phasenzeit + 1 s | FAIL | `watchdog` |
| 30 | beliebig I2…I8 | OBS Versorgung weicht > 15 % vom Stempel ab | — | ABORT | reason=`supply` |
| 31 | FAIL / ABORT | TICK | — | I0 | Drehmoment 0; **vorheriger Stempel bleibt**; Arbiter frei; Status mit Grund |
| 32 | beliebig | OBS Fault (NaN, ADC-Fehlerrate) | — | ABORT | reason=`fault` |

**Budget:** ≤ 120 s inkl. Pausen und Repositionierungen (Rig-Schätzung 75 s,
dazu Polarität/Endpunkte und der breitere Bracket). 30-s-Pausen-Regel der
Bench gilt zwischen **Läufen**, nicht innerhalb; Thermal-EMA bleibt über den
L1-Interlock gedeckelt. Schienenfahrten sind erlaubt (E3, Endzonen-Policy (a):
100 % Duty für 10 s ist ok).

## 4. Tripwires (jeder: SIL-Assertion + Negativ-Kontrolle)

| # | Falsifizierbare Aussage | Negativ-Kontrolle, die ihn brechen muss |
|---|---|---|
| K1 | Boot, API-Reconnect, OTA und Number-Restore erzeugen **0 Ticks mit Duty ≠ 0** | `restore_value: true` + `on_value` einbauen |
| K2 | Ident-Bracket-Start ist unabhängig von Rig-Konstanten: auf einer Sim-Einheit mit Breakaway 0,35 **bewegt sich kein Slider vor der ersten Bracket-Stufe** (kein 0,45-Stall) | Bracket ab 0,60 / festen Stall 0,45 |
| K3 | Auf Sim-Einheiten mit **vertauschter Polarität** und **invertiertem Pot** (alle 4 Kombinationen) landet ein Move auf ±2 % | `dir_sign` fest +1 |
| K4 | Ident-Fehler (jede der FAIL-Ursachen aus §3) lässt den **vorherigen NVS-Stempel bit-identisch** und Duty = 0 | NVS vor Phase I9 schreiben |
| K5 | Hand während Ident (Bewegung bei Duty 0) → ABORT, Duty 0 ≤ 2 Ticks (Ident-Pendant zu T6) | Row 27 löschen |
| K6 | Arbiter: **nie zwei Instanzen mit Duty ≠ 0** im selben Tick; Queue erhält Reihenfolge, kein Kommando geht verloren | Arbiter-Sperre entfernen |
| K7 | R7: auf Sim-Einheit mit Breakaway 0,43 bleibt jeder Impuls ≤ `impulse_duty` und `u_max > F_s_p95`; Grab wird erkannt | `impulse_duty` fest 0,8, `u_max` fest 0,9 |
| K8 | Unidentifiziert (R8): Moves landen innerhalb 2 % **ohne** Cruise-Pfad, Status trägt `nicht identifiziert` jeden Tick | Flag nach erstem Move löschen |
| K9 | Zwei Idents hintereinander: Breakaway ±0,03 Duty, b ±15 %, Impuls-Weiten ±10 % (auto-ident-spec Acceptance) | Mittelung durch Einzelprobe ersetzen |
| K10 | A9 im Gesamtsystem: Tick p99 ≤ 900 µs, ≤ 1 verpasst/250 k, mit **beiden** Instanzen, voller Küche-Last und laufendem Ident | beide Instanzen gleichzeitig aktiv rechnen lassen |
| K11 | Entity-Namen: Diff von `ent ls` vorher/nachher auf der Registry zeigt **nur Zugänge**, keine Umbenennung/Entfernung | eine bestehende Entität umbenennen |

K2/K3/K7 setzen **Plant-Varianten im SIL** voraus (neue Arbeit, §7 Stufe 0):
Breakaway 0,25–0,85, Polarität ±, Pot ±, Rauschen ×1–×3, Backlash bis 2 %,
Versorgung ±15 %. Das ist die eigentliche Antwort auf „massive Unzulänglichkeiten":
der Ident wird gegen eine **Schar von Plants** abgesichert, nicht gegen die Rig.

## 5. Parameter und NVS

UnitCalib (je Instanz) trägt, zusätzlich zu `auto-ident-spec.md` §Parameter-Tabelle:
`dir_sign`, `raw_at_0`, `raw_at_100`, `span_lo`, `span_hi`, `impulse_duty`,
`u_max`, `v_min_cmd`, `F_s(7×2)` + Sustain-Bits, `F_k`, `b` je Richtung,
Impuls-Tabelle 2×6, σ₀, σ_PWM, Spike-Rate. **Stempel:** fw-Version, Instanz-Index,
VM-Bucket (falls Versorgung messbar), Endpunkt-ID, Zählerdatum, Commit-Zähler.
Ungültig, wenn fw-Hauptversion, Instanz-Index oder Endpunkt-ID nicht passen.
Ca. 150 Byte je Instanz. Die Auflösung der drei offenen Fragen aus
`auto-ident-spec.md`: Q1 (Stall bei 0,45) → **ersetzt durch abgeleiteten Stall
nach Bracket** (I6); Q2 (Anschläge) → **ja** (E3); Q3 (Tabelle roh in NVS oder
in FF-Koeffizienten) → **roh** (einfacher zu validieren).

## 6. HA-API je Slider

`set` → `Soll` (Number, kein Restore), liefert Status; `Ident`/`Abbruch` Buttons;
`Status` als Text; Diagnose-Sensoren wie in R4. Zusätzlich ein ESPHome-Service
`slider_ident_report` (Rückgabe: UnitCalib als JSON) für den Offline-Abgleich
mit `plantreport`. Kein Mirror, keine Automation in der Firmware (E2).

## 7. Stufenplan mit Gates

| Stufe | Inhalt | Gate |
|---|---|---|
| 0 | **Offline:** Wrapper-Umbau (Multi-Instanz, ADC1-Besitz, Sync-Pin optional, Arbiter, `/servo/<id>/dump.csv`), Ident L6 im Core, R7-Ableitungen, **SIL-Plant-Schar**, Tripwires K1–K9 | SIL 30/30 + neue Tests grün, Compile grün, alle Negativ-Kontrollen brechen |
| 1 | **Nur Lesen** am echten Gerät: Wiper 1/2 + Versorgung über `slider_servo`, Motoren **aus**; Vergleich mit den Legacy-Sensoren, Tick p99 unter Küche-Last (K10 ohne Motor) | Jan-OK für Flash; Werte decken sich mit `sensor.kuche_licht_ui` ±1 %, p99 ≤ 900 µs |
| 2 | **Motor-Smoketest** je Slider, Jan beobachtet: nur Phase I2 (Polarität), Impulse klein | Bewegung + `dir_sign` plausibel, nichts schlägt an |
| 3 | **Ident-Lauf** je Slider, 2× hintereinander, Dump + `plantreport`-Abgleich | K9 + Cross-Check (Gerät ≈ Offline) |
| 4 | **Moves per HA:** 40 Zufallsmoves (A1-lite) je Slider, `Soll`-Number, Status ehrlich; Reboot-/Reconnect-Test (K1) im Betrieb; 24 h Soak mit Torque-0-Prüfung | ≥ 95 % innerhalb 2 %, K1/K6/K10/K11 grün |
| 5 | **Migration:** YAML-Diff (R3) Jan zeigen, nach OK in `kueche_mief_ESP32.yaml` übernehmen, Entity-Diff K11 | Jans OK auf den Diff, danach Wartungslog |

Jeder Flash braucht Jans OK (`feedback_flash_approval`). Das Gerät ist
**produktiv** (Dunstabzug, KWL-Interaktion, Wasser): Stufe 1–3 laufen auf der
Produktions-Firmware mit `slider_servo` und deaktivierter Motorausgabe, oder auf
einer abgespeckten Bring-up-Firmware, wenn Jan einen Ausfall der
Dunst-/Bewässerungslogik für die Testzeit akzeptiert (§9 Q2).

## 8. Bekannte Risiken

- **Treiber/Spannung** (§9 Q1, geklärt): Motor an 5 V vom Platinen-DCDC. Offen
  bleibt nur der Treiberchip selbst (Annahme L9110 wie Rig); Stufe 2 prüft
  Verhalten statt Annahme (Polarität, Bewegung, Duty-Mapping).
- **Tick unter Küche-Last:** Rig-Budget war ohne BME280/UART/Touch gemessen; das
  esp_timer-gegen-WiFi-Problem (siehe `reference_esp32s2_realtime_gotchas`) kann
  mit mehr Tasks zurückkommen. K10 ist deshalb hartes Gate.
- **ADC-Vorfall** (Rig: raw flach 0, nur OTA-Reboot half): Preflight (§3 Row 4)
  fängt ihn vor jeder Fahrt; Fehlerzähler läuft als Diagnose-Sensor.
- **Motor-Wear:** Ident ist ~25 Moves + Anschläge; kein Dauerlauf, Thermal-EMA
  im Bericht.
- **Dunst-Konsumenten folgen dem fahrenden Slider 2** (R3): gewollt, aber bei
  Tests Lüfter-Ruck einplanen.

## 9. Fragen an Jan — beantwortet 2026-11-07

**Antworten:** Q1: Motorspannung **5 V vom DCDC der Platine** (kein Problem für
den L9110, entspricht der Powerbank-Rig; `K ∝ VM` bleibt vergleichbar, die
22-V-Messung ist nur das Board-Rail). Q2: **Das Küche-Mief-Gerät hängt für
Dunst nicht am Slider** — dort läuft nur Automatik-Betrieb, der Slider darf
tun, was er will, und das Gerät ist der erste Einsatzort (Stufe 1–3 laufen auf
der Produktions-Firmware, kein abgespecktes Bring-up nötig, Ausfallfenster ist
unkritisch). Q3–Q7: **alle Vorschläge bestätigt** (Namen R4, Arbiter, SIL-first,
ADC1-Besitz, R8-Drosselung). Folge für R3: die Sorge „Lüfter folgt dem fahrenden
Slider" entfällt im Automatik-Betrieb.

Ursprüngliche Fragen (zur Nachvollziehbarkeit):

1. **Treiber und Motorspannung** beider Kanäle: Welcher H-Brücken-Chip sitzt auf
   der Küche-Platine, woher kommt die Motorspannung (Board-Rail ≈ 22 V gemessen,
   ein L9110 verträgt ≤ 12 V)? Gleiche Motoren/Fader wie auf der Rig (ALPS?)?
2. **Zugang und Ausfallfenster:** Liegt das Gerät erreichbar (Bench) oder ist es
   verbaut? Darf die Küche-Mief-Logik (Dunst, Bewässerung, AQI) für Stufe 1–3
   ausfallen, oder muss alles auf der Produktions-Firmware laufen?
3. **Namen:** Passen die Entitäts-Vorschläge aus R4 (nach der Bestätigung
   eingefroren)?
4. **Arbiter „ein Slider zur Zeit":** Ok, oder müssen beide gleichzeitig fahren
   können (dann Tick-Budget-Rechnung neu)?
5. **SIL-first:** Ok, dass Stufe 0 eine Plant-Schar im SIL baut, bevor das erste
   Flash passiert?
6. **R2:** Ok, dass `slider_servo` ADC1 besitzt und die Versorgungsspannung samt
   Wipern liefert (Namen und Werte der Entitäten bleiben)?
7. **Unidentifiziert (R8):** Ok, dass ein Slider ohne Ident nur gedrosselt fährt,
   statt ganz gesperrt zu sein?
