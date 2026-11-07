# Wrapper Stufe 0B — slider_servo fit für kueche-mief (2026-11-07)

Branch `stage0-wrapper`, reine Offline-Arbeit (Compile ja, **kein Flash**).
Gesetz: `docs/kueche-mief-spec.md` (R1–R5, R8, K1/K6/K10/K11, §6, §7 Stufe 0),
`docs/controller-spec.md` §4. Core-Dateien unangetastet.

## TL;DR

- `slider_servo` ist **multi-instanzfähig** (MULTI_CONF, max. 4). Die Rig-YAML
  kompiliert **unverändert** (784 986 B Flash, 42,8 %; RAM 125 972 B, 73,2 %;
  Baseline vor dem Umbau 779 854 B / 125 568 B).
- **Küche-Compile grün:** `firmware/kueche_mief_slider_compile.yaml`
  (2 Slider, Versorgung GPIO1 über den Wrapper, kein Sync-Pin, I2C 13/14 mit
  2× BME280, sds011, Touch 7/8, ESPHome-ledc daneben):
  **805 682 B Flash (43,9 % von 1 835 008), RAM 129 816 B (75,5 % von 172 032).**
- Ein Hub besitzt **ADC1** (eine `adc_oneshot`-Unit für alles), Tick, httpd und
  den **Arbiter** (≤ 1 Instanz mit Duty ≠ 0). Extra-ADC1-Kanäle und
  Positions-/Wiper-Sensoren kommen über `sensor: - platform: slider_servo`.
- Ident/Abbruch laufen durch den Arbiter bis zu einem **CORE-ADAPTER**-Stub
  (Status `ident nicht verfügbar`). Der Merge mit dem Core ist ein Job von
  ~10 Zeilen.
- **Offline nicht belegbar:** K10 (Tick-p99 unter Küche-Last), der ≤ 100-µs-
  IDLE-Pfad und die Kollisionsrate am geteilten ADC. Zähler dafür sind
  eingebaut, der Beleg ist Stufe 1 auf Hardware.

## Multi-Instanz: warum MULTI_CONF + impliziter Hub

`slider_servo:` nimmt ein Mapping (Rig) oder eine Liste. Gemeinsame Ressourcen
liegen in **einem** `SliderServoHub`. Codegen legt ihn einmal an; er ist keine
Component, die erste `setup()` einer Instanz initialisiert alles.
Gegen einen Hub-Block mit `servos:`-Liste sprach:

- die Rig-YAML bliebe nicht byte-identisch;
- `id(servo).command_target(x)` und alle Getter blieben nicht die Oberfläche;
- hub-weit ist nur `http_port` einstellbar, und den muss jede Instanz gleich nennen
  (FINAL_VALIDATE prüft das).

Je Instanz:

- **`index`**: NVS-Key `'SVRC' + index`. Ohne Angabe gilt die Listenposition,
  Index 0 ist der bisherige Rig-Blob mit unverändertem Layout. Bei mehreren
  Instanzen sollte der Index explizit gesetzt werden, damit Umsortieren in der
  YAML keine Kalibrierungen tauscht.
- **eigene Pins.** `sync_pin` ist optional; ohne Pin bleiben die W1TS/W1TC-Masken
  0, und der Tick schreibt No-Ops ohne Verzweigung.
- **eigener Recorder** und `/servo/<id>/dump.csv`; `/servo/dump.csv` bleibt Alias
  auf Index 0 (servo-hil-Tools).
- **LEDC:** ein Timer (der letzte, Timer 3) für alle Servo-Kanäle; die Kanäle werden
  von oben vergeben (7/6, 5/4, …). Befund: ESPHome-`ledc` vergibt ab Kanal 0 mit
  Timer = Kanal/2 und setzt das LEDC-Peripheral im eigenen `setup()` zurück. Das
  läuft mit Priorität HARDWARE vor unserem AFTER_WIFI, also unkritisch.
  FINAL_VALIDATE lehnt Überschneidungen ab.

## Arbiter (R1, K6)

- **Invariante:** Nur `owner_` darf Duty ≠ 0 anlegen. Ein Nicht-Besitzer, dessen
  Core trotzdem Duty liefert, wird hart auf 0 gezwungen und in
  `arbiter_blocked` gezählt. Das ist ein Backstop; aus IDLE kann der Core das heute
  nicht.
- **Ablauf:** `command_target()` / `start_ident()` im Main-Task latchen das Ziel
  in Atomics und schieben den Slot in einen SPSC-Ring (8 Einträge). Der Tick
  leert den Ring einmal pro Tick, **bevor** irgendeine Instanz läuft:
  1. Abbruch/Release verarbeiten.
  2. Ring leeren. Ist der Besitzer selbst gemeint und wartet niemand, wird er
     direkt neu angesteuert (Retarget). Andernfalls kommt die Anfrage in
     Eintreffreihenfolge in eine Tick-eigene FIFO. Wartet die Instanz schon,
     gilt für sie der neueste Sollwert (Setpoint-Semantik); kein Slider geht
     verloren, keiner verhungert.
  3. Ist der Besitzer IDLE und kein Ident aktiv, gibt er den Grant zurück.
  4. Der älteste Wartende bekommt den Grant (Resync, dann Ausführung).
- **Überlauf** (Ring voll): Das Kommando wird abgewiesen. Status `queue voll,
  befehl abgewiesen` und Zähler `get_queue_overflows()`; nichts wird still
  verworfen.
- **Abbruch** (`abort_ident()`): entfernt die Instanz aus der FIFO, macht alle bis
  dahin gelatchten Kommandos ungültig, bricht ein Ident ab und lässt den Core los.
  Das Drehmoment ist im nächsten Tick 0. `command_release()` (Rig-Button) behält
  die Rig-Semantik: Ein späteres Ziel läuft trotzdem.

## Billiger IDLE-Pfad

`idle_path: auto` heißt: `full` bei einer Instanz (Rig, regelungsidentisch zu
vorher), `cheap` bei mehreren. Im billigen Pfad wird nur abgetastet
(`idle_sample_divider` 1…10, also 1 kHz bis 100 Hz), dazu kommen Position,
Duty 0 und Sync LOW. Observer und Controller laufen nicht. Beim nächsten Grant
setzt `core_resync_(z)` den Core neu auf die aktuelle Position.

Heute erledigt das `Servo::init()`. Das setzt die L1-Thermal-EMA und die
Diagnose-Logs zurück. Deshalb wechselt eine Instanz erst in den billigen Pfad,
wenn ihre EMA unter **0,05** liegt; bis dahin rechnet der volle Pfad die EMA
exakt ab. Der Fehler im Thermalbudget ist damit auf ≤ 0,05 begrenzt (bei
tau 120 s entspricht das rund 6 s Vollgas). Wunsch an den Core: siehe unten.

Für den Beleg auf Hardware laufen je Instanz mit:

- Histogramm des billigen Pfads (10-µs-Buckets): `get_idle_path_p99_us()`
- Maximum `get_idle_path_max_us()`
- Maximum des vollen Pfads `get_full_path_max_us()`
- Tick-Zähler je Pfad

Hub-weit: Tick-p99 (25-µs-Buckets), verpasste Ticks, ADC-Contention.

## Locking-Modell (R2)

Der S2 hat einen Kern; der Tick-Task läuft mit Priorität 24, darüber liegt nichts,
was diese Strukturen anfasst.

- **Der Tick wartet nie.** Er nimmt keinen Mutex, alles Geteilte ist
  `std::atomic` oder der lock-freie SPSC-Ring.
- **Kommandos:** Erzeuger im Main-Task serialisieren sich nur untereinander
  (`producer_mtx_`). Der Tick ist alleiniger Verbraucher.
- **Geteilter ADC1:** Die Wiper liest der Tick, die Extra-Kanäle der Main-Task über
  `try_read_shared()`. Der Main-Task startet eine Konversion nur, wenn bis zum
  nächsten Tick noch ≥ 250 µs Luft sind, und klammert sie mit `adc_busy_`. Landet
  ein Tick trotzdem in der Klammer (z. B. weil WiFi den Main-Task verdrängt hat),
  sieht er das Flag beim Eintritt. Er hält dann die letzten Wiper-Samples
  (`adc_contention`) und **wartet nicht**. Der Main-Read wiederholt im nächsten
  Loop-Durchlauf.
- **Befund:** IDF `adc_oneshot_read` nimmt nur ein Try-Lock (`ESP_ERR_TIMEOUT` bei
  Kollision) und wandelt in einer Critical Section. Ein unkoordinierter
  Fremdzugriff hätte Tick-Samples also still als ADC-Fehler verloren.
- Auf Dual-Core-Chips (nicht unser Ziel) bliebe der Tick wartefrei; Kollisionen
  zählten dann als ADC-Fehler.

## R5 / K1 — Boot bewegt nie

- `command_target()` und `start_ident()` lehnen alles vor
  `App.is_setup_complete()` ab. Status: `befehl vor boot-ende verworfen`.
- Der Boot-Zustand ist IDLE mit Drehmoment 0 und Hold = gemessene Position.
- Die Soll-Numbers sind Template-Numbers mit `lambda` (Anzeige = Hold/Ziel des
  Servos). ESPHome lehnt `restore_value` mit `lambda` grundsätzlich ab, ein
  Restore ist damit strukturell ausgeschlossen. Es gibt weder `initial_value`
  noch `on_value`.
- OTA `on_begin` ruft `abort_ident()` beider Slider, damit während des Flashens
  Drehmoment 0 anliegt.
- API-Reconnect sendet keine Number-Kommandos, deshalb gibt es keinen eigenen
  Codepfad dafür.

## YAML-Mapping Küche (R2/R3)

Versorgung, gleiche ID, gleicher Name, gleiche Filter:

```yaml
sensor:
  - platform: slider_servo
    type: adc
    pin: GPIO1
    attenuation: 12db
    id: "voltage_supply"
    name: "Küche Mief 24V Versorgung"
    update_interval: 0.9s
    device_class: "voltage"
    state_class: "measurement"
    filters:
      - multiply: 11.5
      - or: [ {delta: 0.5}, {throttle: 600sec} ]
```

**Stufe 1 bis 4, werte-identisch zu heute:** `position_slider_1/2` bekommen
`type: wiper_voltage`. Das ist dieselbe physikalische Größe wie
`platform: adc` (Volt, 12 dB, IDF-Kalibrierung). ID, Name, Einheit,
device_class und die **komplette Legacy-Filterkette** bleiben 1:1. Siehe
`firmware/kueche_mief_slider_compile.yaml`:

```yaml
  - platform: slider_servo
    type: wiper_voltage
    slider_servo_id: licht
    id: position_slider_1
    name: "Küche Licht UI"
    # ... state_class/device_class/unit/update_interval/filters exakt wie heute
```

**Stufe 5, nach dem Ident und Jans OK auf den Diff:** Umstellung auf die
kalibrierte Servo-Position. Die `calibrate_linear`-Punkte entfallen (R3), die
Glättung bleibt:

```yaml
  - platform: slider_servo
    type: position
    slider_servo_id: licht
    id: position_slider_1
    name: "Küche Licht UI"
    state_class: "measurement"
    device_class: power_factor
    unit_of_measurement: "%"
    update_interval: 32ms
    filters:
      - sliding_window_moving_average: {window_size: 6, send_every: 4}
      - clamp: {min_value: 0, max_value: 100}
      - or: [ {delta: 0.5}, {throttle: 10sec} ]
```

Migrationshinweise für Stufe 5:

- Die Küche-Config hat heute **kein `psram:`**. Für die Recorder-Ringe muss es
  ergänzt werden; ohne PSRAM fällt jeder Ring auf 96 KB internes RAM zurück.
- `motor_1_r/l` (gpio) und `motor_2_r/l` (ledc) entfallen, die Pins gehören dem Servo.
- Die Rohwerte der Endpunkte in der Compile-Config sind **grobe Umrechnungen**
  der Legacy-Volt; Ident I3 ersetzt sie.

## Was offline NICHT belegbar ist

| Punkt | Warum | Beleg wann/wie |
|---|---|---|
| **K10**: Tick-p99 ≤ 900 µs, ≤ 1 verpasst/250 k mit beiden Instanzen + Küche-Last + Ident | Laufzeit auf Hardware unter echter WiFi-/I2C-/UART-Last | Stufe 1 (ohne Motor) und Stufe 3/4: `Slider Tick p99`, `Slider Ticks verpasst` |
| IDLE-Pfad ≤ 100 µs | `adc_oneshot_read` × 4 kostet auf dem S2 eine unbekannte Zahl µs (mit Kalibrier-Init pro Read) | `… Idle-Pfad p99`; sonst `idle_sample_divider` hoch (≥ 100 Hz) |
| Kollisionsrate geteilter ADC | hängt am WiFi-Preemption-Muster des Main-Tasks | `get_adc_contention()` (Erwartung: selten; jede Kollision = 1 Tick Sample-Hold) |
| `wiper_voltage` ≡ `platform: adc` ±1 % | ESPHome-adc mittelt ggf. anders; Kalibrierung: das S2-eFuse-Schema, Linear-Fallback geschätzt | Stufe 1: Vergleich gegen `sensor.kuche_licht_ui` |
| LEDC 7…4 auf Timer 3 neben ESPHome-ledc ch0 | nur statisch validiert | Stufe 2: Duty-Mapping beobachten |
| Socket-Budget httpd + api + ota + udp | lwIP-Grenzen zur Laufzeit | Stufe 1: Dump ziehen, während HA verbunden ist |
| K1 bei echtem Reconnect/OTA | strukturell abgesichert, nicht gemessen | Stufe 4 (Reboot-/Reconnect-Test) |

## Wünsche an den Core-Agenten

1. **`Servo::resync(float z)`**: Observer neu setzen (p = z, v = 0, P, stuck),
   Controller IDLE mit hold = z, dabei **L1-EMA und ControllerLog
   behalten**. Ersetzt das `init()` in `core_resync_()` und damit die
   0,05-Schwelle.
2. **Ident-API**, wie sie der CORE-ADAPTER erwartet:
   - `bool start_ident()`, `void abort_ident()`
   - `bool ident_active() const` (hält den Grant)
   - `const char *ident_phase_name() const` (Status `ident(<phase>)`)
   - Zugriff auf den Bericht (Text bzw. JSON für `slider_ident_report`, §6)
3. **NVS-Commit aus dem Main-Task:** I9 darf nicht im Tick schreiben (§4 #10,
   Flash). Vorschlag: `bool calib_commit_pending()`, dazu die zu speichernde
   `UnitCalib` und ein Quittungs-Aufruf. Der Wrapper speichert in `loop()`
   (Blob, dann Commit-Zähler, atomar).
4. **R8-Flag:** Der Core braucht `nicht identifiziert` als Eingang, z. B.
   `set_identified(bool)` oder ein Feld in UnitCalib, für den gedrosselten Modus.
   Der Wrapper liefert `calib_valid_`.
5. **Stempel-Layout (§5):** Der Core definiert Struct und Prüfregel (fw-Version,
   Instanz-Index, Endpunkt-ID, VM-Bucket, Commit-Zähler), der Wrapper macht nur
   die NVS-I/O. Danach wird `kCalibVersion` erhöht.
6. **Versorgungsspannung für den Preflight** (§3 Zeilen 3 und 30): Eingang
   `set_supply_volts(float)`. Der Wrapper speist ihn aus dem Extra-Kanal per
   Atomic im Tick.
7. Optional `Controller::has_pending()`, damit der Status einen gespeicherten
   Befehl in RELEASED sichtbar machen kann.

## Offene Punkte / bewusst entschieden

- **Laufzeitverhalten der Rig ändert sich**, auch wenn ihre YAML unverändert ist.
  Beim nächsten Rig-Flash ist ein HIL-Kurzcheck fällig:
  - LEDC-Kanäle 0/1 → 7/6, Timer 0 → 3
  - httpd `max_open_sockets` 7 → 3, `ctrl_port` 32768 → `http_port + 1`
    (vermeidet eine Kollision mit einem späteren ESPHome-web_server)
  - Thermal 10 s / 0,50 / 0,35 → **120 s / 0,95 / 0,60** (Jans Vorgabe)
- **Neue Diagnose-Entitäten** in der Compile-Config als **Vorschlag** (nicht in
  R4, Jan bestätigt): `Küche Licht UI Thermal`, `Dunstabzug Slider Thermal`,
  `… Idle-Pfad p99`. Die kalibrierten Positionen sind `internal: true`.
- Der Status mappt GLIDE auf `crawl` (die R4-Liste kennt kein glide).
- `Ident Bericht` ist ein Stub-Text, der Service `slider_ident_report` (§6) fehlt
  noch und kommt mit dem Core-Merge.
- Der Recorder schreibt nur Ticks des vollen Pfads (Lücken in `t_ms` sind
  ehrlich).
- Die Polarität in der Compile-Config ist ein Startwert (R6):
  `motsliddf pwm_forward = motor_1_l`, also AIN1 = GPIO11 bzw. analog GPIO16.

## Commits

| Hash | Inhalt |
|---|---|
| `dbdba06` | Hub, Arbiter, ADC1-Besitz, optionaler Sync-Pin, NVS je Index, Dump-URIs, Thermal-Schema |
| `f42b039` | Sensor-Plattform: position / wiper_voltage / adc |
| `aca2a9f` | `kueche_mief_slider_compile.yaml`, Negativ-Kontrollen |
