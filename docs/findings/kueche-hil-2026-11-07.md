# Küche Mief — HIL 2026-11-07 (Stufen 1–4 der Küche-Mief-Spec)

Gerät `kueche-mief` (ESP32-S2 Mini, 192.168.178.57), echte Slider, kein Oszi, Beobachtung per Poti,
Dump (`/servo/<id>/dump.csv`) und HA. Jans Flash-OK galt bis 18:00 Berlin (nur dieses Gerät).
Firmware = `EsphomeZoo/kueche_mief_ESP32.yaml` (Commits `d5c8813` … `HEAD`), Komponente aus diesem Repo (`main`).

## TL;DR

- **Es geht.** Beide Slider werden per HA (`number.set_value` auf das Soll-Number) auf einen Wert gefahren und
  landen im Mittel auf ~0,1–0,15 %. Beide identifizieren sich per Knopf selbst, ohne Oszi, ~120–140 s, das
  Ergebnis überlebt Reboots (NVS).
- Landetest (12 Zufallsmoves je Slider, nach Ident): alle `reached`.

| Slider | |Fehler| median | max | ≤0,3 % | ≤2 % |
|---|---|---|---|---|
| 1 Licht UI | 0,10 % | 0,31 % | 11/12 | 12/12 |
| 2 Dunstabzug | 0,15 % | 0,33 % | 10/12 | 12/12 |

  Vor dem Ident (Nominalwerte der Rig) landete Slider 1 ~2 % vor dem Ziel mit `gave_up` — der Ident ist der Gewinn.
- Ident-Ergebnisse (Slider 1 fünfmal wiederholt: `Fs95` 0,740 jedes Mal, `Fsmin` 0,62–0,64, `raw0` 8166–8174,
  Weg bei 20 ms 2,65–2,79 %): wiederholbar innerhalb der K9-Bänder.

| | dir | raw_at_0 | raw_at_100 | Fs95 | impulse_duty | u_max |
|---|---|---|---|---|---|---|
| Slider 1 | +1 | 8174 | 193 | 0,74 | 0,84 | 0,89 |
| Slider 2 | −1 | 484 | 8191 (geklippt) | 0,67 | 0,77 | 0,82 |

## Hardware-Befunde (jeder war ein Fehler in einer Annahme)

1. **Polarität ist je Slider verschieden** (Spec R6 bestätigt): Slider 1 mit AIN1 = GPIO11 vorwärts; Slider 2
   trieb mit der geratenen Belegung in den unteren Anschlag und musste invertiert werden. Der Ident lernt `dir_sign`.
2. **Der Wiper von Slider 2 sättigt am oberen Anschlag** am ADC-Vollausschlag (raw 8191, σ = 0). Der erste Ident-Lauf
   endete `FAIL(span)`, weil die σ-Plausibilität die Schiene ablehnte. Fix im Core: D22 (geklippte Schiene gültig,
   nutzbare Spanne endet vor dem Klipp), D22b (σ0 wird nicht in der Klemme gemessen — sonst `s0 = 0` und der
   Observer vertraut z blind), D23 (Ident-Start aus dem geklippten Anschlag).
3. **Rig-Konstanten gelten nicht** (§1 der Spec): Breakaway 0,67–0,74 statt Rig-Tabelle, Impulse bringen ~0,3 %
   statt 2,6 % bei 10 ms. R7 (impulse_duty, u_max aus UnitCalib) wirkt.
4. **Versorgungs-Referenz** (D24): ein Boot-Transient (0 V) im Preflight-Mittel führte zu `ABORT(supply)`.
   Referenz ist jetzt der Median gültiger Werte, Transienten zählen als „unbekannt".
5. **ADC1 gehört `slider_servo`** (R2): die ESPHome-`adc`-Plattform ist für GPIO1/5/6 entfallen, IDs/Namen/Filter blieben,
   Werte decken sich mit vorher (Licht UI 31,59 → 31,43 %, Versorgung 22,11 → 21,99 V).
6. **Tick-Budget**: Die untätige Instanz sampelte den ADC 4× pro Tick (~300 µs). Mit `idle_sample_divider: 10`
   sinkt der Hub-Tick-p99 im Ruhezustand auf 650 µs, unter Ident 875–975 µs (K10-Gate p99 ≤ 900 µs ist unter Ident
   **knapp verfehlt**, im Ruhezustand erfüllt).
7. **Task-Watchdog-Resets** (2 von 6 Ident-Läufen vor dem Idle-Teiler): ESPHome zeigt sie als
   `IllegalInstruction` (EXCCAUSE 0), der Reset-Grund-Sensor als `task watchdog`; der PC lag beide Male im Tick-Task.
   Stack-Verbrauch ist es nicht (HWM 7488 von 8192 B frei). Hypothese: Tick nahe 100 % CPU → Main-Loop verhungert.
   Seit dem Idle-Teiler 0 Resets in 5 Läufen; weiter beobachten (Reset-Grund-Sensor).
8. **Dauer**: Ident real 118–140 s (Spec-Budget 120 s); Vorschlag P1+P2 (Bracket kürzen) liegt im Findings-Doc
   `ident-sil-2026-11-07.md` (Q10), noch nicht eingebaut.

## Was geändert wurde (alles in `main`)

Wrapper (`slider_servo.cpp/.h`, `__init__.py`, `sensor.py`): Multi-Instanz, ADC1-Besitz, Arbiter, Ident-Anschluss,
NVS-Ergebnis (`SVI1`), Tick-Stack 8192 + Stack-Diagnose. Core: Ident L6 (Rows 1–32), R7, R8, Thermal 120/0,95/0,60,
Clip D22/D22b/D23, Supply D24. SIL: 194 Tests grün (1 xfail = K9 bei realer Streuung, ausgewiesen).

## Offen

- K10 unter Ident knapp (siehe oben); Watchdog-Resets beobachten; ggf. Ident-Rechenlast senken.
- K9 (zwei Idents bei realer Streuung) im SIL `xfail`; real gut, aber nur an Slider 1 ausgiebig geprüft.
- Entity-IDs tragen das doppelte Geräte-Präfix (`..._kuche_kueche_mief_...`), kosmetisch.
- Soak 24 h, HA-Neustart-/Reconnect-Test (K1) im Betrieb; K11-Registry-Diff.
- Fragen Q2–Q10 des Core-Findings-Docs (Feinschliff, keine Blocker).
