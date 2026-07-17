# Day-Session Handover — 2026-07-17 (~08:00 → 16:00, Jan 8 h AFK)

Anschluss an die Nacht-Session (docs/handover-night-2026-07-16.md,
Summary docs/findings/night-2026-07-17.md). **Niemand an der Bench.**
Alles Blockierende ist morgens erledigt: Phase-1-Gate ZU (Alignment 68 ppm),
Co-Swing verifiziert, Scope läuft (Ch-B-Trigger; EXT ist tot), Rig auf fester
IP .115, Endzonen-Cut ausgebaut (passiver Marker, goto kommt überall raus).

## Mission

Das fehlende Stück des Plant-Modells messen und einbauen (**M3 mit echtem
Strom → K und R**), M2-Spike-Arbeit für die RC-Filter-Entscheidung, die
Dwell-Abhängigkeit der Impulstabelle quantifizieren, und **Phase 3 abschließen**
(Controller-Prototyp im Sim gegen die Akzeptanzmetriken). Stretch:
controller-spec.md-Entwurf (Phase 4) zur Freigabe, wenn Jan zurück ist.

## Zustand bei Übergabe

- Rig: slider-lab @ **192.168.178.115 (fest)**, Firmware v6 (Endzonen-Cut
  passiv, goto_duty-Param, Dump-Fix, Clamp 0.5), Player idle ~50 %, gesund.
- Scope: 3203D verkabelt und verifiziert — Ch A Wiper (Ratio 1.20), Ch B Shunt
  (0.2 Ω GND-Return, Mittel ±0.2..0.5 V, Spikes ±2 V), **Trigger nur Ch B**
  (EXT-Pfad tot, Bench-Diagnose wartet auf Jan). Single Owner!
- Interlocks: Budget 30 s, Thermal ≤0.5 avg/10 s, Abort — **kein Endzonen-Cut
  mehr**; Positionen dürfen jetzt bis an die Anschläge (FLAG_ENDZONE = nur Daten).
- Agenten dieser Session sind fortsetzbar: campaign-runner (Blöcke 0–4 + Craft),
  modeler (2 Pässe, kennt Pipeline), coder (Zyklus 1), demo (PASS).
- Wiedervorlagen aus dem Morgen: ADC-Aussetzer ungeklärt (raw flach 0, nur
  OTA-Reboot half — falls wieder: status + Hand-Move-Test unmöglich ohne Jan →
  OTA-Reboot versuchen, dokumentieren), Tick ignoriert ADC-Fehlercodes,
  labctl-Endzonen-Wortlaut veraltet.

## Querschnittsauftrag (Jan, beim Rausgehen): Positionsabhängigkeit prüfen

> „die Eigenschaften des Sliders sind ev. abhängig von der Position"

Die Nacht bestätigt das für Breakaway (±0.04-Spread, rev-Asymmetrie) und Sustain
(55–80-%-rev-Band). Noch NICHT positionsaufgelöst: **Impulstabelle** (nur
mid-travel — aber sie ist das Endgame-Werkzeug!), **Fein-Klippe** (nur 21 % fwd /
80 % rev), und das neue **M3/I(duty)**. Konsequenz für jeden Block unten: Position
ist Messachse, nicht Konstante — M3-Staircases an ≥3 Positionen (inkl. 65 % rev),
Impuls-Grid an ≥3 Positionen in der Dwell-Kampagne mitfahren, und der modeler
weist für JEDEN Parameter aus, ob er global ist oder positionsabhängig (mit
Zahlen, nicht Bauchgefühl). K/R selbst sollten positionsUNabhängig sein (Motor
ist Motor) — wenn die Stall-I(duty)-Kurve doch mit der Position variiert, ist
das ein Befund über den Antriebsstrang und gehört groß ins Findings-Doc.

## Blöcke (Reihenfolge; hardware-seriell beim campaign-runner)

1. **Coder-Zyklus 2** (klein, zuerst — die Kampagne profitiert):
   (a) adc_oneshot_read-Fehler zählen + loggen (Fehlerzähler als Sensor/Statuszeile,
   KEINE Arbeit im Tick-Pfad außer Zähler-Inkrement); (b) labctl: Endzonen-WARN-
   Wortlaut → „endzone marker (passiv)"; (c) reversal-Approach-Dauer als Param
   (600 ms hardcoded kann bei realem Breakaway nicht kriechen); (d) breakaway-
   Default move_threshold → 5.0 (Bench-Rezept). Kompilieren, flashen, cycle,
   verifizieren.
2. **M3 echt** (Kern des Tages): Staircase-Läufe beide Modi, mehrere Positionen,
   Rungs sub-Breakaway bis Fly; Scope Ch B Mittelwerte je Rung (±0.5 V) →
   I(duty)-Kennlinie; Stall-Rungs → **R** (V=duty·VM, I gemessen); bewegte
   Rungs → Back-EMF → **K**; VM = 5.00 V notieren. Zusätzlich 2–3 Captures
   @ 10 MS/s an PWM-Flanken (M2-Spikes, ±2 V) für die RC-Filter-Entscheidung.
3. **Dwell-Kampagne** (verteilt über den Tag, billig): Impuls-Paare (10/30 ms,
   beide Richtungen) nach Dwell 2 / 10 / 30 / 60+ min — zwischen den Blöcken
   liegen lassen, Orchestrator taktet. Ziel: Warm-vs-Kalt-Faktor für die
   Impulstabelle (Demo lief bei 12–15 s Dwell ×0.76–1.17 — was passiert bei 30 min?).
4. **Modeler-Pass 3**: I(duty) → K, R mit CI; Motor- von Reibungsparametern
   trennen (Kraft endlich in physikalischen Einheiten); Spike/RC-Empfehlung
   final; Dwell-Modell (Stiction vs Ruhezeit); plant-model.md + plant-params.json
   + auto-ident-spec.md aktualisieren (was davon kann On-Device ohne Scope? —
   R evtl. via bekanntem K später, ehrlich ausweisen).
5. **Phase-3-Abschluss** (modeler, offline): Controller-Prototyp im Sim —
   Kaskade FF+PID auf dem Observer, Impuls-Endgame, unidirektionale Anfahrt,
   Override-Handling; gegen Akzeptanzmetriken A1–A8 im Sim fahren (A9 ist
   Firmware). Ehrlich: welche Metrik der Sim NICHT belastbar prüfen kann.
6. **STRETCH**: docs/controller-spec.md-ENTWURF (Phase 4): Layer-Architektur,
   Trigger-Graph als Tabelle, Invarianten als Tripwires, Forbidden-List.
   Klar als DRAFT markiert — **Phase-4-Gate bleibt Jans Freigabe**, nichts
   davon wird implementiert.

## Leitplanken

- Flash NUR slider-lab (Allow-Rules bestehen). Kein anderes Gerät, kein
  EsphomeZoo-Touch. Auf .187 wohnt ein FREMDES Gerät — Finger weg.
- Scope belegt/Fehler → CSV-only weiter; Rig tot und OTA-Reboot hilft nicht →
  Kampagne stoppen, mit vorhandenen Daten weiterarbeiten, NICHT stundenlang retryn.
- 30 s Pausen zwischen Aktuations-Chunks (Motorwärme, L9110 ~800 mA cont.);
  Anschlag-Kontakte sind erlaubt, aber kein absichtliches Dauergrinding.
- Commits klein und oft beim Orchestrator (atomar, deutsch, Befund im Text);
  kein Push.
- API-Overload (529): Backoff 10 min, Agent per SendMessage fortsetzen.
- Abends: docs/findings/day-2026-07-17.md (TL;DR: K/R? RC-Entscheid?
  Dwell-Gesetz? Controller-Sim-Metriken? + offene Fragen), Tracker aktuell,
  Rig idle, Torque 0, alle Loops gestoppt.

## Für Jan am Abend (braucht dich)

- Ohr-Checks: Dither, Reversal-Thunk, Impuls-Klackern.
- EXT-Trigger-Diagnose (Kabel/SMB vs Treiber).
- M1b (AWG/INL) — Umverkabelung.
- controller-spec.md-Freigabe (Phase-4-Gate), falls der Entwurf steht.
- Plan-Frage: Force-Feedback v1 oder v1.1?
