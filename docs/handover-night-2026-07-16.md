# Night-Session Handover — 2026-07-16 → 17

> Rekonstruiert aus `EsphomeZoo/docs/handover-night-2026-07-16.md` (Original beim
> Paste rechts abgeschnitten; Lücken aus Git-Historie und Bench-Verifikation
> ergänzt, Stand 2026-07-16 spät abends).

Mission, in Jans Worten: „get M0 to whatever — so in the morning we have a model,
an idea, or just a working slider." Autonome Nachtschicht: Messkampagne so weit
wie die Datenqualität trägt, dann Phase-3-Modellierung, Stretch =
modellbasierte Open-Loop-Demo. Jan schläft — **niemand ist an der Bench**.

## Nordstern-Ergänzung (Jan, 2026-07-16 abends — in plan.md nachgezogen)

> „the ladder has one more step: get the parameters for model and control **on
> device**. each slider is different. (i have more and different)"

Konsequenz für JEDE Fit-Entscheidung dieser Nacht: bevorzuge Identifikations-
verfahren, die ein Gerät ohne Scope, ohne Host, in begrenzter Zeit selbst
ausführen kann (Breakaway-Rampe, Impuls-Leiter, Step-Fits aus eigenem Recorder).
Scope-gebundene Messungen (Spikes, Coupling, INL) sind Design-Time-Wissen —
die Produktions-Auto-Ident darf sie nicht brauchen. Deliverable:
`docs/auto-ident-spec.md` (Draft): pro Parameter → Bench-Methode vs.
On-Device-Methode vs. Ablage (NVS, pro Exemplar).

## Agenten-Struktur (Orchestrator lädt maximal ab)

| Agent | Lebensdauer | Besitzt | Meldet |
|---|---|---|---|
| Orchestrator (/loop-Heartbeat) | ganze Nacht | Sequenzierung, Phasen-Gates, Commits, Eskalation | Morgen-Summary |
| campaign-runner | persistent, per Block fortgesetzt | labctl-Runs, Qualitäts-Gates, notes.md, Transient-Retries | 1 Absatz + Dateiliste je Block |
| coder | persistent | Firmware-Fixes (goto-Duty, Dump-Latch, Clamp), flasht selbst | Diff + Cycle-Verdict |
| modeler | persistent | Phase 3 offline: Fits+Kovarianz, Simulator, Observer, Plots, plant-model.md, auto-ident-spec.md | Fit-Zahlen + Validierungs-Overlays |
| demo agent | nur wenn Modell validiert | Open-Loop-Move + Recording | Demo oder Begründung |

Pipeline statt Staffellauf: campaign-runner ist hardware-seriell; modeler ist
offline und unblockierbar — sobald M3-Daten liegen, fittet der modeler K/R,
während die Kampagne M5/M6 fährt. Agenten reden NICHT miteinander; Austausch nur
über das Repo (data/, docs/findings/). Orchestrator relayt nur Gate-Entscheidungen.
Completion-Notifications wecken event-getrieben; der /loop-Wakeup ist
Fallback-Heartbeat (überbrückt auch Classifier-Outages per Retry beim nächsten
Wakeup). Commits bleiben beim Orchestrator (atomar, deutsch, Befund im Text).
*(Anmerkung Nacht-Session: die „bestehenden" Firmware-/Analysis-Agenten der
Vorsession sind nicht mehr erreichbar — coder und modeler starten frisch mit
ausführlichem Briefing.)*

## Zustand bei Übergabe

- **Rig**: slider-lab (S2 mini, 192.168.178.115, mDNS langsam — IP nutzen),
  Firmware v3+safe_mode GEFLASHT und verifiziert. Interlocks: Positionsfenster
  [3,97] außer `endstop_safe`, 30 s Budget, Thermal ≤0.5 avg/10 s, Abort-Switch.
  PSU-Kette 12 V → DCDC 5 V → Linear 3.3 V, VM = 5 V („rock stable", gesetzt).
- **H-Bridge = L9110** (Jans Korrektur — NICHT DRV8833): kein
  Coast-Zustand, kein Strom-Limit. Firmware-„FAST" = brake-low (slow decay!),
  „SLOW" = brake-high. Doku-Korrektur in lab-rig-spec R3 + probe-connections +
  plan.md: erledigt zu Beginn der Nacht-Session.
- **Shunt**: 0.2 Ω in der GND-Rückleitung des L9110-Moduls (GND—Shunt—GND-Brücke),
  führt den GESAMTEN Brückenstrom, single-ended. Jan sah live Spikes >500 mV →
  Messbereich ±2 V/±5 V für Spike-Arbeit, ±0.2..0.5 V für gemittelte Levels.
- **Probes** (von Jan neu gesetzt, UNVERIFIZIERT): Ch A = Wiper (ADC-Knoten),
  Ch B = Shunt, EXT = GPIO14 Sync. Frühere Session: Ch A zeigte Fremdsignal
  (vermutlich zweite Pot-Spur, nur ~1/50 Swing) — deshalb ZUERST Co-Swing-Check.
- **PicoScope 3203D**: single owner! PicoScope-7-App muss zu sein (ist zu).
  pico-bench x86-venv: `arch -x86_64 /Users/jan/dev/repos/pico-bench/.venv-x86/bin/python -u`.
  EXT-Trigger-Pfad im Treiber HW-unvalidiert (feuerte vor Ground-Fix nie, danach
  ungetestet). Sicherer Fallback: Trigger auf Ch B (Shunt-Ripple).
- **Breakaway-Bracket (erster Plant-Datenpunkt)**: 0.55 kriecht (0.2 %/4 s),
  0.7 fliegt. Legacy 0.42 ist FALSCH. Firmware-goto fährt 0.55 → **goto unter
  Breakaway nutzlos** — siehe Aufgabe 2.
- **labctl** (gebaut, live-verifiziert, Tests grün, committed in 33218f5):
  `cd /Users/jan/dev/repos/motorslider/analysis && uv run labctl …`
  - `status` / `preflight [--scope] [--no-shunt]` (4.3 s) / `flash` (23–34 s,
    exit 75 = Flash-Handoff nötig) / `run <exp> --params JSON [--scope PRESET]`
    (16–18 s inkl. VERDICT-Zeile) / `cycle`.
  - Firmware-Programme: noise_floor, pwm_coupling(≤0.35!), duty_ladder,
    breakaway_grid, step_response(goto+hold), triangle, reversal,
    impulse_ladder, dither_grid; Params-Flag `endstop_safe`; „dither"-Segment
    amp:0/bias:±0.7 = sauberer 100-ms-Blip (Preflight-Actuation nutzt das).
- **Bekannte Firmware-Bugs/Quirks**: (1) abgebrochener Dump-Download latcht
  `dump_active_` → run_experiment verweigert still bis Socket stirbt; labctl
  konsumiert Bodies + hat Refusal-Detektoren; echter Fix gehört in die Firmware.
  (2) goto-Duty 0.55 zu schwach. (3) run_experiment-Rückgabewert wird von der
  API verworfen — Refusals nur via labctl-Detektor sichtbar.
- **Flash-Autonomie**: Allow-Rules + autoMode-Klausel in
  `EsphomeZoo/.claude/settings.local.json` — NUR slider_lab_rig.yaml. Subagent
  hat damit erfolgreich selbst geflasht. Alle anderen Geräte: explizite Freigabe.
- **Classifier-Outage-Playbook**: allow-gelistete Kommandos laufen weiter;
  gated Arbeit in docs/PENDING.md parken; read-only weiterarbeiten; beim
  nächsten Wakeup retryn.
- **Motor-Dauerschleife** der Vorsession: GESTOPPT (Player idle, verifiziert).
- Housekeeping-Basis: Jans Commit `33218f5` („stage0 ready to go") enthält
  labctl + Tests + yaml + data/2026-07-16/03..14.

## Erste Aufgaben (Reihenfolge fix)

1. **Housekeeping**: L9110-Doku-Fixes (lab-rig-spec R3, probe-connections,
   measurement-campaign M3-Note), plan.md „On-Device-Auto-Ident" + Tracker,
   Handover-Rekonstruktion (dieses Dokument), Settings-Batch (Allow-Rules:
   motorslider-git, uv run, pico-bench-python, curl auf Rig-IP) für
   Classifier-Outage-Resilienz. Commits atomar.
2. **Coder-Zyklus 1** (ein labctl cycle): goto-Duty 0.55 → 0.72 (oder Param),
   Dump-Session-Timeout (Latch-Fix), pwm_coupling-Clamp → 0.5 (M2 braucht
   stallnahe Ströme unter neuem Breakaway). Kompilieren, flashen, verifizieren,
   committen.
3. **Preflight + Alignment-Gate**: `labctl preflight --scope` (Co-Swing Ch A!).
   Dann EIN `labctl run pwm_coupling --params '{"duty":0.35,...}' --scope
   alignment` → Strom-Step (Scope) vs. duty-Step (CSV) = Phase-1-Gate ZU.
   Falls Ch A wieder ~1/50-Swing: Probe falsch → Scope-Checks degradieren,
   Kampagne CSV-only, morgens Jan zeigen.

## Kampagne (Nacht-Zuschnitt: KEIN Umclippen möglich)

Mit A=Wiper/B=Shunt gehen: M1 (ohne 3V3-Vergleich), M2 (Trigger auf
Shunt-Ripple statt PWM-Probe), M3 (Kern der Nacht: beide „Modi",
Staircase-Mittel je Rung → I(duty), Motor-R, Torque-K), M5 (Steps +
triangle), M6 (Strom-Signatur aus GND-Return-Shunt gültig), M4/M7 (NACH
goto-Fix; Grid-Chunks, Positionen ≤95). M8 nur objektiver Teil
(Breakaway-Reduktion); Hörbarkeit → Jan morgens. M1b (AWG/INL) entfällt;
WiFi-off-Vergleich entfällt. Daten → data/<datum>/ + notes.md; VM = 5.00 V.

**Datenqualitäts-Gate vor jedem Block**: tickhealth PASS (strukturell 0,
Jitter ≤ T/4). Rig tot (Reboot braucht Jan): Kampagne stoppen, mit vorhandenen
Daten in Phase 3 — NICHT stundenlang retryn.

## Phase 3 (ab genug Daten; offline, unblockierbar)

1. Fits: I(duty)→K,R; Steps→m,b,F_c mit Kovarianz (Degeneration EHRLICH
   ausweisen); Stribeck-Map aus breakaway_grid; Backlash aus M6;
   Impuls-Tabelle aus impulse_ladder.
2. Plant-Simulator (Stribeck+Backlash+Trägheit+ADC-Modell) → Validierung:
   Overlay Sim vs. echte Runs (Plots nach docs/findings/).
3. Observer-Prototyp (2-State-KF mit Torque-Input) auf ECHTEN Recordings;
   Velocity-Güte vs. Offline-Zero-Phase-Ableitung.
4. docs/plant-model.md + docs/auto-ident-spec.md (Draft).
5. STRETCH „working slider": modellbasiertes Open-Loop-Torque-Profil für einen
   glatten Move (FF aus identifizierten Parametern), als Segmentfolge durchs
   bestehende Player-API (KEIN neuer Controller in Firmware!). Akzeptanz:
   glatter Positionsverlauf ohne Mid-Travel-Stall, Landung ±3 %, Recording
   als Beweis committed. Klappt das, ist „90 % Feedforward" bewiesen,
   bevor eine einzige Regler-Zeile existiert.

## Leitplanken der Nacht

- Flash NUR slider-lab. Kein anderes Gerät, kein EsphomeZoo-Touch.
- Interlocks bleiben; keine endstop_safe-Dauerläufe; 30 s Pause zwischen
  Grid-Chunks (Motorwärme, L9110 ~800 mA cont.).
- Scope belegt/Fehler → CSV-only weiter; Scope ist Bonus, nicht Blocker.
- Commits klein und oft; kein Push (kein Remote).
- Morgens: docs/findings/night-2026-07-17.md (TL;DR zuerst: Modell? Idee?
  working slider? + Zahlen + offene Fragen), Tracker aktualisiert,
  ALLE Loops gestoppt, Torque 0, Player idle.
