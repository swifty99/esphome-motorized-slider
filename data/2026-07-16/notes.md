- 2026-07-16 18:27 `01_noise_floor.csv`: gate run: quiet noise_floor (no API clients during recording), fw with integer-formatter dump fix, USB-powered from Mac, no scope
- 2026-07-16 18:41 `02_noise_floor.csv`: v3 gate evidence (GPTimer + tick task prio 24): 5 quiet runs, 52490 intervals, 0 structural missed/dup, max jitter 115us, tick_us p99 418us
- 2026-07-16 19:42 `03_pwm_coupling.csv`: session 1: pwm_coupling 0.2 fwd fast for alignment proof, knob ~38.5%, rig on PSU
- 2026-07-16 19:55 `05_pwm_coupling.csv`: alignment run captured on scope as 04_alignment_scope.npz (trigger B shunt 25mV rising)
- 2026-07-16 20:10 `07_triangle.csv`: stall-verification triangle run, scope saw flat shunt
- 2026-07-16 20:37 `09_step_response.csv`: shunt check run: scope 08, wiper twitch only
- 2026-07-16 20:43 `11_noise_floor.csv`: preflight actuation blip
- 2026-07-16 20:48 `12_dither.csv`: labctl: endstop_safe nudge out of the end zone before preflight
- 2026-07-16 20:48 `13_dither.csv`: preflight actuation blip
- 2026-07-16 21:54 `15_dither.csv`: preflight actuation blip
- 2026-07-16 22:36 `17_dither.csv`: preflight actuation blip
- 2026-07-16 22:43 `23_dither.csv`: coder: endstop_safe nudge out of the end zone before pwm_coupling verification
- 2026-07-16 22:44 `24_reversal.csv`: coder: reposition to mid-travel (also 2nd goto direction check)
- 2026-07-16 22:45 `25_dither.csv`: coder: -0.85 blip to unstick from high-end zone
- 2026-07-16 22:45 `26_dither.csv`: coder: endstop_safe nudge out of bottom end zone
- 2026-07-16 22:46 `27_pwm_coupling.csv`: coder verification: new 0.5 clamp, mid-travel stall check
- 2026-07-16 22:48 `29_reversal.csv`: coder: goto_duty param end-to-end check
- 2026-07-16 23:02 `30_dither.csv`: preflight actuation blip
- 2026-07-16 23:22 `31_pwm_coupling.csv`: Block 0: alignment run CSV-only (PicoScope off USB bus, openUnit 0x3, no capture possible), VM 5.00 V

## Block 0 (campaign-runner, night session, 23:00-23:25, VM = 5.00 V)
- `labctl status`: player idle, pos 51 %, no reboot — OK to start.
- `labctl preflight --scope`: PREFLIGHT FAIL, but only on the scope gate.
  - [1 connectivity] PASS (api 0.2 s, http 200, player idle)
  - [2 position] PASS 51.0 % (std 0.16 %)
  - [3 actuation] PASS: blip moved 7.11 % (peak duty 700 milli, no interlocks) [`30_dither.csv`]
  - [4 scope] FAIL: capture.py never armed — `ps3000aOpenUnit failed: 0x3` (PICO_NOT_FOUND).
- PicoScope 3203D is OFF THE USB BUS entirely (ioreg: Pico VID 0x0CE9 absent; no process holds it;
  last successful capture was `08_shunt_check_scope.npz` at 20:35). Polled re-enumeration for 3 min +
  retried openUnit at 23:21 — still gone. Needs a physical replug; nobody at the bench.
  Consequence: co-swing check (4a) and shunt check (4b) NOT verified tonight; EXT trigger still
  hardware-unvalidated; Phase-1 scope-alignment gate NOT closed.
- Alignment run executed CSV-only: `31_pwm_coupling.csv` (duty 0.35, 10 s, mid-travel 44.3 %).
  VERDICT PASS — 11099 records, duty 0->350 milli for exactly 10000 ticks, 3 segments,
  0 interlocks, position held 44.3 % -> 44.3 % (true stall), tick health green
  (0 missed/dup, jitter p99 42 us, tick_us p99 451 us). No npz — no scope.
- Campaign recommendation: NO-GO for scope in M1/M2/M3 until the scope is replugged;
  CSV-only blocks remain valid.
