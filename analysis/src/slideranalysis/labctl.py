"""labctl -- one-command loop stages for the motorslider bench rig.

The fast-iteration testkit: each subcommand is one stage of the
code-change -> flash -> experiment -> verdict loop, scriptable by agents
and by humans.

    labctl status                       one-shot rig state + reboot detection
    labctl preflight [--scope]          rig-sanity gates before a campaign
    labctl flash                        compile + OTA upload + boot verify, timed
    labctl run <experiment> [...]       (scope-)arm, fire, fetch, analyze, verdict
    labctl cycle [--experiment X ...]   flash + run in one command

Exit codes: 0 = OK/PASS, 1 = FAIL or error, 2 = verdict WARN,
75 = flash handoff (upload could not run from here; the exact command to
execute is printed for the orchestrator).

``labctl run`` ends with exactly one machine-readable line::

    VERDICT: PASS|WARN|FAIL <short reason>

``labctl preflight`` ends with exactly one machine-readable line::

    PREFLIGHT: PASS|FAIL <short reason>
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import queue
import shlex
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import date, datetime
from pathlib import Path

from . import tickhealth
from .fetch import DEFAULT_DATA_DIR, fetch, next_seq
from .loadrun import load_run, raw_to_percent

# ---------------------------------------------------------------------------
# labconfig -- rig + toolchain constants (single place to edit)
# ---------------------------------------------------------------------------

RIG_HOST = "192.168.178.115"  # slider-lab; mDNS slider-lab.local is SLOW, use the IP
API_PORT = 6053
API_PASSWORD = ""

ESPHOME_BIN = "/Users/jan/dev/repos/EsphomeZoo/.venv/bin/esphome"
REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_YAML = REPO_ROOT / "firmware" / "slider_lab_rig.yaml"

# PicoScope capture: MUST run under the pico-bench x86/Rosetta venv (py3.9)
X86_PYTHON = "/Users/jan/dev/repos/pico-bench/.venv-x86/bin/python"
CAPTURE_PY = REPO_ROOT / "analysis" / "scope-recipes" / "capture.py"
SCOPE_PRESETS = ("diag", "alignment", "noise", "coupling", "ladder")  # capture.py owns these

DATA_DIR = DEFAULT_DATA_DIR  # repo-root/data
STATE_FILE = DATA_DIR / ".labctl_state.json"

EXIT_OK = 0
EXIT_FAIL = 1
EXIT_WARN = 2
EXIT_FLASH_HANDOFF = 75

# Entities polled for status / run supervision (names from firmware yaml)
ENT_UPTIME = "Lab Uptime"
ENT_PLAYER = "Lab Player State"
ENT_TICK = "Lab Tick Time"
ENT_POSITION = "Slider Position"
ENT_SYNC = "Lab Sync Pin"
WANTED_ENTITIES = (ENT_UPTIME, ENT_PLAYER, ENT_TICK, ENT_POSITION, ENT_SYNC)

# ---------------------------------------------------------------------------
# experiment catalog + duration estimation (mirrors the firmware builder)
# ---------------------------------------------------------------------------

GOTO_MAX_S = 4.0        # R7: bang-bang duty 0.55, timeout 4 s (worst case = planned)
BREAKAWAY_MAX_S = 3.0   # breakaway ramp segment max_ms (self-terminating)
RECORD_TAIL_S = 0.5     # recording continues ~500 ms after the program ends

KNOWN_EXPERIMENTS = (
    "noise_floor", "pwm_coupling", "duty_ladder", "breakaway_grid",
    "step_response", "reversal", "impulse_ladder", "dither_grid",
    "triangle", "dither",
)


class LabctlError(Exception):
    """CLI-level error: message printed, exit 1."""


def validate_params(experiment: str, params_json: str | None) -> dict:
    """Validate experiment name + params JSON. Returns the params dict.

    Pure: no I/O. Raises LabctlError with a human-usable message.
    """
    if experiment not in KNOWN_EXPERIMENTS:
        raise LabctlError(
            f"unknown experiment {experiment!r}; known: {', '.join(KNOWN_EXPERIMENTS)}"
        )
    if params_json is None or params_json.strip() == "":
        return {}
    try:
        params = json.loads(params_json)
    except json.JSONDecodeError as exc:
        raise LabctlError(f"--params is not valid JSON: {exc}") from exc
    if not isinstance(params, dict):
        raise LabctlError("--params must be a JSON object, e.g. '{\"duty\":0.2}'")
    return params


def _dir_count(params: dict) -> int:
    """Number of direction passes for dir-list programs (default 'both' = 2)."""
    return 1 if params.get("dir") in ("fwd", "bwd", "rev") else 2


def estimate_duration_s(experiment: str, params: dict) -> float:
    """Planned program duration [s] incl. the 500 ms recording tail.

    Mirrors the firmware program builder (slider_lab.cpp): goto segments count
    their 4 s timeout, breakaway ramps their 3 s max -- i.e. this is the
    *planned* (worst-case) duration, same number the firmware budgets with.
    Pure: no I/O.
    """
    p = params

    def get(key, default):
        return float(p.get(key, default))

    if experiment == "noise_floor":
        t = get("duration_s", 10.0)
    elif experiment == "pwm_coupling":
        t = 0.3 + get("duration_s", 10.0) + 0.3
    elif experiment == "duty_ladder":
        d0, d1, step = get("d0", 0.05), get("d1", 0.6), get("step", 0.05)
        if step <= 0 or d1 < d0:
            raise LabctlError("duty_ladder: needs step > 0 and d1 >= d0")
        rungs = int(math.floor((d1 - d0) / step + 1e-6)) + 1
        t = rungs * (get("step_ms", 2000) + get("pause_ms", 500)) / 1000.0
    elif experiment == "breakaway_grid":
        positions = p.get("positions", [5.0, 27.5, 50.0, 72.5, 95.0])
        trials = len(positions) * int(get("repeats", 1)) * _dir_count(p)
        per = GOTO_MAX_S + 0.5 + BREAKAWAY_MAX_S + get("hold_ms", 300) / 1000.0 + 0.3
        t = trials * per
    elif experiment == "step_response":
        t = GOTO_MAX_S + 0.5 + get("hold_ms", 5000) / 1000.0 + 0.5
    elif experiment == "reversal":
        t = int(get("cycles", 3)) * (GOTO_MAX_S + 0.5 + 0.6 + 0.6 + 1.2 + 0.5)
    elif experiment == "impulse_ladder":
        widths = p.get("durations_ms", [1, 2, 3, 5, 8, 12, 20])
        repeats = int(get("repeats", 5))
        settle = get("settle_ms", 300) / 1000.0
        recenter_every = int(get("recenter_every", 5))
        cold = bool(p.get("cold_reversal", False))
        t = GOTO_MAX_S + 0.5
        shots = 0
        for _ in range(_dir_count(p)):
            for w in widths:
                for _ in range(repeats):
                    if shots > 0 and recenter_every > 0 and shots % recenter_every == 0:
                        t += GOTO_MAX_S + 0.3
                    if cold:
                        t += 0.016 + 0.2
                    t += float(w) / 1000.0 + settle
                    shots += 1
    elif experiment == "dither_grid":
        t = GOTO_MAX_S + 0.5
        if bool(p.get("baseline", True)):
            t += BREAKAWAY_MAX_S + 0.3 + 0.5 + GOTO_MAX_S + 0.5
        t += BREAKAWAY_MAX_S + 0.3 + 0.5
    elif experiment == "triangle":
        t = GOTO_MAX_S + 0.5 + 2 * get("ramp_ms", 4000) / 1000.0 + 0.5
    elif experiment == "dither":
        t = 0.3 + get("duration_s", 5.0) + 0.3
    else:  # pragma: no cover -- guarded by validate_params
        raise LabctlError(f"no duration model for {experiment!r}")
    return t + RECORD_TAIL_S


# ---------------------------------------------------------------------------
# state file + reboot detection (pure logic, thin I/O wrappers)
# ---------------------------------------------------------------------------

def detect_reboot(prev: dict | None, uptime_s: float, now_ts: float,
                  slack_s: float = 30.0) -> str | None:
    """Reboot check against the persisted state. Returns a message or None.

    A reboot happened iff the current uptime is lower than what continuous
    operation since the last contact would predict (prev uptime + elapsed),
    beyond ``slack_s`` (uptime sensor updates only every 10 s). Covers both
    'uptime lower than last seen' and 'rebooted a while ago and recovered'.
    Pure: no I/O.
    """
    if not prev:
        return None
    try:
        prev_uptime = float(prev["uptime_s"])
        prev_ts = float(prev["ts"])
    except (KeyError, TypeError, ValueError):
        return None
    elapsed = now_ts - prev_ts
    if elapsed < 0:  # clock weirdness -- no judgement
        return None
    expected = prev_uptime + elapsed
    if uptime_s + slack_s < expected:
        return (f"REBOOT detected: uptime {uptime_s:.0f} s, but ~{expected:.0f} s "
                f"expected from last contact {elapsed:.0f} s ago "
                f"(device restarted; any unfetched recording is lost)")
    return None


def load_state(path: Path = STATE_FILE) -> dict | None:
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def save_state(uptime_s: float, now_ts: float, host: str,
               path: Path = STATE_FILE) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "uptime_s": uptime_s,
        "ts": now_ts,
        "iso": datetime.fromtimestamp(now_ts).isoformat(timespec="seconds"),
        "host": host,
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# verdict assembly (pure)
# ---------------------------------------------------------------------------

@dataclass
class RunFacts:
    """Everything the verdict is assembled from."""
    experiment: str
    expected_experiment: str | None = None  # what was fired (dump must match)
    n_records: int = 0
    aborted: bool = False
    ticks_endzone: int = 0
    ticks_thermal: int = 0
    ticks_abort: int = 0
    ticks_goto: int = 0
    tick_passed: bool = True
    tick_reasons: tuple[str, ...] = ()
    scope_expected: bool = False
    scope_saved: bool = False
    player_returned_idle: bool = True
    fetched: bool = True


def assemble_verdict(f: RunFacts) -> tuple[str, str]:
    """(level, short reason) -- level is PASS / WARN / FAIL. Pure."""
    fails: list[str] = []
    warns: list[str] = []
    if not f.player_returned_idle:
        fails.append("player did not return to idle")
    if not f.fetched:
        fails.append("CSV fetch failed")
    elif f.expected_experiment and f.experiment != f.expected_experiment:
        fails.append(f"dump is from {f.experiment!r}, expected"
                     f" {f.expected_experiment!r} -- run_experiment likely"
                     " refused (busy / dump active / abort switch)")
    elif f.n_records == 0:
        fails.append("no records in dump")
    if f.aborted:
        fails.append("firmware aborted the run")
    if f.ticks_thermal:
        fails.append(f"thermal interlock on {f.ticks_thermal} ticks")
    if f.ticks_abort:
        fails.append(f"abort flag on {f.ticks_abort} ticks")
    if not f.tick_passed:
        detail = "; ".join(f.tick_reasons) or "gate failed"
        fails.append(f"tickhealth: {detail}")
    if f.ticks_endzone and f.experiment != "step_response":
        # step_response uses the end-zone cut as data (spec R4); elsewhere it
        # means the fader sat/ran outside [3, 97] -- data is suspect, not void.
        warns.append(f"endzone interlock cut torque on {f.ticks_endzone} ticks")
    if f.scope_expected and not f.scope_saved:
        warns.append("scope capture missing/failed")
    if fails:
        return "FAIL", "; ".join(fails)
    if warns:
        return "WARN", "; ".join(warns)
    extra = " (endzone cut = data)" if f.ticks_endzone else ""
    return "PASS", f"tick gates green, {f.n_records} records, no fatal interlocks{extra}"


# ---------------------------------------------------------------------------
# ESPHome native API access
# ---------------------------------------------------------------------------

class Rig:
    """Thin wrapper around aioesphomeapi for the slider-lab node."""

    def __init__(self, host: str = RIG_HOST, port: int = API_PORT):
        from aioesphomeapi import APIClient  # local import: keep pure paths import-light

        self.host = host
        self._client = APIClient(host, port, password=API_PASSWORD)
        self._names: dict = {}
        self._services: list = []
        self._states: dict[str, object] = {}
        self._collect: dict[str, list] = {}
        self._new_state: asyncio.Event = asyncio.Event()
        self.device_info = None

    async def connect(self) -> None:
        await self._client.connect(login=True)
        self.device_info = await self._client.device_info()
        entities, services = await self._client.list_entities_services()
        self._names = {e.key: e.name for e in entities if hasattr(e, "name")}
        self._services = list(services)
        self._client.subscribe_states(self._on_state)

    def _on_state(self, state) -> None:
        name = self._names.get(getattr(state, "key", None))
        if name is not None and hasattr(state, "state"):
            self._states[name] = state.state
            bucket = self._collect.get(name)
            if bucket is not None:
                bucket.append(state.state)
            self._new_state.set()

    async def collect(self, name: str, duration_s: float) -> list:
        """Collect every pushed state of entity ``name`` for ``duration_s``."""
        values: list = []
        self._collect[name] = values
        try:
            await asyncio.sleep(duration_s)
        finally:
            self._collect.pop(name, None)
        return values

    async def disconnect(self) -> None:
        try:
            await self._client.disconnect()
        except Exception:  # noqa: BLE001 -- disconnect must never mask the real error
            pass

    async def wait_until(self, name: str, predicate, timeout_s: float):
        """Wait until entity ``name`` has a state satisfying ``predicate``.

        Returns the state value, or None on timeout. Event-driven off the
        state subscription (the firmware pushes player state every 250 ms).
        """
        deadline = time.monotonic() + timeout_s
        while True:
            value = self._states.get(name)
            if value is not None and predicate(value):
                return value
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._new_state.clear()
            try:
                await asyncio.wait_for(self._new_state.wait(), min(remaining, 1.0))
            except asyncio.TimeoutError:
                pass

    async def snapshot(self, timeout_s: float = 4.0) -> dict[str, object]:
        """Collect one state for every WANTED entity (device sends all on subscribe)."""
        deadline = time.monotonic() + timeout_s
        while any(n not in self._states for n in WANTED_ENTITIES):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._new_state.clear()
            try:
                await asyncio.wait_for(self._new_state.wait(), remaining)
            except asyncio.TimeoutError:
                break
        return dict(self._states)

    async def fire(self, experiment: str, params_json: str) -> None:
        svc = next((s for s in self._services if s.name == "run_experiment"), None)
        if svc is None:
            raise LabctlError("run_experiment service not found on the device")
        result = self._client.execute_service(
            svc, {"name": experiment, "params_json": params_json})
        if asyncio.iscoroutine(result):
            await result


async def api_snapshot(host: str) -> tuple[dict[str, object], object]:
    """One-shot connect -> snapshot -> disconnect. Returns (states, device_info)."""
    rig = Rig(host)
    await rig.connect()
    try:
        states = await rig.snapshot()
    finally:
        await rig.disconnect()
    return states, rig.device_info


# ---------------------------------------------------------------------------
# scope capture subprocess (capture.py under the x86/Rosetta venv)
# ---------------------------------------------------------------------------

class ScopeCapture:
    """Arm analysis/scope-recipes/capture.py and expose its ARMED handshake.

    capture.py prints (unbuffered) an ``ARMED [...]`` line once the scope is
    waiting for its trigger, then ``>>> fire the lab experiment now <<<``.
    The experiment MUST NOT be fired before the ARMED marker was seen --
    firing early loses the trigger edge (this exact race happened on the
    bench; the handshake exists because of it).
    """

    ARMED_MARKER = "ARMED"

    def __init__(self, preset: str, out_path: Path):
        self.preset = preset
        self.out_path = Path(out_path)
        self.cmd = [
            "arch", "-x86_64", X86_PYTHON, "-u", str(CAPTURE_PY),
            "--preset", preset, "--out", str(self.out_path),
        ]
        self._proc: subprocess.Popen | None = None
        self._lines: queue.Queue[str | None] = queue.Queue()
        self._armed = threading.Event()

    def start(self) -> None:
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self._proc = subprocess.Popen(
            self.cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
        assert self._proc is not None and self._proc.stdout is not None
        for line in self._proc.stdout:
            line = line.rstrip("\n")
            print(f"  [scope] {line}", flush=True)
            if line.startswith(self.ARMED_MARKER):
                self._armed.set()
        self._lines.put(None)

    def wait_armed(self, timeout_s: float = 60.0) -> bool:
        """Block until the ARMED marker was printed. False on timeout/exit."""
        deadline = time.monotonic() + timeout_s
        while not self._armed.is_set():
            if self._proc is None or self._proc.poll() is not None:
                return self._armed.is_set()
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.1)
        return True

    def finish(self, timeout_s: float = 120.0) -> bool:
        """Wait for capture.py to exit; True iff it saved a non-empty npz."""
        if self._proc is None:
            return False
        try:
            rc = self._proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            rc = -1
        saved = (rc == 0 and self.out_path.exists()
                 and self.out_path.stat().st_size > 0)
        if not saved and self.out_path.exists() and self.out_path.stat().st_size == 0:
            self.out_path.unlink()  # drop the reserved placeholder
        return saved

    def kill(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            self._proc.kill()
        if self.out_path.exists() and self.out_path.stat().st_size == 0:
            self.out_path.unlink()


# ---------------------------------------------------------------------------
# analysis: tickhealth gate + per-run summary + scope npz summary
# ---------------------------------------------------------------------------

def summarize_run(meta: dict, df) -> tuple[list[str], RunFacts]:
    """Human summary lines + RunFacts skeleton from one loaded run."""
    import numpy as np

    experiment = str(meta.get("experiment", "?"))
    n = len(df)
    facts = RunFacts(experiment=experiment, n_records=n,
                     aborted=bool(meta.get("aborted", 0)))
    lines = [f"run: {experiment}  params: {json.dumps(meta.get('params', {}))}"]
    if n == 0:
        lines.append("  EMPTY dump (0 records)")
        return lines, facts

    facts.ticks_endzone = int(df["ilk_endzone"].sum())
    facts.ticks_thermal = int(df["ilk_thermal"].sum())
    facts.ticks_abort = int(df["abort"].sum())
    facts.ticks_goto = int(df["goto_active"].sum())

    span_s = (df["t_ms"].iloc[-1] - df["t_ms"].iloc[0]) / 1000.0
    n_segments = int(df["segment_start"].sum())
    duty = df["duty_milli"].to_numpy()
    adc = df["adc_raw"].to_numpy(dtype=float)
    k = max(1, min(200, n // 10))  # edge windows for start/end levels
    adc_start, adc_end = float(np.mean(adc[:k])), float(np.mean(adc[-k:]))

    lines += [
        f"  records: {n} ({span_s:.1f} s), segments: {n_segments}, "
        f"aborted: {'YES' if facts.aborted else 'no'}",
        f"  duty_milli: min {int(duty.min())} / max {int(duty.max())}"
        f" ({int(np.count_nonzero(duty))} nonzero ticks)",
        f"  adc_raw: start {adc_start:.0f} -> end {adc_end:.0f} counts"
        f" (range {adc.min():.0f}..{adc.max():.0f}, spread {adc.max() - adc.min():.0f});"
        f" position {raw_to_percent(adc_start):.1f} % -> {raw_to_percent(adc_end):.1f} %",
        f"  interlock ticks: endzone {facts.ticks_endzone}, thermal {facts.ticks_thermal},"
        f" abort {facts.ticks_abort}, goto {facts.ticks_goto}",
    ]
    return lines, facts


def summarize_scope(npz_path: Path, window_s: float = 0.5) -> list[str]:
    """Per-window mean/rms/peak of each channel in a capture.py npz."""
    import numpy as np

    lines: list[str] = []
    with np.load(npz_path, allow_pickle=False) as npz:
        try:
            meta = json.loads(str(npz["meta"]))
        except (KeyError, json.JSONDecodeError):
            meta = {}
        dt_s = float(npz["dt_s"])
        lines.append(
            f"scope: {npz_path.name} (preset {meta.get('preset', '?')},"
            f" {meta.get('rate_hz', 0):.0f} S/s, {meta.get('duration_s', 0):.1f} s)")
        for key in sorted(k for k in npz.files if k.startswith("volts_")):
            v = npz[key]
            lines.append(f"  {key}: per-{window_s:g}s window  mean / rms(ac) / peak [V]")
            n_win = max(1, int(round(len(v) * dt_s / window_s)))
            per = max(1, len(v) // n_win)
            for i in range(n_win):
                w = v[i * per:(i + 1) * per]
                if len(w) == 0:
                    break
                mean = float(w.mean())
                rms = float(np.sqrt(np.mean((w - mean) ** 2)))
                peak = float(np.abs(w).max())
                t0, t1 = i * per * dt_s, (i * per + len(w)) * dt_s
                lines.append(f"    {t0:6.2f}-{t1:6.2f} s  {mean:+.4f}  {rms:.4f}  {peak:.4f}")
    return lines


def analyze(csv_path: Path, scope_npz: Path | None) -> tuple[RunFacts, list[str]]:
    """tickhealth gate (always) + run summary + optional scope summary."""
    meta, df = load_run(csv_path)
    lines, facts = summarize_run(meta, df)
    nominal = float(meta.get("tick_us_nominal", tickhealth.NOMINAL_TICK_US))
    result = tickhealth.check(df, nominal_us=nominal)
    facts.tick_passed = result.passed
    facts.tick_reasons = tuple(result.reasons)
    lines.append(tickhealth.report(result))
    if scope_npz is not None and scope_npz.exists() and scope_npz.stat().st_size > 0:
        try:
            lines += summarize_scope(scope_npz)
        except Exception as exc:  # noqa: BLE001 -- scope summary must not kill the verdict
            lines.append(f"scope: FAILED to summarize {scope_npz.name}: {exc}")
    return facts, lines


# ---------------------------------------------------------------------------
# subcommand: status
# ---------------------------------------------------------------------------

def _fmt_state(value) -> str:
    if isinstance(value, bool):
        return "ON" if value else "off"
    if isinstance(value, float):
        return f"{value:.1f}"
    return str(value)


def cmd_status(args) -> int:
    try:
        states, info = asyncio.run(api_snapshot(args.host))
    except Exception as exc:  # noqa: BLE001 -- report and exit, agents parse this
        print(f"status: API connect to {args.host} failed: {exc}")
        return EXIT_FAIL

    print(f"slider-lab @ {args.host}"
          f" (esphome {getattr(info, 'esphome_version', '?')})")
    for label, name, unit in (
        ("uptime", ENT_UPTIME, "s"),
        ("player", ENT_PLAYER, ""),
        ("position", ENT_POSITION, "%"),
        ("tick time", ENT_TICK, "us"),
        ("sync pin", ENT_SYNC, ""),
    ):
        value = states.get(name)
        shown = "<no state>" if value is None else _fmt_state(value)
        print(f"  {label + ':':<10} {shown} {unit}".rstrip())

    uptime = states.get(ENT_UPTIME)
    now = time.time()
    if isinstance(uptime, (int, float)):
        msg = detect_reboot(load_state(), float(uptime), now)
        print(f"  {msg}" if msg else "  no reboot since last labctl contact")
        save_state(float(uptime), now, args.host)
    else:
        print("  reboot check skipped (no uptime state)")
    return EXIT_OK


# ---------------------------------------------------------------------------
# subcommand: preflight -- rig-sanity gates before a campaign
# ---------------------------------------------------------------------------
# Every gate below encodes a real bench failure: floating probe on the wrong
# pot track, shunt on the wrong bridge ISEN, knob parked in the end zone,
# silent power-cycles. Falsifiable checks, one PREFLIGHT line at the end.

POS_LO, POS_HI = 1.0, 99.0            # plausible sensor range
ENDZONE_LO, ENDZONE_HI = 3.0, 97.0    # firmware interlock zone (R4)
POS_MAX_STD_PCT = 0.5                 # stability over ~1 s
BLIP_DUTY = 0.7                       # real breakaway >= 0.6 (legacy 0.42 is wrong)
BLIP_WIDTH_MS = 100                   # single short pulse, small displacement
BLIP_MIN_MOVE_PCT = 0.5               # ADC must move at least this
BLIP_MIN_DUTY_MILLI = 650             # commanded 0.7 must actually be applied
ADC_VOLTS_PER_COUNT = 2.6 / 8191.0    # 13-bit, ATTEN_DB_12 (~2.6 V usable span)
WIPER_COSWING_FACTOR = 3.0            # scope-vs-firmware delta agreement window
SHUNT_MIN_ENV_V = 0.020               # current envelope through the 0.2R shunt


@dataclass
class Check:
    name: str
    status: str  # PASS / WARN / FAIL / SKIP
    detail: str

    def line(self) -> str:
        return f"  [{self.name}] {self.status}: {self.detail}"


def assemble_preflight(checks: list[Check]) -> tuple[str, str]:
    """(level, short reason) for the PREFLIGHT line. Pure."""
    fails = [c for c in checks if c.status == "FAIL"]
    if fails:
        return "FAIL", "; ".join(f"{c.name}: {c.detail}" for c in fails)
    warns = [c for c in checks if c.status == "WARN"]
    skips = [c for c in checks if c.status == "SKIP"]
    if warns or skips:
        parts = []
        if warns:
            parts.append("warnings: " + "; ".join(f"{c.name}: {c.detail}" for c in warns))
        if skips:
            parts.append(f"{len(skips)} check(s) skipped")
        return "PASS", ", ".join(parts)
    return "PASS", "all checks green"


def blip_direction(position_pct: float) -> int:
    """+1 (fwd = position up) if below mid-travel, else -1 -- away from the
    nearer end stop. Pure."""
    return 1 if position_pct < 50.0 else -1


def blip_params(direction: int, duty: float = BLIP_DUTY,
                width_ms: int = BLIP_WIDTH_MS) -> dict:
    """Params for the goto-free actuation blip.

    Uses the bare ``dither`` program with amp 0: settle 300 ms, hold
    ``direction * duty`` for ``width_ms``, settle 300 ms -- a single short
    torque pulse with recording, no repositioning helper involved.
    """
    return {"freq": 5, "amp": 0.0, "bias": round(direction * duty, 3),
            "duration_s": width_ms / 1000.0}


def evaluate_position(samples: list[float]) -> Check:
    """Position sanity: inside [1, 99], stable over the window. Pure."""
    import numpy as np

    name = "2 position"
    values = np.asarray([s for s in samples if isinstance(s, (int, float))
                         and math.isfinite(s)], dtype=float)
    if len(values) < 3:
        return Check(name, "FAIL", f"only {len(values)} position sample(s) in 1 s"
                                   " -- sensor not updating?")
    mean, std = float(values.mean()), float(values.std())
    stats = f"{mean:.1f} % (std {std:.2f} %, n={len(values)})"
    if not POS_LO <= mean <= POS_HI:
        return Check(name, "FAIL", f"{stats} outside [{POS_LO:.0f}, {POS_HI:.0f}] %"
                                   " -- sensor/wiring implausible")
    if std > POS_MAX_STD_PCT:
        return Check(name, "FAIL", f"{stats} unstable (std > {POS_MAX_STD_PCT} %)"
                                   " -- loose wiper or drive active?")
    if mean < ENDZONE_LO or mean > ENDZONE_HI:
        return Check(name, "WARN", f"{stats} in the end zone"
                                   f" [<{ENDZONE_LO:.0f} or >{ENDZONE_HI:.0f}]"
                                   " -- torque experiments will be interlock-cut;"
                                   " move the knob toward mid-travel")
    return Check(name, "PASS", stats)


def evaluate_blip(df) -> tuple[Check, float]:
    """Actuation+feedback chain check on the blip recording.

    Returns (check, adc_delta_counts). Catches: dead motor supply, dead
    bridge, belt off, frozen ADC, end-zone parking. Pure (DataFrame in).
    """
    import numpy as np

    name = "3 actuation"
    if len(df) == 0:
        return Check(name, "FAIL", "empty blip recording"), 0.0
    ilk = int(df["ilk_endzone"].sum() + df["ilk_thermal"].sum() + df["abort"].sum())
    duty = df["duty_milli"].to_numpy()
    adc = df["adc_raw"].to_numpy(dtype=float)
    first_nz = int(np.argmax(duty != 0)) if np.any(duty != 0) else 0
    baseline = float(np.mean(adc[:first_nz] if first_nz >= 10 else adc[:50]))
    delta_counts = float(np.max(np.abs(adc - baseline)))
    moved_pct = delta_counts / 79.0  # counts per % travel (loadrun contract)
    applied = int(np.max(np.abs(duty)))
    if ilk:
        return Check(name, "FAIL", f"blip was interlock-cut on {ilk} ticks"
                                   " -- move the knob out of the end zone"), delta_counts
    if applied < BLIP_MIN_DUTY_MILLI:
        return Check(name, "FAIL", f"peak applied duty {applied} < {BLIP_MIN_DUTY_MILLI}"
                                   " milli -- torque never reached the bridge"), delta_counts
    if moved_pct < BLIP_MIN_MOVE_PCT:
        return Check(name, "FAIL",
                     f"adc moved only {moved_pct:.2f} % (< {BLIP_MIN_MOVE_PCT} %)"
                     " -- dead motor supply / bridge / belt off / frozen ADC"), delta_counts
    return Check(name, "PASS", f"blip moved {moved_pct:.2f} %"
                               f" (peak duty {applied} milli, no interlocks)"), delta_counts


def evaluate_scope_wiper(wiper_v, adc_delta_counts: float,
                         factor: float = WIPER_COSWING_FACTOR) -> Check:
    """4a: scope wiper channel co-swings with the firmware ADC. Pure.

    Tonight's failure mode: probe on a floating second pot track -- DC level
    plausible, swing ~1/50 of real. Gate: scope delta within ``factor`` of
    the firmware delta (converted via the ADC volts-per-count estimate).
    """
    import numpy as np

    name = "4a scope wiper"
    v = np.asarray(wiper_v, dtype=float)
    scope_delta = float(v.max() - v.min())
    expect_v = adc_delta_counts * ADC_VOLTS_PER_COUNT
    if expect_v <= 0.001:
        return Check(name, "FAIL", "firmware adc delta ~0 -- nothing to cross-check")
    ratio = scope_delta / expect_v
    detail = (f"scope swing {scope_delta * 1000:.0f} mV vs firmware"
              f" {expect_v * 1000:.0f} mV (ratio {ratio:.2f})")
    if not (1.0 / factor) <= ratio <= factor:
        return Check(name, "FAIL", detail + f" outside 1/{factor:.0f}x..{factor:.0f}x"
                                            " -- probe on the wrong track/node?")
    return Check(name, "PASS", detail)


def evaluate_scope_shunt(shunt_v, min_env_v: float = SHUNT_MIN_ENV_V) -> Check:
    """4b: shunt channel shows a current envelope during the blip. Pure.

    Tonight's failure mode: shunt probe on the wrong bridge's ISEN -- flat
    trace during full motor current.
    """
    import numpy as np

    name = "4b scope shunt"
    v = np.asarray(shunt_v, dtype=float)
    env = float(v.max() - v.min())
    detail = f"envelope {env * 1000:.1f} mV (need > {min_env_v * 1000:.0f} mV)"
    if env <= min_env_v:
        return Check(name, "FAIL", detail + " -- no current through the shunt"
                                            " (wrong ISEN / open lead)?")
    return Check(name, "PASS", detail)


async def _preflight_blip(rig: Rig, host: str, direction: int,
                          note: str) -> tuple[Path | None, str]:
    """Fire one blip, wait for idle, fetch the dump. Returns (csv, error)."""
    params = blip_params(direction)
    await rig.fire("dither", json.dumps(params, separators=(",", ":")))
    # run_experiment refuses silently over the API (busy / dump active / abort
    # switch) -- catch that here instead of analyzing a stale buffer later
    left = await rig.wait_until(ENT_PLAYER, lambda v: v != "idle", timeout_s=3.0)
    if left is None:
        return None, ("run_experiment was refused (player never left idle)"
                      " -- device log has the reason (dump active? abort on?)")
    await asyncio.sleep(0.3 + params["duration_s"] + 0.3 + RECORD_TAIL_S)
    state = await rig.wait_until(ENT_PLAYER, lambda v: v == "idle", timeout_s=15.0)
    if state != "idle":
        return None, "player did not return to idle after the blip"
    try:
        csv_path = _fetch_with_retry(host, note, attempts=10)
    except Exception as exc:  # noqa: BLE001
        return None, f"blip dump fetch failed: {exc}"
    meta, _df = load_run(csv_path)
    if meta.get("experiment") != "dither":
        return None, (f"fetched dump is from {meta.get('experiment')!r}, not the"
                      f" blip -- stale buffer ({csv_path.name})")
    return csv_path, ""


async def _preflight_async(args) -> int:
    import requests

    checks: list[Check] = []

    def emit(check: Check) -> None:
        checks.append(check)
        print(check.line())

    print(f"preflight: slider-lab @ {args.host}")

    # -- 1 CONNECTIVITY: API + HTTP dump + uptime/reboot ---------------------
    t0 = time.monotonic()
    rig = Rig(args.host)
    try:
        await rig.connect()
        states = await rig.snapshot()
    except Exception as exc:  # noqa: BLE001
        emit(Check("1 connectivity", "FAIL", f"API connect failed: {exc}"))
        level, reason = assemble_preflight(checks)
        print(f"PREFLIGHT: {level} {reason}")
        return EXIT_FAIL if level == "FAIL" else EXIT_OK
    t_api = time.monotonic() - t0
    try:
        # NOTE: consume the body fully. An abandoned streaming GET keeps the
        # device's dump_active_ latched until its socket send fails, and every
        # run_experiment in between is refused ("CSV dump in progress").
        resp = requests.get(f"http://{args.host}/lab/dump.csv", timeout=15)
        http_code = resp.status_code
        del resp
    except requests.RequestException as exc:
        http_code = None
        http_err = str(exc)
    uptime = states.get(ENT_UPTIME)
    player = states.get(ENT_PLAYER)
    now = time.time()
    reboot_msg = None
    if isinstance(uptime, (int, float)):
        reboot_msg = detect_reboot(load_state(), float(uptime), now)
        save_state(float(uptime), now, args.host)
    detail = (f"api {t_api:.1f} s, http dump"
              f" {http_code if http_code is not None else 'unreachable'},"
              f" uptime {uptime:.0f} s, player {player!r}"
              if isinstance(uptime, (int, float)) else "no uptime state")
    if http_code != 200 or not isinstance(uptime, (int, float)) or player != "idle":
        if http_code is None:
            detail += f" ({http_err})"
        emit(Check("1 connectivity", "FAIL", detail))
        await rig.disconnect()
        level, reason = assemble_preflight(checks)
        print(f"PREFLIGHT: {level} {reason}")
        return EXIT_FAIL
    if reboot_msg:
        emit(Check("1 connectivity", "WARN", f"{detail} -- {reboot_msg}"))
    else:
        emit(Check("1 connectivity", "PASS", detail))

    # -- 2 POSITION SANITY: plausible + stable over ~1 s ---------------------
    samples = await rig.collect(ENT_POSITION, 1.2)
    pos_check = evaluate_position(samples)
    emit(pos_check)
    position = (sum(samples) / len(samples)) if samples else 50.0

    # -- 3 ACTUATION+FEEDBACK: short 0.7-duty blip away from the near end ----
    direction = blip_direction(position)
    adc_delta = 0.0
    csv_path, err = await _preflight_blip(
        rig, args.host, direction, note="preflight actuation blip")
    if csv_path is None:
        emit(Check("3 actuation", "FAIL", err))
    else:
        _meta, df = load_run(csv_path)
        blip_check, adc_delta = evaluate_blip(df)
        blip_check.detail += f" [{csv_path.name}]"
        emit(blip_check)

    # -- 4 SCOPE CROSS-CHECKS: free-run capture during a second blip ---------
    if not args.scope:
        emit(Check("4 scope", "SKIP", "no --scope"))
    elif csv_path is None:
        emit(Check("4 scope", "SKIP", "actuation blip already failed"))
    else:
        folder = DATA_DIR / date.today().isoformat()
        folder.mkdir(parents=True, exist_ok=True)
        npz = folder / f"{next_seq(folder)}_scope_preflight.npz"
        npz.touch()
        scope = ScopeCapture("diag", npz)
        scope.start()
        if not scope.wait_armed(timeout_s=60.0):
            scope.kill()
            emit(Check("4 scope", "FAIL", "capture.py never armed"
                                          " (PicoScope busy/disconnected?)"))
        else:
            csv2, err2 = await _preflight_blip(
                rig, args.host, -direction, note="preflight scope blip")
            saved = scope.finish(timeout_s=60.0)
            if csv2 is None or not saved:
                emit(Check("4 scope", "FAIL",
                           err2 or "scope capture not saved"))
            else:
                import numpy as np

                _m2, df2 = load_run(csv2)
                _c2, adc_delta2 = evaluate_blip(df2)
                with np.load(npz, allow_pickle=False) as data:
                    keys = sorted(k for k in data.files if k.startswith("volts_"))
                    wiper = data[keys[0]] if keys else None
                    shunt = data[keys[1]] if len(keys) > 1 else None
                if wiper is None:
                    emit(Check("4 scope", "FAIL", "npz has no channels"))
                else:
                    emit(evaluate_scope_wiper(wiper, adc_delta2))
                    if args.no_shunt:
                        emit(Check("4b scope shunt", "SKIP",
                                   "--no-shunt (hardware fix pending)"))
                    elif shunt is None:
                        emit(Check("4b scope shunt", "FAIL", "no Ch B in capture"))
                    else:
                        emit(evaluate_scope_shunt(shunt))

    await rig.disconnect()
    level, reason = assemble_preflight(checks)
    print(f"PREFLIGHT: {level} {reason}")
    return EXIT_OK if level == "PASS" else EXIT_FAIL


def cmd_preflight(args) -> int:
    try:
        return asyncio.run(_preflight_async(args))
    except Exception as exc:  # noqa: BLE001 -- agents need the message, not a trace
        print(f"preflight: {type(exc).__name__}: {exc}")
        print(f"PREFLIGHT: FAIL {type(exc).__name__}: {exc}")
        return EXIT_FAIL


# ---------------------------------------------------------------------------
# subcommand: flash
# ---------------------------------------------------------------------------

def _run_stage(cmd: list[str], cwd: Path) -> tuple[int, float, str]:
    """Run one toolchain stage. Returns (rc, seconds, combined output)."""
    t0 = time.monotonic()
    try:
        proc = subprocess.run(cmd, cwd=cwd, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        rc, out = proc.returncode, proc.stdout or ""
    except OSError as exc:  # permission denial / missing binary lands here
        rc, out = 126, f"{type(exc).__name__}: {exc}"
    return rc, time.monotonic() - t0, out


def _wait_http_200(host: str, timeout_s: float = 90.0) -> float | None:
    """Poll http://host/lab/dump.csv until 200. Returns seconds, None on timeout.

    The body is consumed (tiny right after boot: the recorder is empty) --
    abandoning a streaming GET would latch the device's dump_active_ and get
    subsequent run_experiment calls refused.
    """
    import requests

    t0 = time.monotonic()
    deadline = t0 + timeout_s
    while time.monotonic() < deadline:
        try:
            resp = requests.get(f"http://{host}/lab/dump.csv", timeout=5)
            if resp.status_code == 200:
                return time.monotonic() - t0
        except requests.RequestException:
            pass
        time.sleep(1.0)
    return None


def cmd_flash(args) -> int:
    t_total = time.monotonic()
    yaml = FIRMWARE_YAML
    print(f"flash: {yaml.name} -> {args.host}")

    # stage 1: compile (incremental)
    rc, t_compile, out = _run_stage([ESPHOME_BIN, "compile", str(yaml)], yaml.parent)
    if rc != 0:
        print(f"  compile FAILED after {t_compile:.1f} s -- last lines:")
        for line in out.splitlines()[-40:]:
            print(f"    {line}")
        return EXIT_FAIL
    print(f"  compile: OK ({t_compile:.1f} s)")

    # stage 2: OTA upload to the IP
    upload_cmd = [ESPHOME_BIN, "upload", str(yaml), "--device", args.host]
    rc, t_upload, out = _run_stage(upload_cmd, yaml.parent)
    if rc != 0:
        print(f"  upload FAILED/denied (rc {rc}) after {t_upload:.1f} s -- last lines:")
        for line in out.splitlines()[-15:]:
            print(f"    {line}")
        print("  handoff -- run this exact command (cwd firmware/):")
        print(f"    cd {shlex.quote(str(yaml.parent))} && {shlex.join(upload_cmd)}")
        print(f"  then verify with: labctl status   (expect uptime < 60 s)")
        return EXIT_FLASH_HANDOFF
    print(f"  upload: OK ({t_upload:.1f} s)")

    # stage 3: boot -- wait for HTTP 200, then prove reboot into the new build
    t_http = _wait_http_200(args.host)
    if t_http is None:
        print("  boot: device did not answer HTTP 200 within 90 s -- check the rig")
        return EXIT_FAIL
    uptime = None
    for _ in range(3):  # API may lag HTTP by a few seconds after boot
        try:
            states, _info = asyncio.run(api_snapshot(args.host))
            uptime = states.get(ENT_UPTIME)
            if isinstance(uptime, (int, float)):
                break
        except Exception:  # noqa: BLE001
            time.sleep(2.0)
    if not isinstance(uptime, (int, float)):
        print(f"  boot: HTTP up after {t_http:.1f} s but API/uptime unreachable")
        return EXIT_FAIL
    if uptime >= 60.0:
        print(f"  boot: HTTP up, but uptime {uptime:.0f} s >= 60 s --"
              f" device did NOT reboot into the new build")
        return EXIT_FAIL
    t_boot = t_http  # HTTP wait dominates; API check is part of the same stage
    save_state(float(uptime), time.time(), args.host)
    total = time.monotonic() - t_total
    print(f"  boot: OK ({t_boot:.1f} s to HTTP 200, uptime {uptime:.0f} s < 60 s"
          f" -- new build confirmed)")
    print(f"flash: DONE  compile {t_compile:.1f} s + upload {t_upload:.1f} s"
          f" + boot {t_boot:.1f} s = {total:.1f} s total")
    return EXIT_OK


# ---------------------------------------------------------------------------
# subcommand: run
# ---------------------------------------------------------------------------

def _fetch_with_retry(host: str, note: str | None,
                      attempts: int = 30, delay_s: float = 1.0) -> Path:
    """slideranalysis.fetch with retry on 503 (dump refused while busy)."""
    import requests

    last: Exception | None = None
    for _ in range(attempts):
        try:
            return fetch(host, note=note)
        except requests.HTTPError as exc:
            last = exc
            if exc.response is not None and exc.response.status_code != 503:
                raise
        except requests.RequestException as exc:  # brief WiFi hiccups
            last = exc
        time.sleep(delay_s)
    raise LabctlError(f"fetch failed after {attempts} attempts: {last}")


async def _run_async(args) -> int:
    t_total = time.monotonic()
    params = validate_params(args.experiment, args.params)
    params_json = json.dumps(params, separators=(",", ":")) if params else ""
    est_s = estimate_duration_s(args.experiment, params)
    idle_deadline_s = max(30.0, est_s * 1.25 + 10.0)  # ~firmware wall-clock budget
    print(f"run: {args.experiment}  params: {params_json or '(defaults)'}"
          f"  planned <= {est_s:.1f} s")

    # scope arm (optional) -- MUST see ARMED before firing
    scope: ScopeCapture | None = None
    scope_npz: Path | None = None
    if args.scope:
        folder = DATA_DIR / date.today().isoformat()
        folder.mkdir(parents=True, exist_ok=True)
        scope_npz = folder / f"{next_seq(folder)}_scope_{args.scope}.npz"
        scope_npz.touch()  # reserve the seq slot; capture.py overwrites it
        scope = ScopeCapture(args.scope, scope_npz)
        t0 = time.monotonic()
        scope.start()
        if not scope.wait_armed():
            scope.kill()
            print("scope: no ARMED marker from capture.py -- NOT firing")
            return EXIT_FAIL
        print(f"scope: armed after {time.monotonic() - t0:.1f} s -> {scope_npz.name}")

    # connect, pre-flight, fire
    rig = Rig(args.host)
    facts = RunFacts(experiment=args.experiment, scope_expected=scope is not None,
                     fetched=False)
    try:
        await rig.connect()
        states = await rig.snapshot()
        uptime = states.get(ENT_UPTIME)
        if isinstance(uptime, (int, float)):
            msg = detect_reboot(load_state(), float(uptime), time.time())
            if msg:
                print(f"  {msg}")
        player = states.get(ENT_PLAYER)
        if player is not None and player != "idle":
            print(f"run: player is {player!r}, not idle -- refusing to fire")
            await rig.disconnect()
            if scope:
                scope.kill()
            return EXIT_FAIL
        pos = states.get(ENT_POSITION)
        if isinstance(pos, (int, float)):
            print(f"  pre-flight: player idle, position {pos:.1f} %")
        await rig.fire(args.experiment, params_json)
        t_fired = time.monotonic()
        print(f"  fired at +{t_fired - t_total:.1f} s")

        if args.watch:
            # watch mode: stay connected (API traffic during the recording --
            # deliberate tradeoff), print player-state transitions live
            last = None

            async def until_idle():
                nonlocal last
                deadline = time.monotonic() + idle_deadline_s
                while time.monotonic() < deadline:
                    cur = await rig.wait_until(
                        ENT_PLAYER, lambda v: v != last, timeout_s=1.0)
                    if cur is not None and cur != last:
                        print(f"  [watch +{time.monotonic() - t_fired:5.1f}s] {cur}")
                        last = cur
                    if last == "idle":
                        return True
                return False
            # skip the initial idle -> running edge being mistaken for done
            await asyncio.sleep(1.0)
            facts.player_returned_idle = await until_idle()
        else:
            # quiet mode (default): zero API traffic while the recorder runs
            await rig.disconnect()
            wait_s = est_s + 1.5
            print(f"  quiet: disconnected, waiting {wait_s:.1f} s"
                  f" (planned duration + margin)")
            await asyncio.sleep(wait_s)
            rig = Rig(args.host)
            await rig.connect()
            state = await rig.wait_until(
                ENT_PLAYER, lambda v: v == "idle",
                timeout_s=max(5.0, idle_deadline_s - wait_s))
            facts.player_returned_idle = state == "idle"
        states = await rig.snapshot()
        uptime = states.get(ENT_UPTIME)
        if isinstance(uptime, (int, float)):
            save_state(float(uptime), time.time(), args.host)
    finally:
        await rig.disconnect()

    if not facts.player_returned_idle:
        print(f"run: player still not idle after {idle_deadline_s:.0f} s")
        if scope:
            scope.kill()
        level, reason = assemble_verdict(facts)
        print(f"VERDICT: {level} {reason}")
        return EXIT_FAIL
    t_done = time.monotonic()
    print(f"  player idle at +{t_done - t_total:.1f} s")

    # fetch the recording (dump serves 503 until the post-run tail is written)
    csv_path: Path | None = None
    try:
        csv_path = _fetch_with_retry(args.host, args.note)
        facts.fetched = True
        print(f"  fetched: {csv_path}  (+{time.monotonic() - t_total:.1f} s)")
    except (LabctlError, Exception) as exc:  # noqa: BLE001
        print(f"  fetch FAILED: {exc}")

    # scope: capture should have completed by now; wait for the file
    if scope is not None:
        facts.scope_saved = scope.finish()
        print(f"  scope: {'saved ' + scope.out_path.name if facts.scope_saved else 'FAILED'}")

    # analyze
    if csv_path is not None:
        run_facts, lines = analyze(csv_path, scope_npz)
        run_facts.expected_experiment = args.experiment
        run_facts.scope_expected = facts.scope_expected
        run_facts.scope_saved = facts.scope_saved
        run_facts.player_returned_idle = facts.player_returned_idle
        facts = run_facts
        for line in lines:
            print(line)

    level, reason = assemble_verdict(facts)
    print(f"run: total {time.monotonic() - t_total:.1f} s wall")
    print(f"VERDICT: {level} {reason}")
    return {"PASS": EXIT_OK, "WARN": EXIT_WARN}.get(level, EXIT_FAIL)


def cmd_run(args) -> int:
    try:
        return asyncio.run(_run_async(args))
    except LabctlError as exc:
        print(f"run: {exc}")
        return EXIT_FAIL
    except Exception as exc:  # noqa: BLE001 -- agents need the message, not a trace
        print(f"run: {type(exc).__name__}: {exc}")
        return EXIT_FAIL


# ---------------------------------------------------------------------------
# subcommand: cycle
# ---------------------------------------------------------------------------

def cmd_cycle(args) -> int:
    t0 = time.monotonic()
    rc = cmd_flash(args)
    if rc == EXIT_FLASH_HANDOFF:
        print("cycle: flash handed off -- rerun `labctl cycle` (or `labctl run ...`)"
              " after the orchestrator flashed")
        return rc
    if rc != EXIT_OK:
        return rc
    args.experiment = args.experiment or "noise_floor"
    rc = cmd_run(args)
    print(f"cycle: total {time.monotonic() - t0:.1f} s (flash + run)")
    return rc


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="labctl",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--host", default=RIG_HOST,
                        help=f"rig IP (default {RIG_HOST}; avoid mDNS, it is slow)")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status", help="one-shot rig state + reboot detection")

    p_pre = sub.add_parser("preflight",
                           help="rig-sanity gates: connectivity, position,"
                                " actuation blip, scope cross-check")
    p_pre.add_argument("--scope", action="store_true",
                       help="also cross-check scope channels (free-run capture"
                            " during a second blip; needs the PicoScope free)")
    p_pre.add_argument("--no-shunt", action="store_true",
                       help="skip the shunt-envelope check (4b) until the"
                            " shunt hardware fix lands")

    sub.add_parser("flash", help="esphome compile + OTA upload + boot verify, timed")

    p_run = sub.add_parser("run", help="fire an experiment, fetch + analyze, verdict")
    p_run.add_argument("experiment", help=f"one of: {', '.join(KNOWN_EXPERIMENTS)}")
    p_run.add_argument("--params", default=None, help="params JSON object for the program")
    p_run.add_argument("--scope", default=None, metavar="PRESET",
                       help=f"arm capture.py first ({', '.join(SCOPE_PRESETS)})")
    watch = p_run.add_mutually_exclusive_group()
    watch.add_argument("--quiet", dest="watch", action="store_false", default=False,
                       help="disconnect during the recording (default)")
    watch.add_argument("--watch", dest="watch", action="store_true",
                       help="stay connected, stream player state (adds API traffic)")
    p_run.add_argument("--note", default=None, help="appended to the day's notes.md")

    p_cycle = sub.add_parser("cycle", help="flash + run in one command")
    p_cycle.add_argument("--experiment", default="noise_floor",
                         help="experiment for the run stage (default noise_floor)")
    p_cycle.add_argument("--params", default=None, help="params JSON object")
    p_cycle.add_argument("--scope", default=None, metavar="PRESET")
    p_cycle.add_argument("--note", default=None)
    p_cycle.set_defaults(watch=False)

    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    handler = {
        "status": cmd_status,
        "preflight": cmd_preflight,
        "flash": cmd_flash,
        "run": cmd_run,
        "cycle": cmd_cycle,
    }[args.command]
    try:
        return handler(args)
    except LabctlError as exc:
        print(f"labctl: {exc}")
        return EXIT_FAIL


if __name__ == "__main__":
    sys.exit(main())
