"""L1 thermal tripwire (Jan 2026-11-07: "5 min Vollgas unkritisch").

Falsifiable: 300 s of uninterrupted duty 1.0 does NOT trip the L1 thermal
interlock; 480 s does (the protection still exists). Negative control: the
pre-2026-11-07 lab-rig values (tau 10 s, trip 0.5) trip within seconds.
"""

from __future__ import annotations

import subprocess

from test_sil import RUNNER, build_runner  # noqa: F401  (session fixture)


def _thermal(seconds: float, duty: float, old: bool = False) -> dict:
    cmd = [str(RUNNER), "--thermal", str(seconds), str(duty)]
    if old:
        cmd.append("--thermal-old")
    out = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
    return {k: float(v) for k, v in (l.split("=", 1) for l in out.splitlines())}


def test_thermal_5min_full_duty_no_trip():
    r = _thermal(300.0, 1.0)
    assert r["thermal_trip_tick"] == -1, f"tripped at {r['thermal_trip_tick']} ms"


def test_thermal_8min_full_duty_trips():
    r = _thermal(480.0, 1.0)
    trip = r["thermal_trip_tick"]
    assert trip >= 0, "no trip after 8 min full duty: protection gone"
    assert 340_000 <= trip <= 380_000, f"trip at {trip} ms, expected ~360 s"


def test_thermal_negative_control_old_values():
    r = _thermal(300.0, 1.0, old=True)
    assert 0 <= r["thermal_trip_tick"] < 10_000, "old values should trip in ~7 s"
