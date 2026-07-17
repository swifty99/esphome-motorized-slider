"""Tests for the M3 current analysis (R fit, rung extraction)."""

import json

import numpy as np
import pytest

from slideranalysis.m3current import (
    FS_SCOPE,
    RungMeas,
    SHUNT_OHM,
    V_PER_PCT,
    VM,
    collect_all,
    fit_r,
    position_spread,
)
import slideranalysis.m3current as m3

from synth import write_run


def test_fit_r_roundtrip():
    rng = np.random.default_rng(0)
    R, V_eff, c = 8.0, 3.0, 0.14
    rows = []
    for duty in (0.5, 0.6, 0.68, 0.74, 0.8):
        for k in range(3):
            i = duty * (duty * VM - V_eff) / R + c + rng.normal(0, 5e-4)
            rows.append(RungMeas("syn", "fast", 1, "50fwd", duty,
                                 i * 1000.0, "stall"))
    fit = fit_r(rows, "fast")
    assert fit.r_ohm == pytest.approx(R, rel=0.15)
    assert fit.v_eff == pytest.approx(V_eff, abs=0.3)
    assert fit.resid_rms_ma < 2.0


def test_extract_rungs_stall_and_flight(tmp_path, monkeypatch):
    """Synthetic npz+CSV pair: one stalled rung, one rung that flies then
    parks at the stop -- classification and current means must come out."""
    fs = FS_SCOPE
    rung_s, pause_s = 1.0, 0.5
    duties = [0.5, 0.6]
    n = int((0.2 + 2 * (rung_s + pause_s)) * fs)
    i_ma = np.zeros(n)
    wiper = np.full(n, 1.0)  # volts
    t0 = int(0.2 * fs)
    # rung 1: stall at 100 mA
    s1, e1 = t0, t0 + int(rung_s * fs)
    i_ma[s1:e1] = 100.0
    # rung 2: 150 mA, slider flies 30 % in 0.4 s then parks
    s2 = e1 + int(pause_s * fs)
    e2 = s2 + int(rung_s * fs)
    i_ma[s2:e2] = 150.0
    fly = int(0.4 * fs)
    wiper[s2:s2 + fly] = 1.0 - np.linspace(0, 30 * V_PER_PCT, fly)
    wiper[s2 + fly:] = 1.0 - 30 * V_PER_PCT
    meta = json.dumps({"rate_hz": fs})
    np.savez(tmp_path / "01_scope_ladder.npz", meta=meta,
             volts_isen_0r2=i_ma / 1000.0 * SHUNT_OHM, volts_wiper=wiper)
    write_run(tmp_path / "02_duty_ladder.csv", experiment="duty_ladder",
              params={"d0": 0.5, "d1": 0.6, "step": 0.1, "step_ms": 1000,
                      "pause_ms": 500, "dir": "fwd", "mode": "fast"},
              adc=np.full(100, 4000.0))
    monkeypatch.setattr(m3, "LADDER_PAIRS",
                        {"01_scope_ladder": ("02_duty_ladder", "50fwd")})
    rows = collect_all(tmp_path)
    kinds = {(r.duty, r.kind): r for r in rows}
    assert kinds[(0.5, "stall")].i_mean_ma == pytest.approx(100.0, abs=1.0)
    assert (0.6, "flight") in kinds
    assert kinds[(0.6, "flight")].v_mean > 40.0
    assert kinds[(0.6, "atstop")].i_mean_ma == pytest.approx(150.0, abs=1.0)


def test_position_spread_number():
    rows = [RungMeas("a", "fast", 1, p, 0.5, i, "stall")
            for p, i in (("20fwd", 100.0), ("50fwd", 102.0), ("80rev", 99.0))]
    # (max-min)/2 / mean = 1.5/100.33
    assert position_spread(rows) == pytest.approx(0.01495, abs=1e-3)
