import numpy as np
import pytest
from synth import write_run

from slideranalysis.loadrun import load_run
from slideranalysis.tickhealth import check, plot, report


def test_pass_on_clean_run(tmp_path):
    path = write_run(tmp_path / "clean.csv", n_ticks=2000, seed=2)
    _meta, df = load_run(path)
    res = check(df)
    assert res.passed, res.reasons
    assert res.missed == 0
    assert res.duplicated == 0
    assert res.jitter_max_us == pytest.approx(0.0, abs=0.01)
    assert res.tick_us_p99 <= 900.0  # work duration under the guard band
    assert "PASS" in report(res)
    assert "work duration" in report(res)


def test_fail_on_seeded_3ms_gap(tmp_path):
    # negative control: a single 3 ms gap = 2 missed ticks -> structural FAIL
    path = write_run(tmp_path / "gap.csv", n_ticks=2000, seed=2,
                     gap_after=700, gap_dt_ms=3)
    _meta, df = load_run(path)
    res = check(df)
    assert not res.passed
    assert res.missed == 2
    assert any("missed" in r for r in res.reasons)
    assert "FAIL" in report(res)


def test_jitter_fail_but_structural_pass(tmp_path):
    # negative control: one tick 300 us late is NOT a missed tick (structural
    # counters stay zero) but trips the 250 us jitter gate -> FAIL
    path = write_run(tmp_path / "jitter_bad.csv", n_ticks=1000,
                     late_at=400, late_us=300.0)
    _meta, df = load_run(path)
    res = check(df)
    assert res.missed == 0
    assert res.duplicated == 0
    assert res.jitter_max_us == pytest.approx(300.0, abs=1.0)
    assert not res.passed
    assert any("jitter" in r for r in res.reasons)


def test_jitter_within_limit_passes(tmp_path):
    # positive control: one tick 120 us late (above the old +/-100 us band,
    # like the observed hardware worst case 115 us) -> overall PASS, the
    # excursion shows up in the jitter stats instead of as a false miss
    path = write_run(tmp_path / "jitter_ok.csv", n_ticks=1000,
                     late_at=400, late_us=120.0)
    _meta, df = load_run(path)
    res = check(df)
    assert res.passed, res.reasons
    assert res.missed == 0
    assert res.jitter_max_us == pytest.approx(120.0, abs=1.0)
    assert res.jitter_p50_us == pytest.approx(0.0, abs=0.01)
    assert "120.0 us" in report(res)


def test_fail_on_duplicated_record(tmp_path):
    path = write_run(tmp_path / "dup.csv", n_ticks=500, seed=3, duplicate_at=250)
    _meta, df = load_run(path)
    res = check(df)
    assert not res.passed
    assert res.duplicated == 1


def test_fail_on_duration_overrun(tmp_path):
    # negative control: timeline clean, but 2 % of ticks take 1200 us
    # -> p99 > 900 us guard band -> must FAIL
    n = 1000
    tick = np.full(n, 280.0)
    tick[::50] = 1200.0
    path = write_run(tmp_path / "overrun.csv", n_ticks=n, tick_us=tick)
    _meta, df = load_run(path)
    res = check(df)
    assert not res.passed
    assert any("work-duration" in r for r in res.reasons)
    assert res.tick_us_out_of_band == 20


def test_period_mode_behind_switch(tmp_path):
    # legacy period semantics only via tick_us_is_period=True
    path = write_run(tmp_path / "period.csv", n_ticks=500, tick_us=1000.0)
    _meta, df = load_run(path)
    assert check(df, tick_us_is_period=True).passed
    assert not check(df).passed  # 1000 us of WORK would be an overrun
    # typical 250 us durations: PASS by default, FAIL as a period
    path2 = write_run(tmp_path / "typical.csv", n_ticks=500, tick_us=250.0)
    _meta, df2 = load_run(path2)
    assert check(df2).passed
    assert not check(df2, tick_us_is_period=True).passed


def test_plot_smoke(tmp_path):
    path = write_run(tmp_path / "clean.csv", n_ticks=300, seed=4)
    _meta, df = load_run(path)
    res = check(df)
    fig = plot(df, res, title="clean.csv")
    assert len(fig.axes) == 2
