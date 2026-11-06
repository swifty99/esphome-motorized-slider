import numpy as np
import pytest
from synth import write_run

from slideranalysis.loadrun import (
    COUNTS_PER_PERCENT,
    counts_to_percent,
    load_run,
    raw_to_percent,
    sample_columns,
)


def test_roundtrip_with_samples_and_flags(tmp_path):
    rng = np.random.default_rng(1)
    n = 500
    samples = rng.integers(3990, 4010, size=(n, 4))
    flags = np.zeros(n, dtype=int)
    flags[0] = 1            # segment_start
    flags[100] = 1 | 16     # segment_start + goto_active
    flags[200] = 2          # interlock_endzone
    flags[300] = 4 | 8      # interlock_thermal + abort
    path = write_run(
        tmp_path / "run.csv",
        samples=samples,
        experiment="step_response",
        params={"duty": 0.7, "dir": 1},
        duty_milli=700,
        flags=flags,
    )

    meta, df = load_run(path)

    # meta: keys parsed, params is a real dict, numbers are numbers
    assert meta["experiment"] == "step_response"
    assert meta["params"] == {"duty": 0.7, "dir": 1}
    assert meta["tick_us_nominal"] == 1000
    assert "slider_lab" in meta["firmware"]
    assert "13bit" in meta["adc"]

    # data columns incl. optional s0..s3
    assert len(df) == n
    assert sample_columns(df) == ["s0", "s1", "s2", "s3"]
    assert np.allclose(df["adc_raw"], samples.mean(axis=1))
    assert (df["duty_milli"] == 700).all()

    # flags decoded into booleans
    assert bool(df.loc[0, "segment_start"]) and not bool(df.loc[0, "goto_active"])
    assert bool(df.loc[100, "segment_start"]) and bool(df.loc[100, "goto_active"])
    assert bool(df.loc[200, "ilk_endzone"])
    assert bool(df.loc[300, "ilk_thermal"]) and bool(df.loc[300, "abort"])
    assert not df.loc[1, ["segment_start", "ilk_endzone", "ilk_thermal",
                          "abort", "goto_active"]].any()


def test_without_sample_columns(tmp_path):
    path = write_run(tmp_path / "run.csv", n_ticks=100)
    _meta, df = load_run(path)
    assert sample_columns(df) == []
    assert list(df.columns[:6]) == ["t_ms", "adc_raw", "duty_milli", "seg",
                                    "flags", "tick_us"]


def test_percent_conversion_helpers():
    # provisional endpoints: raw 8000 -> 0 %, raw 100 -> 100 % (inverted)
    assert raw_to_percent(8000) == pytest.approx(0.0)
    assert raw_to_percent(100) == pytest.approx(100.0)
    assert raw_to_percent(4050) == pytest.approx(50.0)
    np.testing.assert_allclose(raw_to_percent(np.array([8000.0, 100.0])),
                               [0.0, 100.0])
    # amplitude conversion: 79 counts per % travel
    assert COUNTS_PER_PERCENT == pytest.approx(79.0)
    assert counts_to_percent(79.0) == pytest.approx(1.0)
