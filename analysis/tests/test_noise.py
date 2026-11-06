import numpy as np
import pytest
from synth import write_run

from slideranalysis.loadrun import load_run
from slideranalysis.noise import compare, find_psd_peaks, psd, stats


def test_rms_of_known_white_noise(tmp_path):
    sigma = 5.0
    rng = np.random.default_rng(10)
    adc = 4000.0 + rng.normal(0.0, sigma, 20000)
    path = write_run(tmp_path / "white.csv", adc=adc)
    _meta, df = load_run(path)
    st = stats(df, "white")
    assert st.rms_counts == pytest.approx(sigma, rel=0.05)
    assert st.rms_pct == pytest.approx(st.rms_counts / 79.0, rel=1e-9)
    assert st.ptp_counts > 6 * sigma  # sanity: p-p of 20k gaussian samples
    assert st.intra_tick_sigma is None  # no s0..s3 in this run


def test_psd_peak_detection_finds_seeded_50hz_tone(tmp_path):
    rng = np.random.default_rng(11)
    n = 20000
    t = np.arange(n) / 1000.0
    adc = 4000.0 + rng.normal(0.0, 2.0, n) + 3.0 * np.sin(2 * np.pi * 50.0 * t)
    path = write_run(tmp_path / "tone.csv", adc=adc)
    _meta, df = load_run(path)
    f, pxx = psd(df)
    peaks = find_psd_peaks(f, pxx)
    assert peaks, "no peaks found at all"
    # the most prominent peak is the seeded 50 Hz tone
    assert peaks[0][0] == pytest.approx(50.0, abs=1.5)


def test_intra_tick_spread_from_sample_columns(tmp_path):
    rng = np.random.default_rng(12)
    sigma = 4.0
    samples = 4000.0 + rng.normal(0.0, sigma, size=(5000, 4))
    path = write_run(tmp_path / "samples.csv", samples=np.rint(samples))
    _meta, df = load_run(path)
    st = stats(df, "samples")
    assert st.intra_tick_sigma is not None
    assert st.intra_tick_ptp is not None
    # E[std of 4 gaussian samples, ddof=0] ~ 0.80 sigma; just bound it sanely
    assert 0.5 * sigma < st.intra_tick_sigma < 1.2 * sigma
    assert st.intra_tick_ptp > st.intra_tick_sigma


def test_compare_overlays_runs(tmp_path):
    p1 = write_run(tmp_path / "a.csv", n_ticks=4000, seed=13)
    p2 = write_run(tmp_path / "b.csv", n_ticks=4000, seed=14)
    _m1, df1 = load_run(p1)
    _m2, df2 = load_run(p2)
    fig = compare({"wifi on": df1, "wifi off": df2}, annotate_peaks=False)
    assert len(fig.axes) == 1
    assert len(fig.axes[0].lines) == 2
    labels = [t.get_text() for t in fig.axes[0].get_legend().get_texts()]
    assert any("wifi on" in x for x in labels)
