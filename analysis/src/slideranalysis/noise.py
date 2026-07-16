"""M1 analysis: sensor noise floor.

Per run: RMS (mean-removed) and peak-to-peak of ``adc_raw`` in counts and in
% of travel; intra-tick sample spread when the raw-sample columns s0..s3 are
present; Welch PSD (fs = 1 kHz) with labeled peaks. ``compare(runs)`` overlays
several runs (e.g. WiFi on vs off) in one figure.

All statistics are computed on the raw data -- no filtering in here.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy import signal

from .loadrun import counts_to_percent, load_run, sample_columns

FS_HZ = 1000.0  # recorder tick rate


@dataclass
class NoiseStats:
    label: str
    n: int
    duration_s: float
    mean_counts: float
    rms_counts: float
    ptp_counts: float
    rms_pct: float
    ptp_pct: float
    intra_tick_sigma: float | None = None  # mean per-tick std over s0..s3 [counts]
    intra_tick_ptp: float | None = None    # mean per-tick peak-to-peak [counts]


def stats(df, label: str = "") -> NoiseStats:
    """Noise statistics of one run (pure)."""
    x = df["adc_raw"].to_numpy(dtype=float)
    mean = float(x.mean())
    rms = float(np.sqrt(np.mean((x - mean) ** 2)))
    ptp = float(np.ptp(x))

    intra_sigma = intra_ptp = None
    scols = sample_columns(df)
    if len(scols) >= 2:
        s = df[scols].to_numpy(dtype=float)
        intra_sigma = float(np.mean(s.std(axis=1)))
        intra_ptp = float(np.mean(np.ptp(s, axis=1)))

    return NoiseStats(
        label=label,
        n=len(x),
        duration_s=len(x) / FS_HZ,
        mean_counts=mean,
        rms_counts=rms,
        ptp_counts=ptp,
        rms_pct=float(counts_to_percent(rms)),
        ptp_pct=float(counts_to_percent(ptp)),
        intra_tick_sigma=intra_sigma,
        intra_tick_ptp=intra_ptp,
    )


def psd(df, fs: float = FS_HZ, nperseg: int | None = None):
    """Welch PSD of adc_raw. Returns (f [Hz], Pxx [counts^2/Hz]).

    Default nperseg 2048 (~0.5 Hz resolution at 1 kHz) so a 10 s run still
    averages ~9 segments -- a smooth enough floor for peak detection.
    """
    x = df["adc_raw"].to_numpy(dtype=float)
    if nperseg is None:
        nperseg = min(2048, len(x))
    return signal.welch(x, fs=fs, nperseg=nperseg, detrend="constant")


def find_psd_peaks(f, pxx, prominence_db: float = 10.0, max_peaks: int = 5):
    """Peaks in the PSD, sorted by prominence (descending).

    Returns a list of (freq_hz, pxx_value) tuples. Prominence is evaluated on
    the dB curve so 'peak' means 'sticks >= prominence_db out of the local
    noise floor'.
    """
    db = 10.0 * np.log10(np.asarray(pxx) + 1e-30)
    idx, props = signal.find_peaks(db, prominence=prominence_db)
    order = np.argsort(props["prominences"])[::-1][:max_peaks]
    return [(float(f[i]), float(pxx[i])) for i in idx[order]]


def compare(runs, fs: float = FS_HZ, annotate_peaks: bool = True):
    """Overlay Welch PSDs of several runs in one figure.

    ``runs``: dict {label: df} or iterable of (label, df). A single-run dict
    gives the plain M1 PSD plot. Returns the Figure.
    """
    import matplotlib.pyplot as plt

    items = list(runs.items()) if isinstance(runs, dict) else list(runs)
    fig, ax = plt.subplots(figsize=(9, 5))
    for label, df in items:
        st = stats(df, label)
        f, pxx = psd(df, fs=fs)
        ax.semilogy(
            f, pxx, lw=0.8,
            label=f"{label}  (RMS {st.rms_counts:.2f} cts = {st.rms_pct:.3f} %)",
        )
        if annotate_peaks:
            for freq, val in find_psd_peaks(f, pxx):
                ax.annotate(
                    f"{freq:.0f} Hz", xy=(freq, val),
                    xytext=(0, 6), textcoords="offset points",
                    ha="center", fontsize=8,
                )
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("PSD [counts$^2$/Hz]")
    ax.set_title("adc_raw noise PSD (Welch)")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    return fig


def _print_stats(st: NoiseStats, peaks) -> None:
    print(f"run: {st.label}")
    print(f"  n={st.n} ({st.duration_s:.1f} s)  mean={st.mean_counts:.1f} counts")
    print(f"  RMS: {st.rms_counts:.3f} counts = {st.rms_pct:.4f} % travel")
    print(f"  p-p: {st.ptp_counts:.1f} counts = {st.ptp_pct:.4f} % travel")
    if st.intra_tick_sigma is not None:
        print(f"  intra-tick (s0..s3): mean sigma={st.intra_tick_sigma:.2f} counts,"
              f" mean p-p={st.intra_tick_ptp:.2f} counts")
    if peaks:
        txt = ", ".join(f"{fr:.1f} Hz ({10 * np.log10(v + 1e-30):.1f} dB)" for fr, v in peaks)
        print(f"  PSD peaks: {txt}")
    else:
        print("  PSD peaks: none above prominence threshold")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="M1 noise analysis: RMS/p-p + Welch PSD; multiple CSVs overlay."
    )
    parser.add_argument("csv", type=Path, nargs="+", help="lab CSV dump(s)")
    parser.add_argument("--out", type=Path, default=None,
                        help="figure path (default: <csv>_noise.png / noise_compare.png)")
    parser.add_argument("--fs", type=float, default=FS_HZ, help="sample rate [Hz]")
    args = parser.parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")

    runs = []
    for path in args.csv:
        _meta, df = load_run(path)
        label = path.stem
        runs.append((label, df))
        st = stats(df, label)
        f, pxx = psd(df, fs=args.fs)
        _print_stats(st, find_psd_peaks(f, pxx))

    fig = compare(runs, fs=args.fs)
    if args.out is not None:
        out = args.out
    elif len(args.csv) == 1:
        out = args.csv[0].with_name(args.csv[0].stem + "_noise.png")
    else:
        out = args.csv[0].with_name("noise_compare.png")
    fig.savefig(out, dpi=120)
    print(f"figure: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
