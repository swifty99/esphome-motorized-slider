"""Phase-1 gate check: is the 1 kHz recorder timeline healthy?

Three criteria on one run, all must pass:

1. STRUCTURAL timeline defects (gate: must be zero). From delta-t between
   consecutive ``t_ms`` records: ``missed`` = dt >= 1.5 nominal periods
   (1500 us), counted as ``round(dt/nominal) - 1`` per event; ``duplicated``
   = dt <= 0.5 nominal periods (500 us), incl. non-monotonic records.
2. JITTER (quantitative). |dt - nominal| over all remaining intervals,
   reported as p50/p99/max; gate: max <= nominal/4 (250 us). Evidence from
   real hardware (5 quiet-mode runs, 52k intervals): worst observed jitter
   ~115 us -- a tick 101 us late is jitter, not a missed tick, so it must not
   trip the structural gate.
3. ``tick_us`` = tick WORK DURATION (what the firmware logs; the tick budget
   is 1000 us). Gate: p99 <= nominal - tol = 900 us -- a guard band under the
   budget, since work approaching the period starts eating the next tick.
   The period interpretation (tick_us as measured tick interval) is available
   via ``tick_us_is_period=True`` (CLI: ``--tick-us-period``): then p50/p95/p99
   must sit inside the 1000 +/- 100 us band.

Verdict: PASS iff all three criteria hold.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .loadrun import load_run

NOMINAL_TICK_US = 1000.0
TOL_US = 100.0
MISSED_FACTOR = 1.5     # dt >= 1.5 * nominal -> missed tick(s)
DUPLICATE_FACTOR = 0.5  # dt <= 0.5 * nominal -> duplicated / non-monotonic
JITTER_FACTOR = 0.25    # default jitter limit = nominal / 4


@dataclass
class TickHealthResult:
    n_ticks: int
    duration_s: float
    missed: int
    duplicated: int
    jitter_p50_us: float
    jitter_p99_us: float
    jitter_max_us: float
    tick_us_p50: float
    tick_us_p95: float
    tick_us_p99: float
    tick_us_out_of_band: int
    nominal_us: float
    tol_us: float
    jitter_limit_us: float
    tick_us_is_period: bool
    reasons: list[str] = field(default_factory=list)
    dt_us: np.ndarray = field(default_factory=lambda: np.empty(0), repr=False)

    @property
    def passed(self) -> bool:
        return not self.reasons


def check(
    df,
    nominal_us: float = NOMINAL_TICK_US,
    tol_us: float = TOL_US,
    jitter_limit_us: float | None = None,
    tick_us_is_period: bool = False,
) -> TickHealthResult:
    """Run the tick-health gate on a loaded run (pure, no I/O)."""
    if jitter_limit_us is None:
        jitter_limit_us = JITTER_FACTOR * nominal_us
    lo, hi = nominal_us - tol_us, nominal_us + tol_us
    t_ms = df["t_ms"].to_numpy(dtype=float)
    tick_us = df["tick_us"].to_numpy(dtype=float)
    n = len(t_ms)

    nan = float("nan")
    if n < 2:
        return TickHealthResult(
            n_ticks=n, duration_s=0.0, missed=0, duplicated=0,
            jitter_p50_us=nan, jitter_p99_us=nan, jitter_max_us=nan,
            tick_us_p50=nan, tick_us_p95=nan, tick_us_p99=nan,
            tick_us_out_of_band=0, nominal_us=nominal_us, tol_us=tol_us,
            jitter_limit_us=jitter_limit_us,
            tick_us_is_period=tick_us_is_period,
            reasons=["fewer than 2 records"],
        )

    # 1. structural defects
    dt_us = np.diff(t_ms) * 1000.0
    miss_thresh = MISSED_FACTOR * nominal_us
    dup_thresh = DUPLICATE_FACTOR * nominal_us
    over = dt_us >= miss_thresh
    under = dt_us <= dup_thresh
    missed = int(np.sum(np.maximum(np.rint(dt_us[over] / nominal_us) - 1, 1)))
    duplicated = int(np.sum(under))

    # 2. jitter of the structurally sound intervals
    jitter = np.abs(dt_us[~over & ~under] - nominal_us)
    if jitter.size:
        j50, j99 = (float(v) for v in np.percentile(jitter, [50, 99]))
        jmax = float(jitter.max())
    else:
        j50 = j99 = jmax = nan

    # 3. tick_us work duration (or legacy period mode)
    p50, p95, p99 = (float(v) for v in np.percentile(tick_us, [50, 95, 99]))
    duration_limit = nominal_us - tol_us
    if tick_us_is_period:
        out_of_band = int(np.sum((tick_us < lo) | (tick_us > hi)))
    else:
        out_of_band = int(np.sum(tick_us > duration_limit))

    reasons: list[str] = []
    if missed:
        reasons.append(f"{missed} missed tick(s) ({int(np.sum(over))} gap event(s),"
                       f" dt >= {miss_thresh:.0f} us)")
    if duplicated:
        reasons.append(f"{duplicated} duplicated/non-monotonic record(s)"
                       f" (dt <= {dup_thresh:.0f} us)")
    if jitter.size and jmax > jitter_limit_us:
        reasons.append(f"dt jitter max={jmax:.1f} us exceeds"
                       f" {jitter_limit_us:.0f} us limit")
    if tick_us_is_period:
        for name, val in (("p50", p50), ("p95", p95), ("p99", p99)):
            if not lo <= val <= hi:
                reasons.append(
                    f"tick_us {name}={val:.1f} us outside {lo:.0f}..{hi:.0f} us"
                )
    elif p99 > duration_limit:
        reasons.append(
            f"tick_us p99={p99:.1f} us exceeds the {duration_limit:.0f} us "
            f"work-duration limit (budget {nominal_us:.0f} us)"
        )

    return TickHealthResult(
        n_ticks=n,
        duration_s=(t_ms[-1] - t_ms[0]) / 1000.0,
        missed=missed,
        duplicated=duplicated,
        jitter_p50_us=j50,
        jitter_p99_us=j99,
        jitter_max_us=jmax,
        tick_us_p50=p50,
        tick_us_p95=p95,
        tick_us_p99=p99,
        tick_us_out_of_band=out_of_band,
        nominal_us=nominal_us,
        tol_us=tol_us,
        jitter_limit_us=jitter_limit_us,
        tick_us_is_period=tick_us_is_period,
        reasons=reasons,
        dt_us=dt_us,
    )


def report(result: TickHealthResult) -> str:
    """Human-readable verdict text."""
    lo = result.nominal_us - result.tol_us
    hi = result.nominal_us + result.tol_us
    miss_thresh = MISSED_FACTOR * result.nominal_us
    dup_thresh = DUPLICATE_FACTOR * result.nominal_us
    percentiles = (
        f"p50={result.tick_us_p50:.1f}  p95={result.tick_us_p95:.1f}"
        f"  p99={result.tick_us_p99:.1f} us"
    )
    if result.tick_us_is_period:
        tick_line = (f"  tick_us (period): {percentiles},"
                     f" {result.tick_us_out_of_band} sample(s) outside"
                     f" {lo:.0f}..{hi:.0f} us")
    else:
        tick_line = (f"  tick_us (work duration): {percentiles},"
                     f" {result.tick_us_out_of_band} sample(s) > {lo:.0f} us limit")
    lines = [
        f"tick health: {'PASS' if result.passed else 'FAIL'}",
        f"  records: {result.n_ticks}, span: {result.duration_s:.3f} s",
        f"  dt(t_ms) structural: {result.missed} missed (dt >= {miss_thresh:.0f} us),"
        f" {result.duplicated} duplicated (dt <= {dup_thresh:.0f} us)",
        f"  dt(t_ms) jitter: p50={result.jitter_p50_us:.1f}"
        f"  p99={result.jitter_p99_us:.1f}  max={result.jitter_max_us:.1f} us"
        f" (limit {result.jitter_limit_us:.0f} us)",
        tick_line,
    ]
    for r in result.reasons:
        lines.append(f"  FAIL: {r}")
    return "\n".join(lines)


def plot(df, result: TickHealthResult, title: str = ""):
    """Figure: delta-t histogram + tick_us over time. Returns the Figure."""
    import matplotlib.pyplot as plt

    lo = result.nominal_us - result.tol_us
    hi = result.nominal_us + result.tol_us
    miss_thresh = MISSED_FACTOR * result.nominal_us
    dup_thresh = DUPLICATE_FACTOR * result.nominal_us
    j_lo = result.nominal_us - result.jitter_limit_us
    j_hi = result.nominal_us + result.jitter_limit_us
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))

    dt = result.dt_us
    lo_edge = min(dup_thresh, float(dt.min())) - 100.0
    hi_edge = max(miss_thresh, float(dt.max())) + 100.0
    ax1.hist(dt, bins=np.linspace(lo_edge, hi_edge, 61), color="C0")
    ax1.set_yscale("log")
    ax1.axvline(dup_thresh, color="C3", ls="-", lw=1,
                label=f"structural ({dup_thresh:.0f} / {miss_thresh:.0f} us)")
    ax1.axvline(miss_thresh, color="C3", ls="-", lw=1)
    ax1.axvline(j_lo, color="C1", ls="--", lw=1,
                label=f"jitter limit (+/- {result.jitter_limit_us:.0f} us)")
    ax1.axvline(j_hi, color="C1", ls="--", lw=1)
    ax1.legend(fontsize=8)
    ax1.set_xlabel("delta-t between records [us]")
    ax1.set_ylabel("count")
    ax1.set_title("timeline delta-t histogram")

    t_s = df["t_ms"].to_numpy(dtype=float) / 1000.0
    ax2.plot(t_s, df["tick_us"].to_numpy(dtype=float), lw=0.5, color="C0")
    if result.tick_us_is_period:
        ax2.axhline(lo, color="C3", ls="--", lw=1)
        ax2.axhline(hi, color="C3", ls="--", lw=1)
        ax2.set_title("tick period over run")
    else:
        ax2.axhline(lo, color="C3", ls="--", lw=1,
                    label=f"{lo:.0f} us limit")
        ax2.axhline(result.nominal_us, color="C1", ls=":", lw=1,
                    label=f"{result.nominal_us:.0f} us budget")
        ax2.legend(fontsize=8, loc="upper right")
        ax2.set_title("tick work duration over run")
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("tick_us [us]")

    verdict = "PASS" if result.passed else "FAIL"
    fig.suptitle(f"{title}  --  tick health: {verdict}".strip())
    fig.tight_layout()
    return fig


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Phase-1 tick-health gate: zero structural timeline defects, "
                    "dt jitter max <= nominal/4, tick_us p99 <= nominal - tol."
    )
    parser.add_argument("csv", type=Path, help="lab CSV dump")
    parser.add_argument("--out", type=Path, default=None,
                        help="figure path (default: <csv>_tickhealth.png)")
    parser.add_argument("--nominal-us", type=float, default=None,
                        help="nominal tick [us] (default: meta tick_us_nominal, else 1000)")
    parser.add_argument("--tol-us", type=float, default=TOL_US,
                        help="tick_us tolerance [us] (default 100)")
    parser.add_argument("--jitter-limit-us", type=float, default=None,
                        help="dt jitter gate [us] (default nominal/4 = 250)")
    parser.add_argument("--tick-us-period", action="store_true",
                        help="treat tick_us as measured tick period instead of "
                             "work duration (gate: p50/p95/p99 inside the band "
                             "instead of p99 <= nominal - tol)")
    args = parser.parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")

    meta, df = load_run(args.csv)
    nominal = args.nominal_us
    if nominal is None:
        nominal = float(meta.get("tick_us_nominal", NOMINAL_TICK_US))
    result = check(df, nominal_us=nominal, tol_us=args.tol_us,
                   jitter_limit_us=args.jitter_limit_us,
                   tick_us_is_period=args.tick_us_period)

    fig = plot(df, result, title=args.csv.name)
    out = args.out or args.csv.with_name(args.csv.stem + "_tickhealth.png")
    fig.savefig(out, dpi=120)
    print(f"figure: {out}")
    print(report(result))
    return 0 if result.passed else 1


if __name__ == "__main__":
    sys.exit(main())
