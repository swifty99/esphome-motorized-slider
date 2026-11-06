"""Findings plots (PNG, light surface). One place for style so every figure
in docs/findings/ reads as one system.

Palette (validated adjacent-pair CVD separation + contrast, dataviz method):
measured = blue, model/sim = orange, extras aqua/violet; ink for text, gray
for reference/context lines. Identity is never color-alone: every multi-series
axis carries a legend, and key traces get direct labels where cheap.
"""

from __future__ import annotations

import numpy as np

BLUE = "#2a78d6"     # measured / data
ORANGE = "#eb6834"   # model / simulated
AQUA = "#1baf7a"     # third series (contrast WARN -> always legend + label)
VIOLET = "#4a3aa7"   # fourth series
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#d9d8d2"


def _style(ax, title=None, xlabel=None, ylabel=None):
    ax.grid(True, color=GRID, lw=0.6, alpha=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=8)
    if title:
        ax.set_title(title, fontsize=10, color=INK, loc="left")
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=9, color=INK2)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=9, color=INK2)


def plot_sensor(points, model, out):
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.5, 3.6))
    groups = (  # decay mode is a real coupling factor (M2-CSV grid)
        ("fast", BLUE, "FAST mode / unpowered"),
        ("slow", ORANGE, "SLOW mode"),
    )
    dd = np.linspace(0, max([0.85] + [p.duty_abs for p in points]), 100)
    for mode, color, label in groups:
        pts = [p for p in points if p.mode == mode]
        if not pts:
            continue
        d = np.array([p.duty_abs for p in pts])
        ax1.scatter(d, [p.sigma_counts for p in pts], s=22, color=color,
                    zorder=3, label=label)
        ax2.scatter(d, [p.ptp_counts for p in pts], s=22, color=color,
                    zorder=3, label=label)
    ax1.plot(dd, [model.sigma_counts(x) for x in dd], color=INK2, lw=2,
             label=r"pooled model $\sqrt{\sigma_0^2 + c\,|d|}$")
    ax1.legend(fontsize=8, frameon=False)
    _style(ax1, "adc_raw noise vs duty (stalled)", "|duty|", r"$\sigma$ [counts]")
    ax1.set_ylim(0, max(max(p.sigma_counts for p in points), 14) * 1.15)

    ax2.axhline(model.ptp0_counts or 0, color=INK2, lw=1, ls="--")
    ax2.annotate("unpowered p-p", xy=(dd[-1], model.ptp0_counts or 0),
                 xytext=(0, 4), textcoords="offset points",
                 ha="right", fontsize=8, color=INK2)
    ax2.legend(fontsize=8, frameon=False)
    _style(ax2, "PWM spike tail (p-p, same segments)", "|duty|", "p-p [counts]")
    ax2.set_ylim(0, max(p.ptp_counts for p in points) * 1.15)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def plot_static_map(smap, out):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
    for ax, direction, lb, ub, est, name in (
        (axes[0], +1, smap.lb_fwd, smap.ub_fwd, smap.est_fwd, "forward (position up)"),
        (axes[1], -1, smap.lb_rev, smap.ub_rev, smap.est_rev, "reverse (position down)"),
    ):
        g = smap.grid
        both = ~np.isnan(lb) & ~np.isnan(ub)
        ax.fill_between(g, np.where(np.isnan(lb), np.nan, lb),
                        np.where(np.isnan(ub), np.nan, ub),
                        where=both, color=BLUE, alpha=0.18, lw=0,
                        label="bracketed [stall, break]")
        ax.plot(g, est, color=ORANGE, lw=2, label="estimate (sim uses this)")
        for e in smap.evidence:
            if e.direction != direction:
                continue
            if e.kind in ("stall", "restick"):
                ax.plot(e.pos, e.duty, marker="^", ms=6, color=BLUE, mew=0, ls="")
            elif e.kind in ("break", "creep"):
                ax.plot(e.pos, e.duty, marker="v", ms=6, color=VIOLET, mew=0, ls="")
            elif e.kind == "fly":
                ax.plot(e.pos, e.duty, marker="v", ms=5, color=AQUA, mew=0, ls="")
        ax.plot([], [], "^", color=BLUE, ls="", label="held, no motion (lower bound)")
        ax.plot([], [], "v", color=VIOLET, ls="", label="breakaway/creep (~actual)")
        ax.plot([], [], "v", color=AQUA, ls="", label="flew (upper bound)")
        ax.legend(fontsize=7, frameon=False, loc="upper right")
        _style(ax, f"static friction map -- {name}", "position [% travel]",
               "breakaway duty" if direction > 0 else None)
        ax.set_ylim(0, 1.08)
        ax.set_xlim(0, 100)
    fig.suptitle("Breakaway (static friction) vs position -- duty units, VM=5 V",
                 fontsize=10, color=INK, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out, dpi=130)
    plt.close(fig)


def plot_kinetic_map(kfit, smap, out):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    for f, se, color, label in (
        (kfit.fk_fwd, kfit.fk_fwd_se, BLUE, "kinetic fwd"),
        (kfit.fk_rev, kfit.fk_rev_se, ORANGE, "kinetic rev"),
    ):
        good = ~np.isnan(f)
        ax.errorbar(kfit.nodes[good], f[good], yerr=2 * se[good], color=color,
                    lw=2, marker="o", ms=5, capsize=3, label=label + " (±2σ)")
    ax.plot(smap.grid, smap.est_fwd, color=BLUE, lw=1, ls="--", alpha=0.6,
            label="static fwd (est)")
    ax.plot(smap.grid, smap.est_rev, color=ORANGE, lw=1, ls="--", alpha=0.6,
            label="static rev (est)")
    ax.legend(fontsize=8, frameon=False, ncols=2)
    _style(ax, f"Friction maps (duty units) -- b={kfit.b:.2e}±{2*kfit.b_se:.0e} "
               f"duty/(%/s), m={kfit.m:.2e}±{2*kfit.m_se:.0e} duty/(%/s²)",
           "position [% travel]", "friction [duty]")
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 1.0)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def plot_overlay(t, pos_meas, sim, duty, res, out, title=""):
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(9, 5), sharex=True, height_ratios=[3, 1])
    ax1.plot(t, pos_meas, color=BLUE, lw=1.3, label="measured")
    ax1.plot(t, sim.pos, color=ORANGE, lw=1.6, label="simulated")
    ax1.legend(fontsize=8, frameon=False, loc="best")
    _style(ax1, f"{title}   RMSE {res.rmse_pct:.2f} %, final {res.final_err_pct:+.2f} %, "
                f"max {res.max_err_pct:.2f} %", None, "position [%]")
    ax2.plot(t, duty, color=INK2, lw=1.2, label="applied duty")
    ax2.legend(fontsize=8, frameon=False, loc="best")
    _style(ax2, None, "t [s]", "duty")
    ax2.set_ylim(-1.05, 1.05)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def plot_observer(t, detail, sc, out, title=""):
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(9, 5.6), sharex=True, height_ratios=[2, 3])
    ax1.plot(t, detail["pos_ref"], color=BLUE, lw=1.2, label="position (filtered meas)")
    ax1.plot(t, detail["p_est"], color=ORANGE, lw=1.2, label="KF position")
    ax1.legend(fontsize=8, frameon=False)
    _style(ax1, title, None, "position [%]")

    ax2.plot(t, detail["v_ref"], color=BLUE, lw=1.6,
             label="zero-phase reference (offline)")
    ax2.plot(t, detail["v_est"], color=ORANGE, lw=1.2,
             label=f"KF (RMS {sc.rms_moving:.0f} %/s moving, lag {sc.lag_ms:+.0f} ms)")
    ax2.plot(t, detail["v_legacy"], color=AQUA, lw=1.0, alpha=0.9,
             label=f"legacy PT1-150ms (RMS {sc.rms_moving_legacy:.0f} %/s, "
                   f"lag {sc.lag_ms_legacy:+.0f} ms)")
    ax2.legend(fontsize=8, frameon=False)
    _style(ax2, None, "t [s]", "velocity [%/s]")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
