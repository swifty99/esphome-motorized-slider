"""Friction identification: static/breakaway map + kinetic map + b + m.

Everything works in DUTY UNITS: no current measurement exists yet (scope died
2026-07-16), so electrical K and R are not identifiable and "force" means
"stall-equivalent duty" under the stated assumption force ~ duty at stall.

Two separate fits:

1. STATIC (breakaway) map ``F_s(p, dir)`` -- assembled from *evidence
   intervals*, not a curve fit, because most observations are one-sided:
   - a held duty with no motion      -> lower bound  F_s > d
   - a breakaway/onset event         -> F_s ~ d      (tight upper bound)
   - a segment that flew through     -> upper bound over the traversed range
   - creep (intermittent stick-slip) -> F_s ~ d
   The map reports [lb, ub] per grid position and direction; the point
   estimate for the simulator is the interval midpoint (margin rules stated
   in ``StaticMap.est_notes``). Refit automatically ingests ``breakaway_grid``
   runs when they land -- each trial is validity-gated (the 2026-07-16 run 58
   detector tripped on sensor noise: ramps ended at duty 0.005 with zero real
   displacement; such trials are counted as invalid, not used).

2. KINETIC fit -- one *linear* least-squares problem over every sample where
   the slider actually slides (|v| > v_min):

       u_i = m*a_i + b*v_i + s_i * Fk(p_i, dir_i)        s = sign(v)

   with Fk parameterized as node values on a position grid (linear
   interpolation, separate node sets per direction). Linear in all
   parameters -> honest covariance from the residuals. Includes coast/brake
   samples (u = 0 while still moving): on the L9110 both decay modes brake,
   so b is the *electrodynamic + viscous* damping and stays in effect at
   duty 0. Degeneracies are checked (parameter correlations) and reported.

   KNOWN BIAS: a comes from double-smoothed position (40 Hz zero-phase);
   transients faster than ~25 ms are smeared, so m from this stage is a
   lower-bound-flavored estimate. ``plantsim.refine_inertia`` re-fits m
   against raw position by simulation to quantify that bias.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .segments import Run, Segment, split_segments
from .velocity import edge_mask, zero_phase_velocity

FS_HZ = 1000.0

# classification thresholds (percent travel / percent-per-second)
STALL_DPOS = 0.6       # below: stalled (noise band is ~0.15 % sigma, spikes ~1.5 %)
FLY_DPOS = 8.0         # above: clean slide
CREEP_VMAX = 25.0      # peak |v| below this + moved -> creep (stick-slip)
RESTICK_WINDOW = 0.5   # s of terminal standstill under load -> re-stick evidence
MIN_HOLD_TICKS = 100   # shortest active segment considered for stall evidence


@dataclass
class StaticEvidence:
    kind: str            # stall | creep | break | fly | traverse | restick
    direction: int       # +1 fwd (position up), -1 rev
    duty: float          # |duty|
    pos: float           # position of the event [%] (start pos for fly)
    pos_hi: float | None  # traverse only: covered range [pos, pos_hi]
    source: str          # "<run>:seg<k>"
    note: str = ""

    def bound(self) -> str:
        return {"stall": "lb", "restick": "lb", "creep": "ub~", "break": "ub~",
                "fly": "ub", "traverse": "ub", "sustain": "info"}[self.kind]


@dataclass
class StaticMap:
    grid: np.ndarray
    lb_fwd: np.ndarray
    ub_fwd: np.ndarray
    lb_rev: np.ndarray
    ub_rev: np.ndarray
    est_fwd: np.ndarray
    est_rev: np.ndarray
    evidence: list[StaticEvidence]
    invalid_grid_trials: int
    window_pct: float
    est_notes: str = ""
    conflicts: list[str] = field(default_factory=list)  # lb > ub grid points

    def fs(self, p: float, direction: int) -> float:
        est = self.est_fwd if direction > 0 else self.est_rev
        return float(np.interp(p, self.grid, est))


def _classify_active_segment(s: Segment, direction: int, d: float,
                             out: list[StaticEvidence]) -> None:
    """One constant-duty active segment -> static-friction evidence.

    Robust against ADC spike outliers (observed up to ~3 % singles under PWM):
    displacement is judged on medians of the first/last 50 ms, span on the
    1..99 percentile band -- one spiked tick must not turn a stall into
    fake creep evidence.
    """
    src = f"{s.run}:seg{s.seg}"
    k = min(50, max(s.n // 4, 1))
    p_start = float(np.median(s.pos[:k]))
    p_end = float(np.median(s.pos[-k:]))
    dpos = (p_end - p_start) * direction  # displacement in drive direction
    span = float(np.percentile(s.pos, 99) - np.percentile(s.pos, 1))
    _, v = zero_phase_velocity(s.pos)
    vmax = float(np.max(np.abs(v))) if s.n >= 15 else abs(dpos) / max(s.n / FS_HZ, 1e-3)

    if abs(dpos) < STALL_DPOS:
        if span > 3.0:
            return  # ambiguous (moved out and back, or dense spikes): no evidence
        driven = float(np.mean(np.abs(s.duty) > 0.01))
        if driven < 0.9:
            return  # torque was cut (end-zone interlock): not a real hold
        if s.n >= MIN_HOLD_TICKS:
            out.append(StaticEvidence("stall", direction, d, float(np.median(s.pos)),
                                      None, src, f"held {s.n} ms, no motion"))
        return
    if dpos < 0:
        return  # moved against the drive: not friction evidence
    if dpos < FLY_DPOS and vmax < CREEP_VMAX:
        out.append(StaticEvidence("creep", direction, d, p_start, None, src,
                                  f"crept {dpos:+.1f} % (vmax {vmax:.0f} %/s)"))
    else:
        lo, hi = float(min(s.pos[0], s.pos[-1])), float(max(s.pos[0], s.pos[-1]))
        out.append(StaticEvidence("fly", direction, d, float(s.pos[0]), None, src,
                                  f"flew {dpos:+.1f} %"))
        out.append(StaticEvidence("traverse", direction, d, lo, hi, src, ""))

    # terminal standstill under sustained load = re-stick (or creep arrest);
    # judged on medians (PWM noise p-p alone is ~1 % over 0.5 s)
    k = int(RESTICK_WINDOW * FS_HZ)
    if s.n > k + 50:
        tail_pos = s.pos[-k:]
        tail_duty = s.duty[-k:]
        net = abs(float(np.median(tail_pos[-100:])) - float(np.median(tail_pos[:100])))
        span = float(np.percentile(tail_pos, 99) - np.percentile(tail_pos, 1))
        if net < 0.3 and span < 1.5 and np.all(np.abs(tail_duty) > 0.01):
            out.append(StaticEvidence("restick", direction, d,
                                      float(np.median(tail_pos)), None, src,
                                      f"stationary last {RESTICK_WINDOW:.1f} s under load"))


def _triangle_onsets(s: Segment, out: list[StaticEvidence]) -> None:
    """Up-ramp of a triangle: first sustained-motion onset -> break evidence."""
    if s.n < 200:
        return
    _, v = zero_phase_velocity(s.pos)
    direction = 1 if s.params.get("dir", "fwd") == "fwd" else -1
    vv = v * direction
    moving = vv > 10.0
    # first index where motion sustains >= 30 ms
    kernel = 30
    run_len = np.convolve(moving.astype(int), np.ones(kernel, dtype=int), mode="valid")
    idx = np.flatnonzero(run_len >= kernel)
    if len(idx) == 0:
        return
    i = int(idx[0])
    out.append(StaticEvidence("break", direction, abs(float(s.duty[i])),
                              float(s.pos[i]), None, f"{s.run}:seg{s.seg}",
                              f"ramp onset, v>10 %/s sustained"))


def _grid_trials(segs: list[Segment], out: list[StaticEvidence]) -> int:
    """breakaway_grid: (ramp, hold) pairs, validity-gated. Returns #invalid.

    The firmware logs the RAMP-TERMINATION duty, which lags true onset by up
    to ~+0.05 at creep speeds (detector waits for the move threshold). The
    true onset is refitted from the raw trace: last quiet tick (|v| < 5 %/s
    on the zero-phase velocity) before the position leaves the start by
    0.8 % in drive direction -> duty at that tick.

    Robustness: start/end positions are medians (single ADC spikes of ~1-3 %
    under PWM must not validate a phantom move -- runs 58/2026-07-16 and
    01/2026-07-17 characterized exactly that failure in the firmware
    detector).
    """
    invalid = 0
    for i in range(len(segs) - 1):
        ramp, hold = segs[i], segs[i + 1]
        if ramp.goto or hold.goto:
            continue
        d_end = float(ramp.duty[-1])
        if d_end == 0.0 or hold.n < 40:
            continue
        if hold.duty_const is None or abs((hold.duty_const or 0) - d_end) > 1e-9:
            continue
        if not np.all(np.abs(ramp.duty) <= abs(d_end) + 1e-9):
            continue
        direction = 1 if d_end > 0 else -1
        pos = np.concatenate([ramp.pos, hold.pos])
        duty = np.concatenate([ramp.duty, hold.duty])
        p_start = float(np.median(pos[: max(min(30, ramp.n // 3), 1)]))
        p_end = float(np.median(hold.pos[-min(30, hold.n):]))
        disp = (p_end - p_start) * direction
        if disp < 1.0:
            invalid += 1
            continue
        pf, v = zero_phase_velocity(pos)
        crossed = (pf - p_start) * direction > 0.8
        ci = np.flatnonzero(crossed)
        onset_duty = abs(d_end)
        onset_pos = p_start
        note = "grid trial (onset = ramp end)"
        if len(ci):
            quiet = np.flatnonzero(np.abs(v[: ci[0]]) < 5.0)
            j = int(quiet[-1]) if len(quiet) else int(ci[0])
            onset_duty = abs(float(duty[j]))
            note = (f"grid trial, onset refit {onset_duty:.3f} "
                    f"(ramp ended {abs(d_end):.3f}), moved {disp:+.1f} %")
        out.append(StaticEvidence("break", direction, onset_duty, onset_pos,
                                  None, f"{ramp.run}:seg{ramp.seg}", note))
        # sustain quality: distance covered while held after breakaway
        out.append(StaticEvidence("sustain", direction, abs(d_end), onset_pos,
                                  None, f"{ramp.run}:seg{ramp.seg + 1}",
                                  f"hold_dpos {disp * direction:+.1f} %"))
    return invalid


def collect_static_evidence(runs: list[Run]) -> tuple[list[StaticEvidence], int]:
    ev: list[StaticEvidence] = []
    invalid = 0
    for run in runs:
        segs = split_segments(run)
        if run.experiment == "breakaway_grid":
            invalid += _grid_trials(segs, ev)
            continue
        for s in segs:
            if s.n < 50:
                continue
            if run.experiment == "triangle" and not s.goto:
                d = s.duty_const
                if d is None and float(np.max(np.abs(s.duty))) > 0.05:
                    _triangle_onsets(s, ev)
                    continue
            if s.goto:
                # goto = constant-duty reposition; stall or traverse evidence
                active = np.abs(s.duty) > 0.01
                if active.sum() < MIN_HOLD_TICKS:
                    continue
                d = float(np.median(np.abs(s.duty[active])))
                direction = 1 if np.median(s.duty[active]) > 0 else -1
                sub = Segment(run=s.run, experiment=s.experiment, params=s.params,
                              seg=s.seg, t=s.t[active], pos=s.pos[active],
                              duty=s.duty[active], adc=s.adc[active], goto=False,
                              endzone_ticks=0)
                _classify_active_segment(sub, direction, d, ev)
                continue
            d = s.duty_const
            if d is None or d == 0.0:
                continue
            direction = 1 if d > 0 else -1
            _classify_active_segment(s, direction, abs(d), ev)
    return ev, invalid


def fit_static_map(evidence: list[StaticEvidence], invalid_trials: int = 0,
                   grid_step: float = 5.0, window: float = 8.0) -> StaticMap:
    grid = np.arange(0.0, 100.0 + 1e-9, grid_step)
    maps = {}
    conflicts: list[str] = []
    for direction in (+1, -1):
        lb = np.full(len(grid), np.nan)
        ub = np.full(len(grid), np.nan)
        for k, p in enumerate(grid):
            lbs, ubs = [], []
            for e in evidence:
                if e.direction != direction or e.bound() == "info":
                    continue
                if e.kind == "traverse":
                    # interior of the traverse only (endpoints may be sticky)
                    if e.pos + 2.0 <= p <= (e.pos_hi or e.pos) - 2.0:
                        ubs.append(e.duty)
                    continue
                if abs(e.pos - p) > window:
                    continue
                if e.bound() == "lb":
                    lbs.append(e.duty)
                else:
                    ubs.append(e.duty)
            if lbs:
                lb[k] = max(lbs)
            if ubs:
                ub[k] = min(ubs)
            if lbs and ubs and lb[k] > ub[k] + 1e-9:
                conflicts.append(
                    f"{'fwd' if direction > 0 else 'rev'}@{p:.0f}%: held {lb[k]:.2f} "
                    f"without motion but broke at {ub[k]:.2f} elsewhere/other trial "
                    "-- run-to-run variability or hysteresis, est uses the max")
        # point estimate: midpoint where bracketed; single-sided rules below
        est = np.full(len(grid), np.nan)
        both = ~np.isnan(lb) & ~np.isnan(ub)
        est[both] = np.maximum((lb[both] + ub[both]) / 2.0, lb[both])
        only_ub = np.isnan(lb) & ~np.isnan(ub)
        est[only_ub] = ub[only_ub] - 0.025
        only_lb = ~np.isnan(lb) & np.isnan(ub)
        est[only_lb] = lb[only_lb] + 0.05
        # fill gaps by nearest-neighbour/linear interpolation for the simulator
        if np.any(~np.isnan(est)):
            good = ~np.isnan(est)
            est = np.interp(grid, grid[good], est[good])
        maps[direction] = (lb, ub, est)

    return StaticMap(
        grid=grid,
        lb_fwd=maps[1][0], ub_fwd=maps[1][1],
        lb_rev=maps[-1][0], ub_rev=maps[-1][1],
        est_fwd=maps[1][2], est_rev=maps[-1][2],
        evidence=evidence, invalid_grid_trials=invalid_trials,
        window_pct=window,
        est_notes=("est = mid(lb,ub) where bracketed; ub-0.025 where only an upper "
                   "bound exists; lb+0.05 where only a lower bound exists; gaps "
                   "filled by interpolation between evidenced positions"),
        conflicts=conflicts,
    )


# ---------------------------------------------------------------- kinetic ---

@dataclass
class KineticFit:
    nodes: np.ndarray
    fk_fwd: np.ndarray        # NaN where unsupported; friction at |v| >> v_strib
    fk_fwd_se: np.ndarray
    fk_rev: np.ndarray
    fk_rev_se: np.ndarray
    m: float                  # duty / (%/s^2)
    m_se: float
    b: float                  # duty / (%/s)
    b_se: float
    # Stribeck excess: friction = fk(p,dir) + g_dir * exp(-|v|/v_strib)
    g_fwd: float
    g_fwd_se: float
    g_rev: float
    g_rev_se: float
    v_strib: float            # %/s, selected on a small grid by residual
    corr_mb: float
    resid_rms_duty: float
    n_rows: int
    rows_by_run: dict[str, int]
    v_min: float
    fc_hz: float
    warnings: list[str] = field(default_factory=list)

    def fk(self, p, direction: int):
        f = self.fk_fwd if direction > 0 else self.fk_rev
        good = ~np.isnan(f)
        if not np.any(good):
            return np.full_like(np.asarray(p, dtype=float), np.nan)
        return np.interp(p, self.nodes[good], f[good])


MOTION_EXPERIMENTS = ("step_response", "triangle", "duty_ladder", "dither",
                      "reversal", "impulse_ladder", "breakaway_grid")


def collect_motion_rows(runs: list[Run], v_min: float = 30.0, fc: float = 40.0,
                        ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, int]]:
    """(u, v, a, p) samples where the slider slides. Whole-run filtering.

    goto rows are excluded (repositioning, but same physics -- excluded only
    to keep provenance clean; flip include_goto if coverage is needed).
    """
    from .segments import run_position

    U, V, A, P = [], [], [], []
    prov: dict[str, int] = {}
    for run in runs:
        if run.experiment not in MOTION_EXPERIMENTS:
            continue
        pos = run_position(run)
        if len(pos) < 100:
            continue
        duty = run.df["duty_milli"].to_numpy(dtype=float) / 1000.0
        goto = run.df["goto_active"].to_numpy(dtype=bool)
        _, v = zero_phase_velocity(pos, fc=fc)
        a = np.gradient(v) * FS_HZ
        mask = (np.abs(v) > v_min) & ~goto & edge_mask(len(pos), fc=fc)
        n = int(mask.sum())
        if n == 0:
            continue
        prov[run.name] = n
        U.append(duty[mask])
        V.append(v[mask])
        A.append(a[mask])
        P.append(pos[mask])
    if not U:
        return (np.array([]),) * 4 + ({},)
    return (np.concatenate(U), np.concatenate(V), np.concatenate(A),
            np.concatenate(P), prov)


def _node_weights(p: np.ndarray, nodes: np.ndarray) -> np.ndarray:
    """(n_rows, n_nodes) linear interpolation weight matrix."""
    W = np.zeros((len(p), len(nodes)))
    idx = np.clip(np.searchsorted(nodes, p) - 1, 0, len(nodes) - 2)
    x0 = nodes[idx]
    x1 = nodes[idx + 1]
    w1 = np.clip((p - x0) / (x1 - x0), 0.0, 1.0)
    rows = np.arange(len(p))
    W[rows, idx] = 1.0 - w1
    W[rows, idx + 1] = w1
    return W


def fit_kinetic(runs: list[Run], nodes: np.ndarray | None = None,
                v_min: float = 30.0, fc: float = 40.0,
                min_support: float = 30.0,
                v_strib_grid: tuple = (None,)) -> KineticFit:
    """Linear LSQ: u = m*a + b*v + s*[Fk(p,dir) + g_dir*exp(-|v|/v_s)].

    The optional Stribeck excess g*exp(-|v|/v_s) is OFF by default: on the
    2026-07-16/17 data it fits NEGATIVE (friction apparently rising with
    speed), which is an artifact -- low |v| co-occurs with breakaway/stop
    transients where the 40 Hz zero-phase acceleration is systematically
    underestimated, so the g column soaks up that bias instead of physics.
    The low-speed friction excess that the re-stick events prove is carried
    by the static map + the position-dependent Fk nodes instead. Pass e.g.
    ``v_strib_grid=(60., 100., None)`` to re-enable the grid search once
    less transient-dominated low-speed data (M7/M8) exists.
    Everything stays linear -> honest covariance.
    """
    if nodes is None:
        nodes = np.arange(0.0, 100.0 + 1e-9, 10.0)
    u, v, a, p, prov = collect_motion_rows(runs, v_min=v_min, fc=fc)
    if len(u) < 100:
        raise ValueError(f"only {len(u)} sliding samples -- not enough for the kinetic fit")

    s = np.sign(v)
    W = _node_weights(p, nodes)
    Wf = W * ((s > 0)[:, None])            # + fk for fwd rows
    Wr = -W * ((s < 0)[:, None])           # - fk for rev rows (u = ... + s*Fk)
    nn = len(nodes)

    def solve(v_s):
        cols = [a[:, None], v[:, None], Wf, Wr]
        if v_s is not None:
            strib = np.exp(-np.abs(v) / v_s)
            cols += [(s * strib * (s > 0))[:, None], (s * strib * (s < 0))[:, None]]
        X = np.column_stack(cols)
        support = np.abs(X).sum(axis=0)
        keep = np.ones(X.shape[1], dtype=bool)
        keep[2:2 + 2 * nn] = support[2:2 + 2 * nn] >= min_support
        Xk = X[:, keep]
        theta, _, rank, _ = np.linalg.lstsq(Xk, u, rcond=None)
        resid = u - Xk @ theta
        return X, Xk, keep, theta, resid, rank

    best = None
    for v_s in v_strib_grid:
        X, Xk, keep, theta, resid, rank = solve(v_s)
        rms = float(np.sqrt(np.mean(resid**2)))
        if best is None or rms < best[0] - 1e-6:
            best = (rms, v_s, X, Xk, keep, theta, resid, rank)
    rms, v_s, X, Xk, keep, theta, resid, rank = best

    dof = max(len(u) - Xk.shape[1], 1)
    sigma2 = float(resid @ resid) / dof
    cov = sigma2 * np.linalg.pinv(Xk.T @ Xk)
    se = np.sqrt(np.diag(cov))

    full = np.full(X.shape[1], np.nan)
    full_se = np.full(X.shape[1], np.nan)
    full[keep] = theta
    full_se[keep] = se

    warnings = []
    corr_mb = float(cov[0, 1] / np.sqrt(cov[0, 0] * cov[1, 1]))
    if rank < Xk.shape[1]:
        warnings.append(f"design matrix rank-deficient ({rank}/{Xk.shape[1]})")
    if abs(corr_mb) > 0.9:
        warnings.append(f"m and b highly correlated (r={corr_mb:.2f}) -- "
                        "few independent transients; treat m/b split with care")
    m, b = float(full[0]), float(full[1])
    if m <= 0:
        warnings.append("fitted m <= 0: transients unresolved at this filter "
                        "bandwidth; inertia not identifiable from this data")
    has_strib = v_s is not None
    g_fwd = float(full[2 + 2 * nn]) if has_strib else 0.0
    g_rev = float(full[3 + 2 * nn]) if has_strib else 0.0
    if has_strib and (g_fwd < 0 or g_rev < 0):
        warnings.append(f"negative Stribeck excess fitted (g_fwd={g_fwd:.2f}, "
                        f"g_rev={g_rev:.2f}): friction RISES with speed there; "
                        "interpretation overlaps with b")
    fit = KineticFit(
        nodes=nodes,
        fk_fwd=full[2:2 + nn], fk_fwd_se=full_se[2:2 + nn],
        fk_rev=full[2 + nn:2 + 2 * nn], fk_rev_se=full_se[2 + nn:2 + 2 * nn],
        m=m, m_se=float(full_se[0]),
        b=b, b_se=float(full_se[1]),
        g_fwd=g_fwd, g_fwd_se=float(full_se[2 + 2 * nn]) if has_strib else 0.0,
        g_rev=g_rev, g_rev_se=float(full_se[3 + 2 * nn]) if has_strib else 0.0,
        v_strib=float(v_s) if has_strib else 0.0,
        corr_mb=corr_mb,
        resid_rms_duty=rms,
        n_rows=len(u), rows_by_run=prov,
        v_min=v_min, fc_hz=fc, warnings=warnings,
    )
    return fit
