"""M3 current analysis: motor R, torque/EMF constant K, physical-unit seam.

Data: scope npz (Ch A wiper volts, Ch B shunt volts, 0.2 R in the L9110
module's GND return) paired with duty_ladder CSVs (npz NN <-> CSV NN+1),
2026-07-17 Block M3. VM = 5.00 V.

CIRCUIT MODEL (why the stall line is curved): the shunt sits in the module
GND return. During the PWM ON phase the motor current flows supply->bridge->
motor->bridge->shunt; during the OFF phase (both L9110 modes are brake-type)
it recirculates INSIDE the bridge and largely bypasses the shunt. The shunt
mean is therefore ~ d * I_motor, and with conduction continuous (high duty):

    I_shunt(d) = d * (d*VM - V_eff) / R + c

linear in (VM/R, V_eff/R, c) -> honest covariance. V_eff lumps the bridge
saturation drops (L9110 is a bipolar darlington bridge, ~1.5 V total is
normal), c the logic/base-drive bias. BELOW ~0.4 duty the measured line goes
flat-ish (55 mA at 0.1 where the CCM model would predict ~zero): conduction
is discontinuous / recirculation losses dominate -- R is deliberately NOT
fitted there. Validity range of the R fit: duty >= 0.5.

K (back-EMF/torque constant, expressed per % travel: V/(%/s)) from the
within-rung flight-vs-stop split of the moving rungs: at fixed duty,
I_shunt(t) = I_stall(d) - (d*K/R) * v(t) -> regression of I against wiper
velocity. Force conversion: 1 duty of force corresponds to the extrapolated
full-stall motor current I1 = (VM - V_eff)/R; friction maps in mA are
F_mA = F_duty * I1.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .loadrun import load_run

VM = 5.0
SHUNT_OHM = 0.2
FS_SCOPE = 250_000.0
V_PER_PCT = 0.0275          # 2.75 V wiper span over 100 % travel
R_FIT_DMIN = 0.5            # validity floor of the CCM model


@dataclass
class RungMeas:
    source: str              # npz stem
    mode: str                # fast | slow
    direction: int           # +1 fwd, -1 rev
    pos_label: str           # nominal position tag from the run plan
    duty: float
    i_mean_ma: float         # over the analysis window
    kind: str                # "stall" | "flight" | "atstop"
    v_mean: float = 0.0      # %/s (flight only)
    n_s: float = 0.0         # seconds of data


@dataclass
class RFit:
    mode: str
    r_ohm: float
    r_se: float
    v_eff: float
    v_eff_se: float
    c_ma: float
    c_se: float
    resid_rms_ma: float
    n_points: int
    dmin: float

    @property
    def i1_ma(self) -> float:
        """Extrapolated full-stall motor current (force scale), mA."""
        return (VM - self.v_eff) / self.r_ohm * 1000.0


@dataclass
class KFit:
    k_v_per_pct_s: float
    k_se: float
    per_run: dict = field(default_factory=dict)
    n_samples: int = 0


# --------------------------------------------------------------- loading ---

# npz stem -> (csv stem, position label). From the campaign notes (Block M3).
LADDER_PAIRS = {
    "99_scope_ladder": ("100_duty_ladder", "20fwd"),
    "101_scope_ladder": ("102_duty_ladder", "20fwd"),
    "104_scope_ladder": ("105_duty_ladder", "20fwd"),
    "107_scope_ladder": ("108_duty_ladder", "20fwd"),
    "110_scope_ladder": ("111_duty_ladder", "80rev"),
    "112_scope_ladder": ("113_duty_ladder", "80rev"),
    "114_scope_ladder": ("115_duty_ladder", "80rev"),
    "117_scope_ladder": ("118_duty_ladder", "80rev"),
    "120_scope_ladder": ("121_duty_ladder", "50fwd"),
    "122_scope_ladder": ("123_duty_ladder", "50fwd"),
    "124_scope_ladder": ("125_duty_ladder", "50rev"),
    "126_scope_ladder": ("127_duty_ladder", "50rev"),
    "129_scope_ladder": ("130_duty_ladder", "65rev"),
    "131_scope_ladder": ("132_duty_ladder", "65rev"),
}


def _smooth(x: np.ndarray, n: int) -> np.ndarray:
    if n <= 1:
        return x
    return np.convolve(x, np.ones(n) / n, mode="same")


def _to_1k(d) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Block-average the 250 kS/s capture to a 1 kHz grid.

    Averaging first kills both the PWM chop on the current channel and the
    sampling noise on the wiper -- differentiating at scope rate would drown
    velocity in noise. Returns (i_ma, pos_pct_rel, vel_pct_s) at 1 kHz.
    """
    dec = int(FS_SCOPE / 1000.0)
    i = d["volts_isen_0r2"] / SHUNT_OHM * 1000.0
    w = -d["volts_wiper"] / V_PER_PCT
    n = len(i) // dec
    i_1k = i[: n * dec].reshape(n, dec).mean(axis=1)
    pos_1k = w[: n * dec].reshape(n, dec).mean(axis=1)
    vel_1k = np.gradient(_smooth(pos_1k, 5)) * 1000.0
    return i_1k, pos_1k, vel_1k


def _rung_windows(i_1k: np.ndarray, n_expected: int):
    """Contiguous drive windows (1 kHz grid) from the current envelope."""
    sm = _smooth(i_1k, 11)
    on = sm > 20.0
    edges = np.diff(on.astype(int))
    starts = np.flatnonzero(edges == 1) + 1
    ends = np.flatnonzero(edges == -1) + 1
    wins = [(s, e) for s, e in zip(starts, ends) if (e - s) > 300]
    if len(wins) != n_expected:
        raise ValueError(f"expected {n_expected} rungs, found {len(wins)}")
    return wins


def extract_rungs(datadir: Path, npz_stem: str, csv_stem: str,
                  pos_label: str) -> list[RungMeas]:
    d = np.load(datadir / f"{npz_stem}.npz", allow_pickle=True)
    meta, _df = load_run(datadir / f"{csv_stem}.csv")
    p = meta["params"]
    duties = list(np.round(np.arange(p["d0"], p["d1"] + 1e-9, p["step"]), 3))
    mode = p.get("mode", "fast")
    direction = 1 if p.get("dir", "fwd") == "fwd" else -1

    i_1k, _pos, vel = _to_1k(d)
    wins = _rung_windows(i_1k, len(duties))
    out: list[RungMeas] = []
    for (s, e), duty in zip(wins, duties):
        s2, e2 = s + 50, e - 20
        seg_i = i_1k[s2:e2]
        seg_v = vel[s2:e2]
        moving = np.abs(seg_v) > 30.0
        if float(np.mean(moving)) < 0.05:
            out.append(RungMeas(npz_stem, mode, direction, pos_label, duty,
                                float(seg_i.mean()), "stall",
                                n_s=len(seg_i) / 1000.0))
            continue
        if moving.sum() > 50:
            out.append(RungMeas(npz_stem, mode, direction, pos_label, duty,
                                float(seg_i[moving].mean()), "flight",
                                v_mean=float(np.abs(seg_v[moving]).mean()),
                                n_s=moving.sum() / 1000.0))
        # trailing at-stop part (carriage on the end stop = stall data)
        still = ~moving
        idx = np.flatnonzero(np.diff(still.astype(int)) == 1)
        j0 = int(idx[-1]) + 1 if len(idx) else (0 if still[0] else len(still))
        tail = seg_i[j0:][still[j0:]]
        if len(tail) > 100:
            out.append(RungMeas(npz_stem, mode, direction, pos_label, duty,
                                float(tail.mean()), "atstop",
                                n_s=len(tail) / 1000.0))
    return out


def collect_all(datadir: str | Path) -> list[RungMeas]:
    datadir = Path(datadir)
    rows: list[RungMeas] = []
    for npz_stem, (csv_stem, pos_label) in LADDER_PAIRS.items():
        if not (datadir / f"{npz_stem}.npz").exists():
            continue
        rows.extend(extract_rungs(datadir, npz_stem, csv_stem, pos_label))
    return rows


# ------------------------------------------------------------------ fits ---

def fit_r(rows: list[RungMeas], mode: str, dmin: float = R_FIT_DMIN) -> RFit:
    """CCM model I = d(d*VM - V_eff)/R + c on stall/at-stop points, d >= dmin.

    Linear LSQ in (a2, a1, c) with I = a2*d^2 - a1*d + c; R = VM/a2,
    V_eff = a1/a2 * VM ... V_eff = a1*R. SEs by the delta method.
    """
    pts = [r for r in rows if r.mode == mode and r.kind in ("stall", "atstop")
           and r.duty >= dmin - 1e-9]
    if len(pts) < 4:
        raise ValueError(f"only {len(pts)} points for R fit ({mode})")
    d = np.array([p.duty for p in pts])
    i = np.array([p.i_mean_ma for p in pts]) / 1000.0  # A
    X = np.column_stack([d**2, -d, np.ones_like(d)])
    theta, *_ = np.linalg.lstsq(X, i, rcond=None)
    resid = i - X @ theta
    dof = max(len(i) - 3, 1)
    cov = float(resid @ resid) / dof * np.linalg.pinv(X.T @ X)
    a2, a1, c = theta
    se = np.sqrt(np.diag(cov))
    r_ohm = VM / a2
    r_se = VM / a2**2 * se[0]
    v_eff = a1 / a2 * VM
    # delta method for v_eff = VM*a1/a2
    g = np.array([-VM * a1 / a2**2, VM / a2, 0.0])
    v_se = float(np.sqrt(g @ cov @ g))
    return RFit(mode=mode, r_ohm=float(r_ohm), r_se=float(r_se),
                v_eff=float(v_eff), v_eff_se=v_se,
                c_ma=float(c * 1000), c_se=float(se[2] * 1000),
                resid_rms_ma=float(np.sqrt(np.mean(resid**2)) * 1000),
                n_points=len(pts), dmin=dmin)


def fit_k(datadir: str | Path, rfits: dict[str, RFit]) -> KFit:
    """K from within-rung I-vs-v regression over the flight samples of the
    moving ladders (runs 104/107/114/117): slope = -d*K/R."""
    datadir = Path(datadir)
    ks, ws, per_run = [], [], {}
    n_tot = 0
    for npz_stem, (csv_stem, pos_label) in LADDER_PAIRS.items():
        path = datadir / f"{npz_stem}.npz"
        if not path.exists():
            continue
        meta, _ = load_run(datadir / f"{csv_stem}.csv")
        p = meta["params"]
        if p["d0"] < 0.65:      # stall ladders: no flight
            continue
        mode = p.get("mode", "fast")
        r = rfits[mode].r_ohm
        d = np.load(path, allow_pickle=True)
        i_1k, _pos, vel = _to_1k(d)
        duties = list(np.round(np.arange(p["d0"], p["d1"] + 1e-9, p["step"]), 3))
        wins = _rung_windows(i_1k, len(duties))
        run_ks = []
        for (s, e), duty in zip(wins, duties):
            s2 = s + 30
            seg_i = i_1k[s2:e]
            seg_v = np.abs(vel[s2:e])
            m = seg_v > 40.0
            if m.sum() < 80:
                continue
            iv, vv = seg_i[m], seg_v[m]
            if np.ptp(vv) < 40.0:   # no leverage: skip
                continue
            A = np.column_stack([vv, np.ones_like(vv)])
            (slope, _b), *_ = np.linalg.lstsq(A, iv, rcond=None)
            k = -slope / 1000.0 * r / duty          # V/(%/s)
            n_eff = m.sum() / 10.0                  # ~independent samples
            run_ks.append(k)
            ks.append(k)
            ws.append(n_eff)
            n_tot += int(m.sum())
        if run_ks:
            per_run[npz_stem] = [round(float(k), 6) for k in run_ks]
    ks = np.array(ks)
    ws = np.array(ws)
    k_mean = float(np.average(ks, weights=ws))
    k_se = float(np.sqrt(np.average((ks - k_mean) ** 2, weights=ws)
                         / max(len(ks) - 1, 1)))
    return KFit(k_v_per_pct_s=k_mean, k_se=k_se, per_run=per_run,
                n_samples=n_tot)


def position_spread(rows: list[RungMeas]) -> float:
    """Max relative spread of stall current across positions, per (mode,
    duty), over the 0.1-0.6 table -- Jan's position-independence number."""
    worst = 0.0
    for mode in ("fast", "slow"):
        for duty in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6):
            grp = [r.i_mean_ma for r in rows
                   if r.mode == mode and r.kind == "stall"
                   and abs(r.duty - duty) < 1e-6]
            if len(grp) >= 3:
                spread = (max(grp) - min(grp)) / 2.0 / np.mean(grp)
                worst = max(worst, spread)
    return float(worst)


# ------------------------------------------------------- RC filter check ---

def rc_spike_suppression(npz_path: str | Path, r_src: float = 1500.0,
                         c_farad: float = 100e-9,
                         skip_s: float = 0.25) -> dict:
    """Apply a first-order RC digitally to the measured unclipped wiper
    spike capture (10 MS/s); report raw-trace and ADC-EMULATED statistics.

    The first ``skip_s`` is dropped: the scope channel is AC-coupled and its
    settle transient after drive-on (tau ~150 ms, initial ~+130 mV) is a
    capture artifact -- including it overstated the "low-frequency tail" and
    nearly hid the RC's benefit on first analysis. The genuine wiper
    disturbance is ~0.1-0.3 us switching bursts (~380 kHz ringing, up to
    ~0.2 V): the measured ADC spike tail (p-p 140-260 counts under PWM) is
    ADC samples landing inside those bursts (~5 %/tick odds), and the RC
    removes exactly that.

    ADC emulation: 4 samples 2 us apart per 1 ms tick, averaged (firmware
    scheme); counts at 0.348 mV/count (13-bit over the 2.85 V rail).
    """
    d = np.load(npz_path, allow_pickle=True)
    meta = json.loads(str(d["meta"]))
    fs = float(meta["rate_hz"])
    x = d["volts_wiper_ac"][int(skip_s * fs):]
    x = x - np.median(x)
    tau = r_src * c_farad
    alpha = (1.0 / fs) / (tau + 1.0 / fs)
    from scipy.signal import lfilter
    y = lfilter([alpha], [1, -(1 - alpha)], x)

    def stats(z):
        a = np.abs(z)
        return dict(max_mv=float(a.max() * 1000),
                    p999_mv=float(np.percentile(a, 99.9) * 1000),
                    rms_mv=float(np.sqrt(np.mean(z**2)) * 1000))

    def tick_stats(z):
        n_ticks = int(len(z) / fs * 1000) - 1
        idx = ((np.arange(n_ticks) * fs / 1000).astype(int)[:, None]
               + (np.arange(4) * int(2e-6 * fs))[None, :])
        t = z[idx].mean(axis=1) * 1000.0   # mV per tick
        t = t - np.median(t)
        return dict(rms_mv=float(t.std()), ptp_mv=float(np.ptp(t)),
                    rms_counts=float(t.std() / 0.348),
                    ptp_counts=float(np.ptp(t) / 0.348))

    return dict(fs_hz=fs, tau_us=tau * 1e6,
                fc_hz=1.0 / (2 * np.pi * tau),
                pre=stats(x), post=stats(y),
                adc_pre=tick_stats(x), adc_post=tick_stats(y))


# ----------------------------------------------------------------- CLI ----

def main(argv=None) -> int:
    """m3fit: R/K from the M3 scope+CSV ladders; writes motor-params.json,
    merges the physical seam into plant-params.json, emits plots."""
    import argparse

    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument("datadir", type=Path, help="session dir with M3 npz/CSV")
    ap.add_argument("--out", type=Path, default=Path("../docs/findings"))
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")
    args.out.mkdir(parents=True, exist_ok=True)

    rows = collect_all(args.datadir)
    print(f"{len(rows)} rung measurements "
          f"({sum(r.kind == 'stall' for r in rows)} stall, "
          f"{sum(r.kind == 'flight' for r in rows)} flight, "
          f"{sum(r.kind == 'atstop' for r in rows)} at-stop)")

    rfits = {m: fit_r(rows, m) for m in ("fast", "slow")}
    for f in rfits.values():
        print(f"\n== R FIT ({f.mode}, CCM quadratic model, duty >= {f.dmin}) ==")
        print(f"  R      = {f.r_ohm:.2f} ± {2*f.r_se:.2f} ohm (2sigma; motor+shunt+wiring ON-path)")
        print(f"  V_eff  = {f.v_eff:.2f} ± {2*f.v_eff_se:.2f} V (bridge sat + brush; "
              "partially degenerate with c over the narrow duty window)")
        print(f"  c      = {f.c_ma:.0f} ± {2*f.c_se:.0f} mA   resid {f.resid_rms_ma:.1f} mA, n={f.n_points}")
        print(f"  I1     = {f.i1_ma:.0f} mA (extrapolated full-stall motor current, force scale)")

    kf = fit_k(args.datadir, rfits)
    r_pool = float(np.mean([f.r_ohm for f in rfits.values()]))
    k_over_r = kf.k_v_per_pct_s / r_pool
    print(f"\n== K (within-rung flight-vs-stop regression) ==")
    print(f"  K      = {kf.k_v_per_pct_s*1e3:.3f} ± {2*kf.k_se*1e3:.3f} mV/(%/s) (2sigma over rungs)")
    print(f"  K/R    = {k_over_r*1e6:.1f} uA/(%/s) per duty (regression-direct, R-independent)")
    print(f"  per-run K [mV/(%/s)]: " + ", ".join(
        f"{run.split('_')[0]}:{','.join(f'{1e3*k:.2f}' for k in ks)}"
        for run, ks in kf.per_run.items()))

    # consistency vs the mechanical b (duty units)
    i_on0 = 0.25  # A: folded ON-level at stall (see plant-model 8) ~ I1/1000
    b_e_cruise = 0.7 * k_over_r / i_on0
    b_e_brake = (kf.k_v_per_pct_s**2 / r_pool) / (i_on0 * kf.k_v_per_pct_s) if kf.k_v_per_pct_s else 0
    # brake: F = K_t*I_short, I_short = K*v/R; in duty units /(K_t*I_on0)
    b_e_brake = (kf.k_v_per_pct_s / r_pool) / i_on0
    print(f"\n== CONSISTENCY vs b = 5.5e-4 duty/(%/s) (kinetic fit) ==")
    print(f"  electro braking b_e (shorted, duty 0) ~ {b_e_brake:.1e} duty/(%/s)")
    print(f"  electro share of b ~ {b_e_brake/5.5e-4*100:.0f} % -> mechanical remainder "
          f"{(5.5e-4-b_e_brake):.1e} (must be >0: {'OK' if b_e_brake < 5.5e-4 else 'INCONSISTENT'})")

    spread = position_spread(rows)
    print(f"\n== POSITION-INDEPENDENCE (stall 0.1-0.6, all positions) ==")
    print(f"  max spread ±{spread*100:.2f} % (campaign claim ±1.5 %: "
          f"{'CONFIRMED' if spread <= 0.016 else 'EXCEEDED'})")

    rc = rc_spike_suppression(args.datadir / "139_scope_spikes_wiper.npz")
    print(f"\n== RC FILTER (measured unclipped wiper spikes, digital 1st-order RC "
          f"R={1500:.0f} ohm C=100 nF, AC-settle skipped) ==")
    print(f"  fc = {rc['fc_hz']:.0f} Hz, tau = {rc['tau_us']:.0f} us")
    print(f"  node pre : max {rc['pre']['max_mv']:.0f} mV  p99.9 {rc['pre']['p999_mv']:.1f} mV  "
          f"rms {rc['pre']['rms_mv']:.1f} mV")
    print(f"  node post: max {rc['post']['max_mv']:.1f} mV  p99.9 {rc['post']['p999_mv']:.1f} mV  "
          f"rms {rc['post']['rms_mv']:.1f} mV")
    print(f"  ADC-emulated tick (4x avg): pre rms {rc['adc_pre']['rms_counts']:.0f} counts "
          f"p-p {rc['adc_pre']['ptp_counts']:.0f}  ->  post rms {rc['adc_post']['rms_counts']:.1f} "
          f"p-p {rc['adc_post']['ptp_counts']:.0f} counts")

    # ---------------------------------------------------------- outputs ---
    motor = dict(
        vm_v=VM, shunt_ohm=SHUNT_OHM,
        r_ohm={m: round(f.r_ohm, 2) for m, f in rfits.items()},
        r_se_ohm={m: round(f.r_se, 2) for m, f in rfits.items()},
        v_eff_v={m: round(f.v_eff, 2) for m, f in rfits.items()},
        c_ma={m: round(f.c_ma, 1) for m, f in rfits.items()},
        r_fit_validity="duty >= 0.5 (CCM); model I_shunt = d(d*VM - V_eff)/R + c",
        k_v_per_pct_s=round(kf.k_v_per_pct_s, 6),
        k_se=round(kf.k_se, 6),
        k_over_r_a_per_pct_s=round(k_over_r, 8),
        i_on_stall_a=i_on0,
        i1_ma={m: round(f.i1_ma, 0) for m, f in rfits.items()},
        position_spread_frac=round(spread, 4),
        b_electro_duty_per_pct_s=round(b_e_brake, 6),
        note=("duty-unit force F corresponds to motor current F * I_on_stall; "
              "K expressed per % slider travel (V/(%/s)); shunt sees ON-phase "
              "current only (recirculation is bridge-internal)"),
    )
    mp = args.out / "motor-params.json"
    mp.write_text(json.dumps(motor, indent=2) + "\n", encoding="utf-8")
    print(f"\nmotor params -> {mp}")

    pp = args.out / "plant-params.json"
    if pp.exists():
        d = json.loads(pp.read_text(encoding="utf-8"))
        d.setdefault("meta", {})["physical"] = motor
        pp.write_text(json.dumps(d, indent=2) + "\n", encoding="utf-8")
        print(f"physical seam merged into {pp}")

    if not args.no_plots:
        _plots(rows, rfits, args.datadir, args.out, rc)
        print(f"plots -> {args.out}/m3_*.png")
    return 0


def _plots(rows, rfits, datadir, out, rc):
    import matplotlib.pyplot as plt

    from .plots import AQUA, BLUE, INK2, ORANGE, _style

    # stall lines + fits
    fig, ax = plt.subplots(figsize=(7.5, 4.4))
    for mode, color in (("fast", BLUE), ("slow", ORANGE)):
        pts = [r for r in rows if r.mode == mode and r.kind in ("stall", "atstop")]
        ax.plot([p.duty for p in pts], [p.i_mean_ma for p in pts], "o",
                ms=5, mew=0, color=color, label=f"{mode.upper()} stall (all positions)")
        f = rfits[mode]
        dd = np.linspace(f.dmin, 0.85, 50)
        ax.plot(dd, (dd * (dd * VM - f.v_eff) / f.r_ohm + f.c_ma / 1000) * 1000,
                color=color, lw=2,
                label=f"{mode.upper()} CCM fit: R={f.r_ohm:.1f}±{2*f.r_se:.1f} Ω")
    ax.axvspan(0.05, R_FIT_DMIN, color=INK2, alpha=0.08, lw=0)
    ax.annotate("DCM / recirculation region\n(R not identifiable)",
                xy=(0.27, 200), fontsize=8, color=INK2, ha="center")
    ax.legend(fontsize=8, frameon=False, loc="upper left")
    _style(ax, "M3 stall shunt current vs duty -- shunt sees d x I_motor(ON)",
           "duty", "I_shunt mean [mA]")
    fig.tight_layout()
    fig.savefig(out / "m3_stall_lines.png", dpi=130)
    plt.close(fig)

    # K regression example (104: fast fwd moving rung 0.68)
    path = Path(datadir) / "104_scope_ladder.npz"
    if path.exists():
        d = np.load(path, allow_pickle=True)
        i_1k, _pos, vel = _to_1k(d)
        wins = _rung_windows(i_1k, 3)
        s, e = wins[0]
        seg_i, seg_v = i_1k[s + 30:e], np.abs(vel[s + 30:e])
        m = seg_v > 40
        fig, ax = plt.subplots(figsize=(7, 4))
        ax.plot(seg_v[m], seg_i[m], ".", ms=3, color=BLUE, alpha=0.5,
                label="flight samples (1 kHz), rung 0.68 FAST fwd")
        A = np.column_stack([seg_v[m], np.ones(m.sum())])
        (sl, b0), *_ = np.linalg.lstsq(A, seg_i[m], rcond=None)
        vv = np.linspace(seg_v[m].min(), seg_v[m].max(), 10)
        ax.plot(vv, sl * vv + b0, color=ORANGE, lw=2,
                label=f"slope {sl:.4f} mA/(%/s) -> K/R")
        ax.legend(fontsize=8, frameon=False)
        _style(ax, "Back-EMF signature: shunt current vs slider speed at fixed duty",
               "|v| [%/s]", "I_shunt [mA]")
        fig.tight_layout()
        fig.savefig(out / "m3_k_regression.png", dpi=130)
        plt.close(fig)

    # RC filter on measured spikes (AC-settle transient skipped, as in the fit)
    dd = np.load(Path(datadir) / "139_scope_spikes_wiper.npz", allow_pickle=True)
    x = dd["volts_wiper_ac"][int(0.25 * 1e7):] * 1000
    x = x - np.median(x)
    fs = 1e7
    tau = 1500 * 100e-9
    alpha = (1 / fs) / (tau + 1 / fs)
    from scipy.signal import lfilter
    y = lfilter([alpha], [1, -(1 - alpha)], x)
    ipk = int(np.argmax(np.abs(x)))
    sl = slice(max(ipk - 3000, 0), ipk + 5000)
    t_us = np.arange(sl.start, sl.stop) / fs * 1e6
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.plot(t_us, x[sl], color=BLUE, lw=0.7, label="wiper AC, measured (10 MS/s)")
    ax.plot(t_us, y[sl], color=ORANGE, lw=1.4,
            label=f"after RC 1.5 kΩ/100 nF (fc {rc['fc_hz']:.0f} Hz): "
                  f"p99.9 {rc['pre']['p999_mv']:.0f} -> {rc['post']['p999_mv']:.0f} mV")
    ax.legend(fontsize=8, frameon=False)
    _style(ax, "PWM spike on the ADC node: worst measured event, RC applied digitally",
           "t [µs]", "wiper AC [mV]")
    fig.tight_layout()
    fig.savefig(out / "m3_rc_filter.png", dpi=130)
    plt.close(fig)
