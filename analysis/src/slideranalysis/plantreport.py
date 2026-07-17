"""plantreport -- one-shot Phase-3 fitting pipeline over a session directory.

    cd analysis && uv run plantreport ../data/2026-07-16 --out ../docs/findings

Re-runnable: point it at the directory again when new CSVs (M4 grid, M6
reversals, M7 impulse ladder) land -- every fit rescans the directory and
ingests what qualifies. Emits:

- plant-params.json        (PlantParams for the simulator/observer)
- sensor_noise.png, friction_static_map.png, friction_kinetic_map.png
- overlay_<run>.png        (sim vs recorded position, same duty trace)
- observer_<run>.png       (KF velocity vs zero-phase reference + legacy PT1)
- stdout: fit tables incl. uncertainties, degeneracy warnings, overlay and
  observer scores.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from . import backlash, frictionfit, impulses, plots, sensorfit
from .observer import ObserverConfig, score
from .plantsim import PlantParams, overlay_arrays, refine_inertia, simulate
from .segments import Run, load_dirs, run_position, split_segments

# Detector-characterization runs (campaign notes 2026-07-16/17): the firmware
# breakaway detector false-triggering on ADC noise was measured on purpose.
# Junk for friction by design; still perfectly good for the SENSOR fit.
FRICTION_EXCLUDE = ("58_breakaway_grid", "01_breakaway_grid")


def _slice_run(run: Run) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(t, duty, pos) from the first non-goto tick (start at rest)."""
    goto = run.df["goto_active"].to_numpy(dtype=bool)
    i0 = int(np.argmax(~goto)) if goto.any() else 0
    t = run.df["t_ms"].to_numpy(dtype=float)[i0:] / 1000.0
    duty = run.df["duty_milli"].to_numpy(dtype=float)[i0:] / 1000.0
    pos = run_position(run)[i0:]
    return t - t[0], duty, pos


def _is_transient_case(run: Run) -> bool:
    if run.experiment == "step_response" and abs(run.params.get("duty", 0)) >= 0.65:
        return True
    if run.experiment == "dither" and abs(run.params.get("bias", 0)) >= 0.7:
        _, _, pos = _slice_run(run)
        return float(np.ptp(pos)) > 10.0
    return False


def _overlay_targets(runs: list[Run]) -> list[Run]:
    out = []
    for r in runs:
        if r.experiment == "step_response" and abs(r.params.get("duty", 0)) >= 0.55:
            out.append(r)
        elif r.experiment == "triangle" and r.params.get("peak", 0) >= 0.6:
            out.append(r)
        elif r.experiment in ("duty_ladder", "impulse_ladder"):
            out.append(r)
        elif r.experiment == "dither" and abs(r.params.get("bias", 0)) >= 0.7:
            _, _, pos = _slice_run(r)
            if float(np.ptp(pos)) > 10.0:
                out.append(r)
    return out


def _observer_targets(runs: list[Run]) -> list[Run]:
    names = {"47_step_response", "49_step_response", "52_step_response",
             "54_triangle", "45_step_response"}
    picked = [r for r in runs if r.basename in names
              and (r.experiment == "triangle"
                   or (r.experiment == "step_response"
                       and abs(r.params.get("duty", 0)) >= 0.55))]
    if picked:
        return picked
    return [r for r in runs if r.experiment == "step_response"
            and abs(r.params.get("duty", 0)) >= 0.65][:3]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("datadirs", type=Path, nargs="+",
                    help="session director(y|ies) with lab CSVs")
    ap.add_argument("--out", type=Path, default=Path("../docs/findings"),
                    help="output directory for plots + plant-params.json")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--no-refine", action="store_true",
                    help="skip the simulation-based inertia refinement")
    ap.add_argument("--fc", type=float, default=40.0,
                    help="zero-phase velocity cutoff [Hz]")
    args = ap.parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")

    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    runs = load_dirs(args.datadirs)
    fric_runs = [r for r in runs if r.basename not in FRICTION_EXCLUDE]
    print(f"loaded {len(runs)} runs from {[str(d) for d in args.datadirs]}")
    print(f"friction fits exclude {len(runs) - len(fric_runs)} "
          f"detector-characterization runs: {FRICTION_EXCLUDE}")

    # ---------------------------------------------------------- sensor ----
    sensor, npoints = sensorfit.fit_sensor(runs)
    print("\n== SENSOR MODEL (adc_raw counts; 1 count = "
          f"{1/sensor.counts_per_percent:.4f} % travel) ==")
    print(f"  sigma0 (unpowered)     : {sensor.sigma0_counts:.2f} counts "
          f"[spread {sensor.sigma0_ci[0]:.2f}..{sensor.sigma0_ci[1]:.2f}] "
          f"= {sensor.sigma0_counts/sensor.counts_per_percent:.3f} %")
    if sensor.sigma0_sample_counts:
        print(f"  per raw sample         : {sensor.sigma0_sample_counts:.2f} counts; "
              f"intra-tick sigma {sensor.intra_tick_sigma_counts:.2f} "
              "(white: mean-of-4 = sample/2 holds)")
    if sensor.coupling_c_counts2 is not None:
        print(f"  coupling sigma^2 slope : c = {sensor.coupling_c_counts2:.1f} "
              f"counts^2/duty, CI {sensor.coupling_c_ci}")
    print(f"  pooled sigma under PWM : {sensor.sigma_pwm_counts:.2f} counts"
          if sensor.sigma_pwm_counts else "  (no stalled-PWM segments seen)")
    print(f"  p-p unpowered / PWM    : {sensor.ptp0_counts:.0f} / "
          f"{sensor.ptp_pwm_counts:.0f} counts (spike tail!)")
    print(f"  quantization           : 1 LSB raw, 0.25 counts adc_raw -- "
          "negligible vs sigma0")
    for n in sensor.notes:
        print(f"  NOTE: {n}")
    mrows = sensorfit.mode_table(npoints)
    if any(r[0] == "slow" for r in mrows):
        print("  coupling by decay mode (median over stalled segments):")
        print(f"    {'mode':>6s} {'duty':>5s} {'n':>3s} {'pos':>5s} {'sigma':>6s} {'p-p':>6s}")
        for mode, d, n, pos, sig, ptp in mrows:
            print(f"    {mode:>6s} {d:5.2f} {n:3d} {pos:5.0f} {sig:6.1f} {ptp:6.0f}")
        pairs, ptp_ratio, sig_ratio = sensorfit.mode_comparison(npoints)
        if pairs:
            print(f"    -> SLOW/FAST on {len(pairs)} duty+position-matched pairs: "
                  f"p-p x{ptp_ratio:.2f}, sigma x{sig_ratio:.2f} "
                  "(recommendation: FAST decay -- couples less)")
    if not args.no_plots:
        plots.plot_sensor(npoints, sensor, out / "sensor_noise.png")

    # ---------------------------------------------------------- static ----
    ev, invalid = frictionfit.collect_static_evidence(fric_runs)
    smap = frictionfit.fit_static_map(ev, invalid_trials=invalid)
    print(f"\n== STATIC (BREAKAWAY) MAP == {len(ev)} evidence items, "
          f"{invalid} INVALID breakaway_grid trials (noise-tripped detector)")
    print("  pos:  " + " ".join(f"{p:5.0f}" for p in smap.grid[::2]))
    for name, lb, ub in (("fwd", smap.lb_fwd, smap.ub_fwd),
                         ("rev", smap.lb_rev, smap.ub_rev)):
        fmt = lambda a: " ".join("  ---" if np.isnan(x) else f"{x:5.2f}" for x in a[::2])
        print(f"  {name} lb: {fmt(lb)}")
        print(f"  {name} ub: {fmt(ub)}")
    for c in smap.conflicts:
        print(f"  CONFLICT: {c}")
    if not args.no_plots:
        plots.plot_static_map(smap, out / "friction_static_map.png")

    # --------------------------------------------------------- kinetic ----
    kfit = frictionfit.fit_kinetic(fric_runs, fc=args.fc)
    print(f"\n== KINETIC FIT == {kfit.n_rows} sliding samples "
          f"(|v|>{kfit.v_min:.0f} %/s) from {len(kfit.rows_by_run)} runs")
    print(f"  m = {kfit.m:.3e} ± {2*kfit.m_se:.1e} duty/(%/s^2)  "
          f"[2-sigma; SYSTEMATIC bias from {kfit.fc_hz:.0f} Hz smoothing]")
    print(f"  b = {kfit.b:.3e} ± {2*kfit.b_se:.1e} duty/(%/s)   corr(m,b)={kfit.corr_mb:+.2f}")
    if kfit.v_strib:
        print(f"  Stribeck excess: g_fwd = {kfit.g_fwd:.3f} ± {2*kfit.g_fwd_se:.3f}, "
              f"g_rev = {kfit.g_rev:.3f} ± {2*kfit.g_rev_se:.3f}, "
              f"v_s = {kfit.v_strib:.0f} %/s (grid-selected)")
    else:
        print("  Stribeck excess: not selected (no residual improvement)")
    print(f"  residual RMS = {kfit.resid_rms_duty:.3f} duty (model error incl. "
          "stick-slip texture)")
    print("  node:   " + " ".join(f"{p:5.0f}" for p in kfit.nodes))
    for name, f, se in (("fk_fwd", kfit.fk_fwd, kfit.fk_fwd_se),
                        ("fk_rev", kfit.fk_rev, kfit.fk_rev_se)):
        print(f"  {name}: " + " ".join(
            "  ---" if np.isnan(x) else f"{x:5.2f}" for x in f))
        print(f"    ±2σ : " + " ".join(
            "     " if np.isnan(x) else f"{2*s:5.2f}" for x, s in zip(f, se)))
    for w in kfit.warnings:
        print(f"  WARN: {w}")
    if not args.no_plots:
        plots.plot_kinetic_map(kfit, smap, out / "friction_kinetic_map.png")

    # ------------------------------------------------------------ plant ---
    plant = PlantParams.from_fits(sensor, smap, kfit,
                                  meta={"source": [str(d) for d in args.datadirs],
                                        "resid_rms_duty": kfit.resid_rms_duty,
                                        "m_lsq": kfit.m,
                                        "warnings": kfit.warnings})

    if not args.no_refine and plant.m > 0:
        cases = []
        for r in fric_runs:
            if _is_transient_case(r):
                _, duty, pos = _slice_run(r)
                cases.append((duty, pos))
        if cases:
            m_ref, factors, curve = refine_inertia(
                plant, cases, factors=np.geomspace(0.03, 8.0, 29))
            flat = curve.min() / max(curve[0], curve[-1])
            print(f"\n== INERTIA REFINEMENT (sim vs raw position, {len(cases)} "
                  f"transients) ==")
            if m_ref <= plant.m * factors[0] * 1.01 or m_ref >= plant.m * factors[-1] * 0.99:
                print("  WARN: refinement hit the factor-grid edge -- m is only "
                      "bounded, not located, by these transients")
            print(f"  m_lsq = {kfit.m:.3e} -> m_refined = {m_ref:.3e} "
                  f"(factor {m_ref/kfit.m:.2f})")
            print(f"  objective flatness min/edge = {flat:.2f} "
                  "(closer to 1 = m weakly identified)")
            near = factors[curve <= curve.min() * 1.2]
            m_lo, m_hi = kfit.m * near.min(), kfit.m * near.max()
            print(f"  m range (objective within 20 % of min): "
                  f"[{m_lo:.1e}, {m_hi:.1e}]")
            plant.meta["m_refined"] = m_ref
            plant.meta["m_range_20pct"] = [float(m_lo), float(m_hi)]
            plant.meta["m_objective_flatness"] = float(flat)
            plant.m = m_ref

    # physical seam from the M3 fit (m3fit writes motor-params.json); keep it
    # attached across regenerations -- controller code never reads it, it is
    # the K/R bridge for when the model moves to physical units
    mp = out / "motor-params.json"
    if mp.exists():
        import json as _json
        plant.meta["physical"] = _json.loads(mp.read_text(encoding="utf-8"))

    pj = out / "plant-params.json"
    plant.to_json(pj)
    print(f"\nplant params -> {pj}")

    # ------------------------------------------------- M7 impulse table ---
    events = impulses.collect_impulses(fric_runs)
    if events:
        print(f"\n== M7 IMPULSE TABLE (measured, settle-to-settle; duty 0.8) ==")
        print(f"  {'dir':>4s} {'width':>6s} {'n':>3s} {'travel med':>10s} "
              f"{'range':>16s} {'sim @same p0':>12s}")
        for direction, w, n, med, lo, hi in impulses.impulse_table(events):
            evs = [e for e in events if e.width_ms == w
                   and np.sign(e.duty) == direction and not e.cut]
            sims = []
            for e in evs:
                duty = np.concatenate([np.full(e.width_ms, e.duty), np.zeros(500)])
                r = simulate(plant, duty, p0=e.pos_start, endzone=True)
                sims.append(r.pos[-1] - e.pos_start)
            sim_med = float(np.median(sims)) if sims else float("nan")
            print(f"  {'fwd' if direction > 0 else 'rev':>4s} {w:4d}ms {n:3d} "
                  f"{med:+9.2f}% {lo:+7.2f}..{hi:+7.2f}% {sim_med:+11.2f}%")
        ncut = sum(1 for e in events if e.cut)
        if ncut:
            print(f"  ({ncut} pulses dropped: end-zone interference)")

    # ------------------------------------------------------ M6 backlash ---
    trials = backlash.collect_reversal_trials(fric_runs)
    if trials:
        pairs = backlash.backlash_pairs(trials)
        print(f"\n== M6 BACKLASH (reversal vs same-direction control dead time) ==")
        for pos, d, tr, tc, delta in pairs:
            print(f"  pos {pos:5.1f} % duty {d:.2f}: reversal {tr} ms, "
                  f"control {tc} ms -> delta {delta:+d} ms")
        stuck_trials = [t for t in trials if not t.moved]
        for t in stuck_trials:
            print(f"  NOTE {t.run}: test drive {t.rev_dir:+d}x{t.duty:.2f} at "
                  f"{t.pos:.1f} % never moved (breakaway, not backlash)")
        if pairs:
            deltas = [p[4] for p in pairs]
            print(f"  verdict: delta {min(deltas):+d}..{max(deltas):+d} ms at 1 kHz "
                  "resolution -> no resolvable dead band; sim backlash stub = 0 stands")

    # --------------------------------------------------------- overlays ---
    print("\n== SIM VALIDATION OVERLAYS (recorded duty -> sim vs measured pos) ==")
    print(f"  {'run':28s} {'RMSE%':>6s} {'final%':>7s} {'max%':>6s}")
    overlay_rows = []
    for r in _overlay_targets(fric_runs):
        t, duty, pos = _slice_run(r)
        sim, res = overlay_arrays(plant, duty, pos, run=r.name)
        overlay_rows.append(res)
        print(f"  {r.name:28s} {res.rmse_pct:6.2f} {res.final_err_pct:+7.2f} "
              f"{res.max_err_pct:6.2f}")
        if not args.no_plots:
            safe = r.name.replace("/", "_")
            plots.plot_overlay(t, pos, sim, duty, res,
                               out / f"overlay_{safe}.png",
                               title=f"{r.name} ({r.experiment} "
                                     f"{r.params.get('duty', r.params.get('bias', r.params.get('peak', '')))})")

    # --------------------------------------------------------- observer ---
    cfg = ObserverConfig.from_plant(plant, model_err_duty=kfit.resid_rms_duty)
    print("\n== OBSERVER (2-state KF, duty-driven) vs zero-phase reference ==")
    print(f"  {'run':28s} {'RMS mov':>8s} {'lag':>6s} | legacy PT1: "
          f"{'RMS mov':>8s} {'lag':>6s}")
    for r in _observer_targets(runs):
        t, duty, pos = _slice_run(r)
        sc, detail = score(r.name, duty, pos, plant, cfg, fc_ref=args.fc)
        print(f"  {r.name:28s} {sc.rms_moving:7.1f}  {sc.lag_ms:+5.0f}ms | "
              f"{sc.rms_moving_legacy:12.1f}  {sc.lag_ms_legacy:+5.0f}ms")
        if not args.no_plots:
            safe = r.name.replace("/", "_")
            plots.plot_observer(t, detail, sc, out / f"observer_{safe}.png",
                                title=f"{r.name}: KF vs offline reference")

    return 0


if __name__ == "__main__":
    sys.exit(main())
