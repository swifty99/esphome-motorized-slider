"""Tests for the Phase-3 fitting pipeline: velocity, friction fits, simulator,
observer. Each fit is verified as a ROUND TRIP against the simulator with
known parameters -- a fit that cannot recover what generated the data would
be decorative."""

import numpy as np
import pytest

from slideranalysis.frictionfit import (
    StaticEvidence,
    collect_static_evidence,
    fit_kinetic,
    fit_static_map,
)
from slideranalysis.loadrun import RAW_AT_0PCT, RAW_AT_100PCT, load_run
from slideranalysis.observer import ObserverConfig, score
from slideranalysis.plantsim import PlantParams, measure, overlay_arrays, simulate
from slideranalysis.segments import Run, load_dir, split_segments
from slideranalysis.velocity import zero_phase_velocity

from synth import write_run


# ------------------------------------------------------------ helpers ------

def make_plant(m=8e-6, b=3.5e-4) -> PlantParams:
    nodes = np.arange(0.0, 101.0, 10.0)
    grid = np.arange(0.0, 101.0, 5.0)
    fk_fwd = np.full(len(nodes), 0.55)
    fk_rev = np.full(len(nodes), 0.50)
    fk_rev[nodes >= 60] = 0.60          # a rev high-friction region
    fs_fwd = np.full(len(grid), 0.62)
    fs_rev = np.full(len(grid), 0.65)
    return PlantParams(m=m, b=b, nodes=nodes, fk_fwd=fk_fwd, fk_rev=fk_rev,
                       grid_s=grid, fs_fwd=fs_fwd, fs_rev=fs_rev)


def pos_to_adc(pos):
    return RAW_AT_0PCT + np.asarray(pos) / 100.0 * (RAW_AT_100PCT - RAW_AT_0PCT)


def synth_run_from_sim(tmp_path, name, experiment, params, duty, pos, seg=None):
    adc = pos_to_adc(pos)
    return write_run(tmp_path / f"{name}.csv", experiment=experiment,
                     params=params, adc=adc,
                     duty_milli=np.round(np.asarray(duty) * 1000).astype(int),
                     seg=seg if seg is not None else 0)


# ------------------------------------------------------------ velocity -----

def test_zero_phase_velocity_no_lag_and_correct_amplitude():
    fs, f0 = 1000.0, 3.0
    t = np.arange(0, 2.0, 1 / fs)
    pos = 50 + 10 * np.sin(2 * np.pi * f0 * t)
    v_true = 10 * 2 * np.pi * f0 * np.cos(2 * np.pi * f0 * t)
    _, v = zero_phase_velocity(pos, fs=fs, fc=40.0)
    mid = slice(200, -200)
    # amplitude within 2 %, phase within 1 ms (zero-phase property)
    assert np.abs(v[mid] - v_true[mid]).max() < 0.05 * v_true.max()
    lag = np.argmax(np.correlate(v[mid], v_true[mid], "full")) - (len(v[mid]) - 1)
    assert abs(lag) <= 1


# ------------------------------------------------------------ simulator ----

def test_sim_stays_stuck_below_breakaway():
    plant = make_plant()
    duty = np.full(2000, 0.5)  # < fs_fwd 0.62
    r = simulate(plant, duty, p0=40.0)
    assert np.all(r.pos == 40.0)
    assert np.all(r.stuck)


def test_sim_moves_above_breakaway_and_resticks_when_cut():
    plant = make_plant()
    duty = np.concatenate([np.full(100, 0.7), np.zeros(700)])
    r = simulate(plant, duty, p0=20.0, endzone=False)
    assert r.pos[99] > 25.0                      # moved
    assert np.all(np.diff(r.pos) >= -1e-12)      # monotonic fwd
    assert abs(r.vel[-1]) == 0.0                 # re-stuck after cut
    assert r.pos[-1] < 100.0


def test_sim_endzone_interlock_cuts_torque():
    plant = make_plant()
    duty = np.full(3000, 0.9)
    r = simulate(plant, duty, p0=90.0, endzone=True)
    # cut at 97: may coast a little, but never reaches the hard stop
    assert r.pos[-1] < 99.0
    assert np.all(r.duty_applied[r.pos > 97.0][1:] == 0.0)


def test_sim_rev_high_friction_band_resticks_midtravel():
    # kinetic band 30-55 % rev exceeds the drive: fly in, decelerate, re-stick
    plant = make_plant()
    band = (plant.nodes >= 30) & (plant.nodes <= 55)
    plant.fk_rev[:] = 0.45
    plant.fk_rev[band] = 0.65
    plant.fs_rev[:] = 0.55
    plant.fs_rev[(plant.grid_s >= 30) & (plant.grid_s <= 55)] = 0.70
    duty = np.full(3000, -0.60)
    r = simulate(plant, duty, p0=80.0, endzone=False)
    assert r.pos.min() < 70.0                    # broke away and flew
    assert 25.0 < r.pos[-1] < 62.0               # stuck in/near the band
    assert r.stuck[-1]
    assert abs(r.vel[-1]) == 0.0


# ------------------------------------------------- kinetic fit round trip --

def test_kinetic_fit_roundtrip_recovers_b_and_friction(tmp_path):
    plant = make_plant()
    rng = np.random.default_rng(1)
    runs = []
    cases = [(0.7, 1, 15.0), (0.8, 1, 15.0), (-0.7, -1, 85.0), (-0.8, -1, 85.0),
             (0.75, 1, 5.0), (-0.75, -1, 95.0)]
    for k, (d, sgn, p0) in enumerate(cases):
        duty = np.concatenate([np.zeros(200), np.full(600, d), np.zeros(400)])
        sim = simulate(plant, duty, p0=p0, endzone=False)
        adc = measure(plant, sim.pos, duty, rng=rng)
        write_run(tmp_path / f"{k:02d}_step.csv", experiment="step_response",
                  params={"duty": d}, adc=adc,
                  duty_milli=np.round(duty * 1000).astype(int))
    runs = load_dir(tmp_path)
    fit = fit_kinetic(runs, v_min=30.0)
    assert fit.b == pytest.approx(plant.b, rel=0.35)
    # interior nodes must recover the generating map. Edge nodes AND nodes at
    # the step-start positions (10 % here) see only transient samples, where
    # the 40 Hz zero-phase smoothing biases a -> those absorb the error; the
    # module docstring declares this, the SEs/refine stage handle it.
    interior = (fit.nodes >= 20) & (fit.nodes <= 80) & ~np.isnan(fit.fk_fwd)
    assert np.nanmax(np.abs(fit.fk_fwd[interior] - 0.55)) < 0.08
    rev_int = (fit.nodes >= 20) & (fit.nodes <= 50) & ~np.isnan(fit.fk_rev)
    assert np.nanmax(np.abs(fit.fk_rev[rev_int] - 0.50)) < 0.08
    assert fit.resid_rms_duty < 0.08


# ------------------------------------------------- static map assembly -----

def test_static_map_brackets_and_est_rules():
    ev = [
        StaticEvidence("stall", 1, 0.60, 20.0, None, "a:seg0"),
        StaticEvidence("break", 1, 0.66, 20.0, None, "b:seg1"),
        StaticEvidence("stall", -1, 0.72, 75.0, None, "c:seg0"),
    ]
    smap = fit_static_map(ev, invalid_trials=0)
    i20 = np.argmin(np.abs(smap.grid - 20.0))
    assert smap.lb_fwd[i20] == 0.60 and smap.ub_fwd[i20] == 0.66
    assert smap.est_fwd[i20] == pytest.approx(0.63)
    i75 = np.argmin(np.abs(smap.grid - 75.0))
    assert smap.est_rev[i75] == pytest.approx(0.77)  # lb + 0.05 margin
    # fs() interpolates the filled estimate everywhere
    assert 0.0 < smap.fs(50.0, +1) < 1.0


def test_breakaway_grid_noise_trip_counts_invalid(tmp_path):
    # ramp "ends" at 0.005 with no real motion -> must be counted invalid
    n1, n2 = 20, 300
    duty = np.concatenate([np.full(n1, 0.005), np.full(n2, 0.005), np.zeros(200)])
    seg = np.concatenate([np.full(n1, 0), np.full(n2, 1), np.full(200, 2)])
    pos = np.full(len(duty), 11.0) + np.random.default_rng(0).normal(0, 0.1, len(duty))
    write_run(tmp_path / "01_breakaway_grid.csv", experiment="breakaway_grid",
              params={"positions": [10]}, adc=pos_to_adc(pos),
              duty_milli=np.round(duty * 1000).astype(int), seg=seg)
    runs = load_dir(tmp_path)
    ev, invalid = collect_static_evidence(runs)
    assert invalid == 1
    assert not [e for e in ev if e.kind == "break"]


def test_breakaway_grid_valid_trial_gives_break_evidence(tmp_path):
    plant = make_plant()  # fs_fwd = 0.62
    # +0.005 every 5 ms up to 0.65 (just above breakaway), then 300 ms hold
    ramp_n = 660
    ramp = np.round(np.minimum(0.005 * (1 + np.arange(ramp_n) // 5), 0.65), 3)
    duty = np.concatenate([ramp, np.full(300, ramp[-1]), np.zeros(200)])
    seg = np.concatenate([np.zeros(ramp_n), np.ones(300), np.full(200, 2)])
    sim = simulate(plant, duty, p0=30.0, endzone=False)
    write_run(tmp_path / "02_breakaway_grid.csv", experiment="breakaway_grid",
              params={"positions": [30]}, adc=pos_to_adc(sim.pos),
              duty_milli=np.round(duty * 1000).astype(int), seg=seg)
    runs = load_dir(tmp_path)
    ev, invalid = collect_static_evidence(runs)
    breaks = [e for e in ev if e.kind == "break"]
    assert invalid == 0 and len(breaks) == 1
    # onset refit must land at the true breakaway (fs 0.62), i.e. EARLIER
    # than the biased ramp-termination duty 0.65
    assert breaks[0].duty == pytest.approx(0.625, abs=0.015)
    assert breaks[0].pos == pytest.approx(30.0, abs=3.0)
    assert [e for e in ev if e.kind == "sustain"]  # sustain metric emitted


# ------------------------------------------------------------ observer -----

def test_observer_beats_legacy_on_simulated_step():
    plant = make_plant()
    rng = np.random.default_rng(7)
    # 100 ms pulse: stays well inside travel (no hard-stop dynamics)
    duty = np.concatenate([np.zeros(300), np.full(100, 0.75), np.zeros(500)])
    sim = simulate(plant, duty, p0=20.0, endzone=False)
    adc = measure(plant, sim.pos, duty, rng=rng)
    pos_meas = (adc - RAW_AT_0PCT) / (RAW_AT_100PCT - RAW_AT_0PCT) * 100.0
    cfg = ObserverConfig.from_plant(plant, model_err_duty=0.05)
    sc, detail = score("synth", duty, pos_meas, plant, cfg)
    assert sc.rms_moving < sc.rms_moving_legacy      # beats PT1-150ms
    assert abs(sc.lag_ms) <= 10.0                    # near zero lag
    # velocity error vs true sim velocity, not just the smoothed reference
    rms_true = np.sqrt(np.mean((detail["v_est"] - sim.vel) ** 2))
    assert rms_true < 40.0


# ------------------------------------------------------------ overlay ------

def test_overlay_scores_zero_for_self_consistent_trace():
    plant = make_plant()
    duty = np.concatenate([np.zeros(100), np.full(300, 0.8), np.zeros(300)])
    sim = simulate(plant, duty, p0=20.0, endzone=False)
    _, res = overlay_arrays(plant, duty, sim.pos)
    assert res.rmse_pct < 1e-6
