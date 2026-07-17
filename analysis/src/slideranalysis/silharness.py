"""SIL harness: closed loop of perturbed plant + sensor + EKF + controller.

The controller and observer always run on the NOMINAL identified plant; the
simulated truth is a PERTURBED plant (friction scaled within the measured
±0.04-duty trial spread, m/b within their fit uncertainty) with sensor noise
and the measured PWM spike tail switched on. A controller that only works on
the nominal plant is decoration.

CLI (`uv run acceptance`) runs the plan.md acceptance metrics in sim and
writes plots + a PASS/FAIL table. Explicitly NOT credibly testable in sim:

- A6 (audible thunk): no acoustics in the model.
- A8 (manual grab): the hand is modeled as infinitely stiff position forcing;
  real grab dynamics (hand impedance, partial grips) are not represented --
  the harness tests the detection/release LOGIC only.
- A4 at 50 %/s: below the identified regime (|v| > 30 %/s) the sim's friction
  is an extrapolation, and the measured cliff says the real plant will
  stick-slip there; the sim result is reported but flagged NOT CREDIBLE.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from .controller import Controller, ControllerConfig, Mode, nominal_plant
from .loadrun import RAW_AT_0PCT, RAW_AT_100PCT
from .observer import DutyDrivenKF, ObserverConfig
from .plantsim import PlantParams, PlantStepper

FS_HZ = 1000.0

# measured spike tail (sensorfit): singles up to ~2-5 %; modeled as rare
# uniform outliers on top of the gaussian floor
SPIKE_PROB_PWM = 0.004
SPIKE_COUNTS = (60.0, 240.0)


def perturbed_plant(rng: np.random.Generator,
                    friction_scale: tuple[float, float] | None = None,
                    base: PlantParams | None = None) -> PlantParams:
    """Randomize the plant within measured bounds.

    Friction (fs AND fk, per direction): x(1+delta), delta ~ U(-0.07, 0.07)
    which spans the observed ±0.04-duty breakaway spread at the ~0.6 level.
    m: x(1+U(-0.25, 0.25)) (fit range), b: x(1+U(-0.15, 0.15)).
    ``friction_scale`` pins (fwd, rev) deltas for stress tests.
    """
    p = base or nominal_plant()
    d_fwd, d_rev = (friction_scale if friction_scale is not None
                    else rng.uniform(-0.07, 0.07, 2))
    return replace(
        p,
        fk_fwd=p.fk_fwd * (1 + d_fwd), fs_fwd=p.fs_fwd * (1 + d_fwd),
        fk_rev=p.fk_rev * (1 + d_rev), fs_rev=p.fs_rev * (1 + d_rev),
        m=p.m * (1 + rng.uniform(-0.25, 0.25)),
        b=p.b * (1 + rng.uniform(-0.15, 0.15)),
    )


class SensorSim:
    """ADC model: gaussian floor (duty-dependent) + PWM spike tail + 0.25-count
    quantization, returned in % travel."""

    def __init__(self, plant: PlantParams, rng: np.random.Generator):
        self.plant = plant
        self.rng = rng
        self._scale = (RAW_AT_100PCT - RAW_AT_0PCT) / 100.0

    def sample(self, pos: float, duty: float) -> float:
        pl = self.plant
        sigma = pl.sigma_pwm_counts if abs(duty) > 0 else pl.sigma0_counts
        raw = RAW_AT_0PCT + pos * self._scale + self.rng.normal(0.0, sigma)
        if abs(duty) > 0 and self.rng.random() < SPIKE_PROB_PWM:
            raw += self.rng.uniform(*SPIKE_COUNTS) * self.rng.choice((-1.0, 1.0))
        raw = np.round(raw * 4.0) / 4.0
        return (raw - RAW_AT_0PCT) / self._scale


@dataclass
class Grab:
    start_tick: int
    duration: int          # ticks the hand holds the slider still


@dataclass
class LoopResult:
    target: float
    t: np.ndarray
    pos: np.ndarray          # sim truth
    vel: np.ndarray
    duty: np.ndarray
    p_hat: np.ndarray
    v_hat: np.ndarray
    mode: list
    ctrl: Controller
    settled_tick: int | None

    @property
    def final_err(self) -> float:
        return float(np.mean(self.pos[-100:]) - self.target)

    def rest_window(self) -> slice | None:
        if self.settled_tick is None:
            return None
        return slice(self.settled_tick, len(self.pos))


def run_move(true_plant: PlantParams, p0: float, target: float,
             rng: np.random.Generator,
             cfg: ControllerConfig | None = None,
             nominal: PlantParams | None = None,
             max_s: float = 8.0, rest_s: float = 2.5,
             grab: Grab | None = None) -> LoopResult:
    """One closed-loop move + post-settle rest window."""
    nominal = nominal or nominal_plant()
    ctrl = Controller(nominal, cfg)
    kf = DutyDrivenKF(nominal, ObserverConfig.from_plant(nominal,
                                                         model_err_duty=0.14),
                      p0=p0)
    plant = PlantStepper(true_plant, p0=p0)
    sensor = SensorSim(nominal, rng)

    n_max = int(max_s * FS_HZ)
    rest_needed = int(rest_s * FS_HZ)
    pos, vel, duty, ph, vh, modes = [], [], [], [], [], []
    u_prev = 0.0
    grabbed_pos = None
    started = False
    settled_at = None

    for i in range(n_max):
        z = sensor.sample(plant.p, u_prev)
        p_hat, v_hat = kf.step(u_prev, z)
        if not started:
            ctrl.set_target(target, p_hat)
            started = True
        u = ctrl.tick(p_hat, v_hat)
        plant.step(u)
        # grab injection: infinitely stiff hand holds the slider still
        if grab is not None and grab.start_tick <= i < grab.start_tick + grab.duration:
            if grabbed_pos is None:
                grabbed_pos = plant.p
            plant.force(grabbed_pos, 0.0, True)
        elif grab is not None and i == grab.start_tick + grab.duration:
            grabbed_pos = None

        pos.append(plant.p); vel.append(plant.v); duty.append(u)
        ph.append(p_hat); vh.append(v_hat); modes.append(ctrl.mode)
        u_prev = u

        if ctrl.settled and settled_at is None:
            settled_at = i
        if settled_at is not None and i - settled_at >= rest_needed:
            break

    return LoopResult(
        target=ctrl.target if ctrl.target is not None else target,
        t=np.arange(len(pos)) / FS_HZ,
        pos=np.array(pos), vel=np.array(vel), duty=np.array(duty),
        p_hat=np.array(ph), v_hat=np.array(vh), mode=modes,
        ctrl=ctrl, settled_tick=settled_at,
    )


# ------------------------------------------------------------ metrics -----

@dataclass
class MetricResult:
    metric: str
    target: str
    result: str
    verdict: str      # PASS | FAIL | NOT CREDIBLE
    notes: str = ""


def eval_a1_a2_a3(n: int = 40, seed: int = 1) -> tuple[list[MetricResult], list[LoopResult], list[float]]:
    rng = np.random.default_rng(seed)
    errs, settle_ok, rest_silent, rest_pp, gave_up = [], [], [], [], 0
    results = []
    for k in range(n):
        p0 = rng.uniform(8, 92)
        while True:
            tgt = rng.uniform(8, 92)
            if abs(tgt - p0) >= 5.0:
                break
        plant = perturbed_plant(rng)
        res = run_move(plant, p0, tgt, rng)
        results.append(res)
        errs.append(abs(res.final_err))
        gave_up += int(res.ctrl.log.gave_up)
        w = res.rest_window()
        if w is None or (len(res.pos) - w.start) < 2000:
            settle_ok.append(False)
            rest_silent.append(False)
            rest_pp.append(np.nan)
        else:
            settle_ok.append(True)
            rest_silent.append(bool(np.all(res.duty[w.start:] == 0.0)))
            rest_pp.append(float(np.ptp(res.pos[w.start:])))
    errs = np.array(errs)
    med, p95 = float(np.median(errs)), float(np.percentile(errs, 95))
    rows = [
        MetricResult("A1 landing (40 randomized moves, perturbed plants)",
                     "median <=0.5 %, p95 <=1.0 %",
                     f"median {med:.2f} %, p95 {p95:.2f} %, max {errs.max():.2f} %"
                     f", gave_up {gave_up}",
                     "PASS" if (med <= 0.5 and p95 <= 1.0) else "FAIL"),
        MetricResult("A2 rest silence after settle",
                     "duty identically 0 for >=2 s, no re-trigger",
                     f"{sum(rest_silent)}/{sum(settle_ok)} settled moves silent",
                     "PASS" if all(r for r, ok in zip(rest_silent, settle_ok) if ok)
                     and all(settle_ok) else "FAIL"),
        MetricResult("A3 no limit cycle at rest",
                     "true position p-p <=0.3 %",
                     f"max p-p {np.nanmax(rest_pp):.3f} %",
                     "PASS" if np.nanmax(rest_pp) <= 0.3 else "FAIL",
                     "true (sim) position; measured p-p is sensor noise"),
    ]
    return rows, results, list(errs)


def eval_a4(seed: int = 2) -> tuple[list[MetricResult], LoopResult]:
    rng = np.random.default_rng(seed)
    rows = []
    example = None
    for v_c, credible in ((150.0, True), (50.0, False)):
        ripples = []
        for k in range(4):
            plant = perturbed_plant(rng)
            cfg = ControllerConfig(v_cruise=v_c,
                                   v_min_cmd=min(120.0, v_c))
            res = run_move(plant, 15.0, 88.0, rng, cfg=cfg)
            # cruise window: MOVE mode, mid-travel, past the accel phase
            mask = np.array([m == Mode.MOVE for m in res.mode])
            mask &= (res.pos > 30) & (res.pos < 75)
            v = res.vel[mask]
            if len(v) > 100:
                ripples.append(float(np.std(v) / np.mean(np.abs(v))))
            if example is None and credible:
                example = res
        r = max(ripples) if ripples else float("nan")
        if credible:
            rows.append(MetricResult(
                "A4 cruise smoothness @150 %/s (validated regime)",
                "sigma/mean <=0.15", f"worst of 4 runs: {r:.3f}",
                "PASS" if r <= 0.15 else "FAIL"))
        else:
            rows.append(MetricResult(
                "A4 cruise smoothness @50 %/s (plan.md value)",
                "sigma/mean <=0.15", f"worst of 4 runs: {r:.3f}",
                "NOT CREDIBLE",
                "50 %/s is below the identified regime AND below the cliff -- "
                "the sim extrapolates; the real plant will stick-slip there"))
    return rows, example


def eval_a5(n: int = 10, seed: int = 3) -> list[MetricResult]:
    rng = np.random.default_rng(seed)
    times = []
    for k in range(n):
        p0 = rng.uniform(10, 42)
        tgt = p0 + 50.0
        if k % 2:
            p0, tgt = tgt, p0     # half of them reverse
        plant = perturbed_plant(rng)
        res = run_move(plant, p0, tgt, rng)
        times.append(res.settled_tick / FS_HZ if res.settled_tick else np.inf)
    times = np.array(times)
    return [MetricResult(
        "A5 50 %-travel move incl. settle (10 runs, half rev)",
        "<=1.2 s",
        f"median {np.median(times):.2f} s, max {times.max():.2f} s",
        "PASS" if times.max() <= 1.2 else "FAIL")]


def eval_a7(n: int = 10, seed: int = 4) -> list[MetricResult]:
    rng = np.random.default_rng(seed)
    times, overshoots = [], []
    for k in range(n):
        tgt = rng.uniform(20, 80)
        p0 = tgt - 3.0 if k % 2 == 0 else tgt + 3.0
        plant = perturbed_plant(rng)
        res = run_move(plant, p0, tgt, rng)
        times.append(res.settled_tick / FS_HZ if res.settled_tick else np.inf)
        overshoots.append(res.ctrl.log.overshoots)
    times = np.array(times)
    return [MetricResult(
        "A7 detent pull-in from 3 % (10 runs, both sides)",
        "settled <=400 ms, <=1 overshoot",
        f"median {np.median(times)*1e3:.0f} ms, max {times.max()*1e3:.0f} ms, "
        f"max overshoots {max(overshoots)}",
        "PASS" if (times.max() <= 0.4 and max(overshoots) <= 1) else "FAIL")]


def eval_band_stress(seed: int = 5) -> tuple[list[MetricResult], LoopResult]:
    """Rev moves into/through the 55-80 % rev poor-sustain band, friction at
    the worst measured end (+7 %)."""
    rng = np.random.default_rng(seed)
    errs, kicks, gave_up = [], [], 0
    example = None
    cases = [(90.0, 65.0), (85.0, 60.0), (95.0, 70.0), (80.0, 30.0),
             (90.0, 62.0), (88.0, 58.0)]
    for p0, tgt in cases:
        plant = perturbed_plant(rng, friction_scale=(0.07, 0.07))
        res = run_move(plant, p0, tgt, rng)
        errs.append(abs(res.final_err))
        kicks.append(res.ctrl.log.kicks)
        gave_up += int(res.ctrl.log.gave_up)
        if example is None and res.ctrl.log.kicks > 0:
            example = res
        if example is None:
            example = res
    return [MetricResult(
        "Stress: rev moves into 55-80 % band, friction +7 %",
        "completes, lands <=1.0 %",
        f"max err {max(errs):.2f} %, kicks used {kicks}, gave_up {gave_up}",
        "PASS" if (max(errs) <= 1.0 and gave_up == 0) else "FAIL")], example


def eval_grab(seed: int = 6) -> tuple[list[MetricResult], LoopResult]:
    rng = np.random.default_rng(seed)
    plant = perturbed_plant(rng)
    grab = Grab(start_tick=140, duration=1000)  # mid-flight, 1 s hold
    res = run_move(plant, 20.0, 80.0, rng, grab=grab, max_s=8.0)
    log = res.ctrl.log
    detect_ms = ((log.grab_detect_tick - grab.start_tick) / FS_HZ * 1e3
                 if log.grab_detect_tick else float("inf"))
    # torque zero within 2 ticks of detection, and stays zero while grabbed
    if log.grab_detect_tick is not None:
        after = res.duty[log.grab_detect_tick + 1:grab.start_tick + grab.duration]
        released = bool(np.all(after == 0.0))
        fight = float(np.max(np.abs(after))) if len(after) else 0.0
    else:
        released, fight = False, float("nan")
    silent_after = bool(np.all(res.duty[-1500:] == 0.0))
    verdict = "PASS" if (log.grab_detect_tick is not None and released
                         and silent_after) else "FAIL"
    return [MetricResult(
        "Grab during move (LOGIC only -- hand modeled as rigid clamp)",
        "detect, release immediately, no fight, silent after",
        f"detected after {detect_ms:.0f} ms (escalation-failure detector), "
        f"torque after detection: {fight:.2f}, adopted position, "
        f"silent {'yes' if silent_after else 'NO'}",
        verdict,
        "A8's 100 ms / real grab dynamics NOT testable in sim")], res


# --------------------------------------------------------------- plots ----

def _plot_run(res: LoopResult, out: Path, title: str) -> None:
    import matplotlib.pyplot as plt

    from .plots import AQUA, BLUE, INK2, ORANGE, _style

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 7), sharex=True,
                                        height_ratios=[3, 2, 1.4])
    ax1.axhline(res.target, color=INK2, lw=1, ls=":")
    ax1.annotate("target", xy=(res.t[-1], res.target), fontsize=8,
                 color=INK2, xytext=(-2, 4), textcoords="offset points",
                 ha="right")
    ax1.plot(res.t, res.pos, color=BLUE, lw=1.4, label="true position")
    ax1.plot(res.t, res.p_hat, color=ORANGE, lw=0.9, label="KF estimate")
    ax1.legend(fontsize=8, frameon=False)
    _style(ax1, title, None, "position [%]")
    ax2.plot(res.t, res.vel, color=BLUE, lw=1.2, label="true velocity")
    ax2.plot(res.t, res.v_hat, color=ORANGE, lw=0.9, label="KF estimate")
    ax2.legend(fontsize=8, frameon=False)
    _style(ax2, None, None, "velocity [%/s]")
    ax3.plot(res.t, res.duty, color=AQUA, lw=1.0, label="commanded duty")
    ax3.legend(fontsize=8, frameon=False)
    _style(ax3, None, "t [s]", "duty")
    ax3.set_ylim(-1, 1)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def _plot_a1(errs: list[float], out: Path) -> None:
    import matplotlib.pyplot as plt

    from .plots import BLUE, INK2, ORANGE, _style

    fig, ax = plt.subplots(figsize=(7, 3.4))
    e = np.sort(np.array(errs))
    ax.plot(np.arange(1, len(e) + 1), e, "o", ms=5, color=BLUE, mew=0)
    for val, name, color in ((np.median(e), "median", ORANGE),
                             (np.percentile(e, 95), "p95", INK2)):
        ax.axhline(val, color=color, lw=1.2, ls="--")
        ax.annotate(f"{name} {val:.2f} %", xy=(1, val), fontsize=8,
                    color=color, xytext=(2, 3), textcoords="offset points")
    _style(ax, "A1: |landing error| of 40 randomized moves (sorted)",
           "move (sorted by error)", "|error| [%]")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


# ----------------------------------------------------------------- CLI ----

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Controller acceptance run in the validated sim (SIL).")
    ap.add_argument("--out", type=Path, default=Path("../docs/findings"))
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--impulse-a4", action="store_true",
                    help="run only the A4@50-as-impulse-trains study")
    args = ap.parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")
    args.out.mkdir(parents=True, exist_ok=True)

    if args.impulse_a4:
        impulse_train_study(args.out, seed=args.seed + 10,
                            plot=not args.no_plots)
        return 0

    rows: list[MetricResult] = []
    r123, a1_results, errs = eval_a1_a2_a3(seed=args.seed)
    rows += r123
    a4_rows, a4_example = eval_a4(seed=args.seed + 1)
    rows += a4_rows
    rows += eval_a5(seed=args.seed + 2)
    rows += eval_a7(seed=args.seed + 3)
    band_rows, band_example = eval_band_stress(seed=args.seed + 4)
    rows += band_rows
    grab_rows, grab_example = eval_grab(seed=args.seed + 5)
    rows += grab_rows
    rows.append(MetricResult("A6 reversal thunk", "no audible thunk", "--",
                             "NOT CREDIBLE", "no acoustics in the sim"))
    rows.append(MetricResult("A8 manual grab (real dynamics)",
                             "detect <=100 ms, release <=2 ticks", "--",
                             "NOT CREDIBLE",
                             "hand impedance not modeled; logic tested above"))

    print(f"\n{'METRIC':58s} {'TARGET':34s} {'RESULT':52s} VERDICT")
    for r in rows:
        print(f"{r.metric:58s} {r.target:34s} {r.result:52s} {r.verdict}"
              + (f"\n{'':58s} note: {r.notes}" if r.notes else ""))

    if not args.no_plots:
        _plot_a1(errs, args.out / "controller_a1_errors.png")
        # a representative long move from the A1 batch
        longest = max(a1_results, key=lambda r: abs(r.pos[0] - r.target))
        _plot_run(longest, args.out / "controller_move_example.png",
                  f"A1 move {longest.pos[0]:.0f} -> {longest.target:.0f} % "
                  f"(perturbed plant, err {longest.final_err:+.2f} %)")
        if band_example is not None:
            _plot_run(band_example, args.out / "controller_band_move.png",
                      f"Band stress: rev into 55-80 % at friction +7 % "
                      f"(kicks {band_example.ctrl.log.kicks}, "
                      f"err {band_example.final_err:+.2f} %)")
        if grab_example is not None:
            _plot_run(grab_example, args.out / "controller_grab.png",
                      "Grab during move: detect -> release -> adopt -> silent")
        if a4_example is not None:
            _plot_run(a4_example, args.out / "controller_cruise.png",
                      "A4 cruise 15 -> 88 % at v_des 150 %/s (perturbed plant)")
        print(f"\nplots -> {args.out}/controller_*.png")

    return 0


if __name__ == "__main__":
    sys.exit(main())


# ------------------------------------------- A4@50 impulse-train study ----

def impulse_train_run(true_plant: PlantParams, p0: float, target: float,
                      v_des: float, width_ms: int,
                      rng: np.random.Generator,
                      nominal: PlantParams | None = None,
                      max_s: float = 12.0, min_gap_ms: int = 0):
    """Slow-traverse trajectory mode: calibrated pulse train.

    Deficit-triggered scheduler: a reference ramp advances at v_des; a pulse
    of fixed ``width_ms`` at +-0.8 duty fires whenever the estimated position
    lags the ramp by half the pulse's expected travel (measured M7 table).
    Self-pacing: a plant that travels short fires sooner. Runs on the KF
    estimate, not sim truth.
    """
    nominal = nominal or nominal_plant()
    cfg = ControllerConfig()
    ctrl = Controller(nominal, cfg)     # only for the pulse-width table
    expected = np.interp(width_ms, cfg.fwd_widths, cfg.fwd_travel)
    kf = DutyDrivenKF(nominal, ObserverConfig.from_plant(nominal, 0.14), p0=p0)
    plant = PlantStepper(true_plant, p0=p0)
    sensor = SensorSim(nominal, rng)

    d = 1.0 if target > p0 else -1.0
    n = int(max_s * 1000)
    pos = np.empty(n); vel = np.empty(n); duty = np.empty(n)
    ref = p0
    u_prev = 0.0
    pulse_left = 0
    fired = 0
    done_ticks = 0
    last_end = -10**6
    i_end = n
    for i in range(n):
        z = sensor.sample(plant.p, u_prev)
        p_hat, v_hat = kf.step(u_prev, z)
        ref += d * v_des * 0.001
        # clamp the ramp so a stuck plant cannot wind up an unbounded deficit
        if (ref - p_hat) * d > 3.0 * expected:
            ref = p_hat + d * 3.0 * expected
        u = 0.0
        gap = i - last_end
        if pulse_left > 0:
            pulse_left -= 1
            u = 0.8 * d
            last_end = i
        elif ((ref - p_hat) * d >= 0.5 * expected
              and (target - p_hat) * d > 0.3 and gap >= min_gap_ms):
            pulse_left = width_ms - 1
            u = 0.8 * d
            last_end = i
            fired += 1
        plant.step(u)
        pos[i] = plant.p; vel[i] = plant.v; duty[i] = u
        u_prev = u
        # done: at/near target (same tolerance the pulse guard uses), settled
        if (target - p_hat) * d <= 0.3 and pulse_left == 0 and abs(plant.v) < 3:
            done_ticks += 1
            if done_ticks >= 50:
                i_end = i + 1
                break
        else:
            done_ticks = 0
    return (np.arange(i_end) / 1000.0, pos[:i_end], vel[:i_end],
            duty[:i_end], fired)


def impulse_train_metrics(t, pos, v_des: float):
    """A4-style numbers on the TRUE position: window-mean velocity ripple
    (100 ms, non-overlapping, central 80 % of the traverse) and the visible
    staircase amplitude (travel between consecutive rest dwells)."""
    n = len(pos)
    lo, hi = int(0.1 * n), int(0.9 * n)
    w = 100
    v_win = np.array([(pos[i + w] - pos[i]) / (w / 1000.0)
                      for i in range(lo, hi - w, w)])
    ripple = float(np.std(v_win) / np.mean(np.abs(v_win)))
    v_mean = float((pos[hi] - pos[lo]) / ((hi - lo) / 1000.0))
    # staircase: rest dwells = |v| < 3 %/s over >= 5 ms
    dpos = np.abs(np.diff(pos)) * 1000.0
    at_rest = dpos < 3.0
    steps = []
    last_rest_pos = None
    run = 0
    for i in range(lo, hi):
        if at_rest[min(i, len(at_rest) - 1)]:
            run += 1
            if run == 5:
                if last_rest_pos is not None:
                    steps.append(abs(pos[i] - last_rest_pos))
                last_rest_pos = pos[i]
        else:
            run = 0
    max_step = float(max(steps)) if steps else 0.0
    return dict(v_mean=v_mean, ripple_100ms=ripple, max_step_pct=max_step,
                n_steps=len(steps))


def impulse_train_study(out: Path, seed: int = 11, plot: bool = True):
    """A4@50 via impulse trains: 2 plants (nominal, friction +7 %), several
    pulse widths, 50 %/s target over 20 -> 70 %."""
    print("\n== A4@50 %/s AS IMPULSE TRAINS (sim; measured M7 pulse table) ==")
    print(f"  {'width':>6s} {'plant':>9s} {'fired':>6s} {'v_mean':>7s} "
          f"{'ripple(100ms)':>13s} {'max step':>9s} {'steps':>6s}")
    results = {}
    for width, min_gap in ((3, 0), (5, 0), (8, 0), (20, 0),
                           (5, 17), (8, 20)):
        for label, fscale in (("nominal", None), ("+7% fric", (0.07, 0.07))):
            rng = np.random.default_rng(seed)
            plant = perturbed_plant(rng, friction_scale=fscale) \
                if fscale else nominal_plant()
            t, pos, vel, duty, fired = impulse_train_run(
                plant, 20.0, 70.0, 50.0, width, rng, min_gap_ms=min_gap)
            m = impulse_train_metrics(t, pos, 50.0)
            tag = f"{width}ms" + (f"/g{min_gap}" if min_gap else "")
            print(f"  {tag:>8s} {label:>9s} {fired:6d} {m['v_mean']:6.1f} "
                  f"{m['ripple_100ms']:13.3f} {m['max_step_pct']:8.2f}% "
                  f"{m['n_steps']:6d}")
            results[(width, min_gap, label)] = (t, pos, vel, duty, m)

    if plot:
        import matplotlib.pyplot as plt

        from .plots import AQUA, BLUE, INK2, ORANGE, _style

        fig, axes = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True,
                                 height_ratios=[3, 2])
        cases = {(8, 20): ORANGE, (8, 0): BLUE, (20, 0): AQUA}
        for (width, gap), color in cases.items():
            t, pos, vel, duty, m = results[(width, gap, "nominal")]
            lbl = f"{width} ms" + (f", min gap {gap} ms" if gap else "")
            axes[0].plot(t, pos, color=color, lw=1.2,
                         label=f"{lbl}: ripple {m['ripple_100ms']:.2f}, "
                               f"max step {m['max_step_pct']:.2f} %")
            axes[1].plot(t, vel, color=color, lw=0.8, alpha=0.8)
        tt = np.linspace(0, 1.0, 50)
        axes[0].plot(tt, 20 + 50 * tt, color=INK2, lw=1, ls=":",
                     label="ideal 50 %/s ramp")
        axes[0].legend(fontsize=8, frameon=False)
        _style(axes[0], "50 %/s traverse synthesized from calibrated impulse "
                        "trains (nominal plant)", None, "position [%]")
        _style(axes[1], None, "t [s]", "true velocity [%/s]")
        axes[0].set_xlim(0, 1.15)
        fig.tight_layout()
        fig.savefig(out / "controller_a4_impulse_trains.png", dpi=130)
        plt.close(fig)
        print(f"  plot -> {out}/controller_a4_impulse_trains.png")
    return results
