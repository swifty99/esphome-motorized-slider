"""Plant simulator: stick-slip friction map + inertia + brake damping + sensor.

State (p [%], v [%/s]), input u = signed duty in [-1, 1]. Semantics:

- STUCK: v = 0 until |u| exceeds the static map F_s(p, sign u).
- SLIDING:  m * dv/dt = u - sign(v) * F_k(p, sign v) - b * v
  b stays active at u = 0: the L9110 has no coast -- duty 0 shorts the motor
  (brake), so "coast" is really electrodynamic braking. b is fitted from
  exactly such samples and already contains that effect.
- Re-stick: v crossing ~0 while |u| <= F_s -> stuck. Because F_k can locally
  exceed the applied duty (the measured rev high-friction band ~55-80 %),
  the sim reproduces mid-travel re-sticks.
- End zone: outside [3, 97] the firmware interlock forces duty 0; the sim
  mirrors that when ``endzone=True`` (turn off when feeding *recorded* duty
  traces -- those already contain the cut).
- Hard stops at 0/100 %.
- Backlash: STUB (width 0). No M6 reversal-at-speed data yet; the parameter
  exists so the fit can drop in once data lands.
- Sensor: pos -> ADC counts + N(0, sigma(duty)) + 0.25-count quantization
  (mean of 4 int samples). Spike tail not modeled (documented in sensorfit).

KNOWN UNMODELED (stated, not hidden): stick-slip creep (real 0.65-duty creep
moves ~4 %/s intermittently; the sim either sticks or slides cleanly),
duty->force is assumed linear and mode-independent (fast vs slow decay differ
by ~5-15 % in the 0.70 rungs), no thermal/supply drift.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from pathlib import Path

import numpy as np

from .loadrun import COUNTS_PER_PERCENT, RAW_AT_0PCT, RAW_AT_100PCT


@dataclass
class PlantParams:
    # dynamics (duty units: force == stall-equivalent duty)
    m: float                  # duty / (%/s^2)  inertia-equivalent
    b: float                  # duty / (%/s)    viscous + brake damping
    # kinetic friction map (value at |v| >> v_strib)
    nodes: np.ndarray
    fk_fwd: np.ndarray
    fk_rev: np.ndarray
    # static (breakaway) map
    grid_s: np.ndarray
    fs_fwd: np.ndarray
    fs_rev: np.ndarray
    # Stribeck excess: friction(p, v) = fk(p,dir) + g_dir * exp(-|v|/v_strib)
    g_fwd: float = 0.0
    g_rev: float = 0.0
    v_strib: float = 100.0
    # sensor
    sigma0_counts: float = 8.1
    sigma_pwm_counts: float = 11.7
    counts_per_percent: float = COUNTS_PER_PERCENT
    # behavior
    backlash_pct: float = 0.0          # STUB until M6 data
    v_stick: float = 3.0               # %/s: below this + |u|<Fs -> stick
    endzone: tuple[float, float] = (3.0, 97.0)
    meta: dict = field(default_factory=dict)

    def __post_init__(self):
        for name in ("nodes", "fk_fwd", "fk_rev", "grid_s", "fs_fwd", "fs_rev"):
            setattr(self, name, np.asarray(getattr(self, name), dtype=float))

    def fk(self, p: float, direction: float, v: float = 1e9) -> float:
        """Kinetic friction at position p, drive direction, speed |v|."""
        f = self.fk_fwd if direction > 0 else self.fk_rev
        base = float(np.interp(p, self.nodes, f))
        g = self.g_fwd if direction > 0 else self.g_rev
        if g and self.v_strib > 0:
            base += g * float(np.exp(-abs(v) / self.v_strib))
        return base

    def fs(self, p: float, direction: float) -> float:
        f = self.fs_fwd if direction > 0 else self.fs_rev
        return float(np.interp(p, self.grid_s, f))

    # -- serialization ----------------------------------------------------
    def to_json(self, path: str | Path) -> None:
        d = asdict(self)
        for k, v in d.items():
            if isinstance(v, np.ndarray):
                d[k] = [round(float(x), 6) for x in v]
        Path(path).write_text(json.dumps(d, indent=2) + "\n", encoding="utf-8")

    @classmethod
    def from_json(cls, path: str | Path) -> "PlantParams":
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        d["endzone"] = tuple(d.get("endzone", (3.0, 97.0)))
        return cls(**d)

    @classmethod
    def from_fits(cls, sensor, static_map, kinetic, meta: dict | None = None) -> "PlantParams":
        """Assemble from sensorfit/frictionfit results, filling unsupported
        kinetic nodes by interpolation between supported ones."""
        def fill(nodes, f):
            good = ~np.isnan(f)
            if not np.any(good):
                raise ValueError("kinetic map has no supported nodes in one direction")
            return np.interp(nodes, nodes[good], f[good])

        return cls(
            m=kinetic.m, b=kinetic.b,
            nodes=kinetic.nodes,
            fk_fwd=fill(kinetic.nodes, kinetic.fk_fwd),
            fk_rev=fill(kinetic.nodes, kinetic.fk_rev),
            g_fwd=max(kinetic.g_fwd, 0.0),
            g_rev=max(kinetic.g_rev, 0.0),
            v_strib=kinetic.v_strib or 100.0,
            grid_s=static_map.grid,
            fs_fwd=static_map.est_fwd,
            fs_rev=static_map.est_rev,
            sigma0_counts=sensor.sigma0_counts,
            sigma_pwm_counts=(sensor.sigma_pwm_counts or sensor.sigma0_counts),
            meta=meta or {},
        )


@dataclass
class SimResult:
    t: np.ndarray
    pos: np.ndarray
    vel: np.ndarray
    stuck: np.ndarray
    duty_applied: np.ndarray


class PlantStepper:
    """Per-tick stepping interface to the plant (for closed-loop use).

    Exactly the physics of :func:`simulate` (which delegates here). The
    closed-loop harness may additionally force the state (grab simulation)
    via :meth:`force` -- the hand is modeled as infinitely stiff, which is a
    stated limitation (real grab dynamics are NOT represented).
    """

    def __init__(self, params: PlantParams, p0: float, v0: float = 0.0,
                 dt: float = 0.001, endzone: bool = True,
                 endstop_safe: bool = False):
        self.params = params
        self.dt = dt
        self.endzone = endzone
        self.endstop_safe = endstop_safe
        self.p = float(p0)
        self.v = float(v0)
        self.stuck = abs(self.v) < params.v_stick

    def force(self, p: float, v: float = 0.0, stuck: bool = True) -> None:
        """Override the state (grab/disturbance injection)."""
        self.p, self.v, self.stuck = float(p), float(v), stuck

    def step(self, u_cmd: float) -> tuple[float, float, bool, float]:
        """One 1 kHz tick with commanded duty. Returns (p, v, stuck, u_applied)."""
        params = self.params
        dt = self.dt
        m, b = params.m, params.b
        lo, hi = params.endzone
        p, v, stuck = self.p, self.v, self.stuck

        u = u_cmd
        if self.endzone and not self.endstop_safe and (p < lo or p > hi):
            u = 0.0
        if stuck:
            sdir = np.sign(u)
            if sdir != 0 and abs(u) > params.fs(p, sdir):
                stuck = False
                v = 0.0
        if not stuck:
            sdir = np.sign(v) if v != 0.0 else np.sign(u)
            if sdir == 0.0:
                stuck = True
            else:
                fk_tot = params.fk(p, sdir, v)
                if v == 0.0:
                    # unstick tick: kinetic friction is dissipative -- it cannot
                    # exceed the drive that just broke static contact. If the
                    # low-speed kinetic level (fk+g) is above |u|, motion starts
                    # and immediately decays = the measured "poor sustain".
                    fk_tot = min(fk_tot, abs(u) * 0.95)
                acc = (u - sdir * fk_tot - b * v) / m
                v_new = v + acc * dt
                if v != 0.0 and np.sign(v_new) != np.sign(v):
                    # decelerated through zero: re-stick unless u re-breaks
                    if abs(u) <= params.fs(p, np.sign(u) if u != 0 else sdir):
                        v_new = 0.0
                        stuck = True
                elif v == 0.0 and np.sign(v_new) != np.sign(u):
                    v_new = 0.0   # friction map inconsistency guard (fk > u at unstick)
                    stuck = True
                v = v_new
                if abs(v) < params.v_stick and abs(u) <= params.fs(p, sdir):
                    v = 0.0
                    stuck = True
        p += v * dt
        if p <= 0.0:
            p, v, stuck = 0.0, 0.0, True
        elif p >= 100.0:
            p, v, stuck = 100.0, 0.0, True

        self.p, self.v, self.stuck = p, v, stuck
        return p, v, stuck, u


def simulate(params: PlantParams, duty, p0: float, v0: float = 0.0,
             dt: float = 0.001, endzone: bool = True,
             endstop_safe: bool = False) -> SimResult:
    """Integrate the plant over a duty trace (one value per tick)."""
    duty = np.asarray(duty, dtype=float)
    n = len(duty)
    pos = np.empty(n)
    vel = np.empty(n)
    stk = np.empty(n, dtype=bool)
    ua = np.empty(n)

    stepper = PlantStepper(params, p0=p0, v0=v0, dt=dt, endzone=endzone,
                           endstop_safe=endstop_safe)
    for i in range(n):
        pos[i], vel[i], stk[i], ua[i] = stepper.step(duty[i])

    return SimResult(t=np.arange(n) * dt, pos=pos, vel=vel, stuck=stk, duty_applied=ua)


def measure(params: PlantParams, pos, duty, rng=None) -> np.ndarray:
    """Apply the sensor model: position [%] -> noisy quantized adc_raw counts."""
    rng = rng or np.random.default_rng(0)
    pos = np.asarray(pos, dtype=float)
    duty = np.asarray(duty, dtype=float)
    raw = RAW_AT_0PCT + pos / 100.0 * (RAW_AT_100PCT - RAW_AT_0PCT)
    sigma = np.where(np.abs(duty) > 0.0, params.sigma_pwm_counts, params.sigma0_counts)
    noisy = raw + rng.normal(0.0, 1.0, len(pos)) * sigma
    return np.round(noisy * 4.0) / 4.0


# ------------------------------------------------------------- validation ---

@dataclass
class OverlayResult:
    run: str
    rmse_pct: float
    final_err_pct: float
    max_err_pct: float
    n: int
    note: str = ""


def overlay_arrays(params: PlantParams, duty: np.ndarray, pos_meas: np.ndarray,
                   run: str = "", endzone: bool = False) -> tuple[SimResult, OverlayResult]:
    """Simulate a recorded duty trace and score against the measured position.

    endzone=False by default: recorded traces already contain the interlock's
    duty cut, so the sim must not cut a second time at its own position.
    """
    p0 = float(np.median(pos_meas[:20]))
    sim = simulate(params, duty, p0=p0, endzone=endzone)
    err = sim.pos - pos_meas
    res = OverlayResult(
        run=run,
        rmse_pct=float(np.sqrt(np.mean(err**2))),
        final_err_pct=float(np.mean(err[-200:])),
        max_err_pct=float(np.max(np.abs(err))),
        n=len(err),
    )
    return sim, res


def refine_inertia(params: PlantParams, cases: list[tuple[np.ndarray, np.ndarray]],
                   factors: np.ndarray | None = None,
                   window_ms: int = 150) -> tuple[float, np.ndarray, np.ndarray]:
    """Re-fit m by simulation against raw position (no filter bias).

    cases: [(duty_trace, pos_meas)], each starting at rest. The objective is
    the position RMSE over the first ``window_ms`` after drive onset ONLY --
    inertia is visible in nothing else, and scoring whole runs lets m drift
    off to compensate friction-map errors instead (observed: the unwindowed
    objective pushed m to the grid edge).

    Scans m over a factor grid (robust; also exposes how flat the objective
    is = how weakly m is identified). Returns (m_best, factors, rms_curve).
    """
    if factors is None:
        factors = np.geomspace(0.2, 8.0, 25)

    windows = []
    for duty, pos_meas in cases:
        active = np.abs(duty) > 0.05
        if not active.any():
            continue
        i0 = int(np.argmax(active))
        i1 = min(i0 + window_ms, len(duty))
        lead = max(i0 - 20, 0)
        windows.append((duty[lead:i1], pos_meas[lead:i1]))

    base_m = params.m
    rms = np.empty(len(factors))
    for k, f in enumerate(factors):
        params.m = base_m * f
        errs = []
        for duty, pos_meas in windows:
            _, res = overlay_arrays(params, duty, pos_meas)
            errs.append(res.rmse_pct**2)
        rms[k] = np.sqrt(np.mean(errs)) if errs else np.nan
    params.m = base_m
    return float(base_m * factors[int(np.argmin(rms))]), factors, rms
