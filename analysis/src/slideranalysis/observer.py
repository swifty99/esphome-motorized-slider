"""Observer prototype: 2-state Kalman filter driven by commanded duty.

Phase-3 item 3 (plan.md): position+velocity KF whose prediction runs the
identified plant model (friction map, b, m) on the applied duty, correcting
with the raw ADC position sample each tick. Strictly causal -- everything here
could run on the device at 1 kHz.

Judged on REAL recordings against the offline zero-phase velocity reference
(``velocity.zero_phase_velocity``): velocity RMS error and lag. A legacy-style
baseline (finite difference + PT1, tau = 150 ms -- what the old controller
had) is computed alongside for context.

Process model (EKF, additive white acceleration noise):

    p' = p + v dt
    v' = v + a(p, v, u) dt      a from the plant model, 0 when stuck

    F = [[1, dt], [0, 1 - (b/m) dt]]   (sliding)  |  [[1, 0], [0, 0]] (stuck)
    Q = sigma_a^2 * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]
    R = (sigma_adc(duty) / counts_per_percent)^2

The friction map enters as a known input, not a state: model error (the fit's
residual, ~0.05 duty RMS) is what sigma_a must cover -- sigma_a defaults to
resid_rms / m interpreted as acceleration noise.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .plantsim import PlantParams
from .velocity import zero_phase_velocity

FS_HZ = 1000.0


@dataclass
class ObserverConfig:
    sigma_a: float           # accel process noise [%/s^2] (white)
    r_quiet_pct: float       # measurement sigma, duty == 0 [%]
    r_pwm_pct: float         # measurement sigma under PWM [%]
    v_stick: float = 3.0
    gate_sigma: float = 5.0  # innovation gate (spike rejection), in sigmas

    @classmethod
    def from_plant(cls, plant: PlantParams, model_err_duty: float = 0.05,
                   err_corr_tau_s: float = 0.1) -> "ObserverConfig":
        """Process noise from the fit residual, whitened honestly.

        The friction-model error (residual ~0.1 duty) is NOT white at 1 kHz --
        it persists over ~err_corr_tau_s (map error changes with position, a
        ~100 ms scale at typical speeds). The equivalent white accel density
        is scaled by sqrt(dt/tau); feeding the raw residual as per-tick white
        noise makes the KF differentiate measurement noise instead.
        """
        dt = 1.0 / FS_HZ
        sigma_a = model_err_duty / plant.m * np.sqrt(dt / err_corr_tau_s)
        return cls(
            sigma_a=float(sigma_a),
            r_quiet_pct=plant.sigma0_counts / plant.counts_per_percent,
            r_pwm_pct=plant.sigma_pwm_counts / plant.counts_per_percent,
            v_stick=plant.v_stick,
        )


class DutyDrivenKF:
    """2-state EKF: state [p (%), v (%/s)], input duty, measurement p."""

    def __init__(self, plant: PlantParams, cfg: ObserverConfig,
                 p0: float = 50.0, v0: float = 0.0):
        self.plant = plant
        self.cfg = cfg
        self.x = np.array([p0, v0], dtype=float)
        self.P = np.diag([1.0, 100.0])
        self._stuck = True
        dt = 1.0 / FS_HZ
        self._Q = cfg.sigma_a**2 * np.array(
            [[dt**4 / 4, dt**3 / 2], [dt**3 / 2, dt**2]]
        )
        self._dt = dt

    def predict(self, u: float) -> None:
        p, v = self.x
        dt = self._dt
        pl = self.plant
        stuck = abs(v) < self.cfg.v_stick and abs(u) <= pl.fs(p, np.sign(u) if u else 1.0)
        self._stuck = stuck
        if stuck:
            a = 0.0
            v = 0.0
            F = np.array([[1.0, 0.0], [0.0, 0.0]])
        else:
            sdir = np.sign(v) if v != 0 else np.sign(u)
            fk_tot = pl.fk(p, sdir, v)
            if abs(v) < self.cfg.v_stick:
                fk_tot = min(fk_tot, abs(u) * 0.95)  # same unstick cap as the sim
            a = (u - sdir * fk_tot - pl.b * v) / pl.m
            F = np.array([[1.0, dt], [0.0, 1.0 - (pl.b / pl.m) * dt]])
        p_new = p + v * dt
        v_new = v + a * dt
        # hard stops: the rail is part of the plant
        if p_new <= 0.0:
            p_new, v_new = 0.0, max(v_new, 0.0)
        elif p_new >= 100.0:
            p_new, v_new = 100.0, min(v_new, 0.0)
        self.x = np.array([p_new, v_new])
        self.P = F @ self.P @ F.T + self._Q

    def update(self, z_pct: float, u: float) -> None:
        r = self.cfg.r_pwm_pct if abs(u) > 0 else self.cfg.r_quiet_pct
        H = np.array([[1.0, 0.0]])
        S = float(self.P[0, 0] + r**2)
        innov = z_pct - self.x[0]
        if self._stuck and innov * innov > self.cfg.gate_sigma**2 * S:
            # ADC spike tail (sensorfit: p-p 2-3x sigma under PWM): at rest a
            # single implausible sample is rejected, not fused. While moving
            # the gate stays open -- there large innovations are usually
            # honest model error (P does not carry the friction-map error).
            return
        K = (self.P @ H.T / S).ravel()
        self.x = self.x + K * innov
        IKH = np.eye(2) - np.outer(K, H.ravel())
        self.P = IKH @ self.P

    def step(self, u: float, z_pct: float) -> tuple[float, float]:
        self.predict(u)
        self.update(z_pct, u)
        return float(self.x[0]), float(self.x[1])


def run_on_arrays(plant: PlantParams, cfg: ObserverConfig,
                  duty: np.ndarray, pos_meas: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    kf = DutyDrivenKF(plant, cfg, p0=float(pos_meas[0]))
    n = len(duty)
    p_est = np.empty(n)
    v_est = np.empty(n)
    for i in range(n):
        p_est[i], v_est[i] = kf.step(float(duty[i]), float(pos_meas[i]))
    return p_est, v_est


def legacy_velocity(pos_meas: np.ndarray, tau_s: float = 0.150) -> np.ndarray:
    """Legacy-style causal estimate: finite difference + PT1 (tau=150 ms)."""
    fd = np.empty_like(pos_meas)
    fd[0] = 0.0
    fd[1:] = np.diff(pos_meas) * FS_HZ
    alpha = (1.0 / FS_HZ) / (tau_s + 1.0 / FS_HZ)
    out = np.empty_like(fd)
    acc = 0.0
    for i, x in enumerate(fd):
        acc += alpha * (x - acc)
        out[i] = acc
    return out


@dataclass
class ObserverScore:
    run: str
    rms_all: float           # %/s vs zero-phase reference, whole run
    rms_moving: float        # %/s where |v_ref| > 10
    lag_ms: float            # shift minimizing RMS (positive = estimate late)
    rms_all_legacy: float
    rms_moving_legacy: float
    lag_ms_legacy: float
    pos_rms: float           # % position estimate error vs filtered reference


def _best_lag(v_est: np.ndarray, v_ref: np.ndarray, max_shift: int = 120) -> tuple[float, int]:
    best = (np.inf, 0)
    n = len(v_ref)
    for k in range(-max_shift, max_shift + 1):
        if k >= 0:
            a, bb = v_est[k:], v_ref[: n - k]
        else:
            a, bb = v_est[: n + k], v_ref[-k:]
        r = float(np.sqrt(np.mean((a - bb) ** 2)))
        if r < best[0]:
            best = (r, k)
    return best[0], best[1]


def score(run: str, duty: np.ndarray, pos_meas: np.ndarray,
          plant: PlantParams, cfg: ObserverConfig,
          fc_ref: float = 40.0) -> tuple[ObserverScore, dict]:
    pos_ref, v_ref = zero_phase_velocity(pos_meas, fc=fc_ref)
    p_est, v_est = run_on_arrays(plant, cfg, duty, pos_meas)
    v_leg = legacy_velocity(pos_meas)

    moving = np.abs(v_ref) > 10.0
    def rms(x, mask=None):
        d = x if mask is None else x[mask]
        return float(np.sqrt(np.mean(d**2))) if len(d) else float("nan")

    _, lag = _best_lag(v_est, v_ref)
    _, lag_leg = _best_lag(v_leg, v_ref)
    sc = ObserverScore(
        run=run,
        rms_all=rms(v_est - v_ref),
        rms_moving=rms(v_est - v_ref, moving),
        lag_ms=lag * 1000.0 / FS_HZ,
        rms_all_legacy=rms(v_leg - v_ref),
        rms_moving_legacy=rms(v_leg - v_ref, moving),
        lag_ms_legacy=lag_leg * 1000.0 / FS_HZ,
        pos_rms=rms(p_est - pos_ref),
    )
    detail = {"pos_ref": pos_ref, "v_ref": v_ref, "p_est": p_est,
              "v_est": v_est, "v_legacy": v_leg, "moving": moving}
    return sc, detail
