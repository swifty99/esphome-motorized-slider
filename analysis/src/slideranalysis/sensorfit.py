"""Sensor model fit: noise sigma, quantization, PWM-coupling term vs duty.

Inputs (whatever the session directory offers -- re-run when new CSVs land):

- ``noise_floor`` runs (motor unpowered, ideally raw-sample mode s0..s3)
  -> baseline sigma, intra-tick sample spread, quantization check.
- Stalled segments under PWM (``pwm_coupling`` runs, ``duty_ladder`` rungs
  below breakaway, stalled ``step_response`` holds)
  -> noise vs |duty| = the coupling term.

Model (counts refer to ``adc_raw`` = mean of 4 raw 13-bit samples):

    sigma_adc(d) = sqrt(sigma0^2 + c * |d|)      d = |duty|, 0..1
    plus a heavy tail under PWM: p-p grows ~2-3x while sigma grows ~1.4x
    (spikes, not broadband). ``spike`` stats quantify the tail separately.

Quantization: raw samples are integers (1 LSB); adc_raw resolves 0.25 counts.
sigma0_sample ~ 16 counts >> 1 LSB, so quantization noise (1/sqrt(12) LSB)
is negligible against thermal/track noise -- reported, not modeled.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .loadrun import COUNTS_PER_PERCENT, sample_columns
from .segments import Run, Segment, split_segments

STALL_DPOS_PCT = 0.6     # segment counts as stalled if |net move| below this
MIN_STALL_TICKS = 500    # and it lasted at least this many ticks


@dataclass
class NoisePoint:
    run: str
    seg: int
    duty_abs: float
    pos_pct: float
    n: int
    sigma_counts: float       # std of adc_raw, linearly detrended
    ptp_counts: float
    spike_frac: float         # fraction of ticks with |dev| > 5*sigma0


@dataclass
class SensorModel:
    counts_per_percent: float
    # unpowered baseline
    sigma0_counts: float
    sigma0_ci: tuple[float, float]
    sigma0_sample_counts: float | None    # per raw sample, if s0..s3 present
    intra_tick_sigma_counts: float | None
    # PWM coupling: sigma^2 = sigma0^2 + c*|d|
    coupling_c_counts2: float | None
    coupling_c_ci: tuple[float, float] | None
    sigma_pwm_counts: float | None        # pooled sigma over all stalled-PWM points
    ptp_pwm_counts: float | None          # typical (median) p-p under PWM
    ptp0_counts: float | None             # typical p-p unpowered
    quantization_lsb_counts: float = 1.0
    notes: list[str] = field(default_factory=list)

    def sigma_counts(self, duty_abs: float) -> float:
        """Model sigma of adc_raw at |duty| (counts)."""
        if duty_abs <= 0.0 or self.coupling_c_counts2 is None:
            base = self.sigma0_counts
            if duty_abs > 0.0 and self.sigma_pwm_counts is not None:
                return self.sigma_pwm_counts
            return base
        return float(np.sqrt(self.sigma0_counts**2 + self.coupling_c_counts2 * duty_abs))

    def sigma_pct(self, duty_abs: float) -> float:
        return self.sigma_counts(duty_abs) / self.counts_per_percent


def _detrended_sigma(adc: np.ndarray, t: np.ndarray) -> float:
    tt = t - t[0]
    fit = np.polyval(np.polyfit(tt, adc, 1), tt)
    return float((adc - fit).std())


def collect_noise_points(runs: list[Run], sigma0_hint: float = 8.5) -> list[NoisePoint]:
    """Stalled/unpowered segments -> (duty, sigma) points.

    Only segments with constant duty, >= MIN_STALL_TICKS and net movement
    < STALL_DPOS_PCT qualify (a moving trace would masquerade as noise).
    """
    pts: list[NoisePoint] = []
    for run in runs:
        if run.experiment not in ("noise_floor", "pwm_coupling", "duty_ladder",
                                  "step_response", "breakaway_grid", "reversal"):
            continue
        for s in split_segments(run):
            if s.goto or s.n < MIN_STALL_TICKS:
                continue
            d = s.duty_const
            if d is None or abs(s.dpos) > STALL_DPOS_PCT:
                continue
            if float(np.ptp(s.pos)) > 4.0 * STALL_DPOS_PCT:  # excursions = motion
                continue
            sigma = _detrended_sigma(s.adc, s.t)
            dev = np.abs(s.adc - np.median(s.adc))
            pts.append(
                NoisePoint(
                    run=s.run, seg=s.seg, duty_abs=abs(d),
                    pos_pct=float(np.median(s.pos)), n=s.n,
                    sigma_counts=sigma, ptp_counts=float(np.ptp(s.adc)),
                    spike_frac=float(np.mean(dev > 5.0 * sigma0_hint)),
                )
            )
    return pts


def fit_sensor(runs: list[Run]) -> tuple[SensorModel, list[NoisePoint]]:
    """Fit the sensor model from a session's runs. Honest about what it saw."""
    pts = collect_noise_points(runs)
    notes: list[str] = []

    quiet = [p for p in pts if p.duty_abs == 0.0]
    powered = [p for p in pts if p.duty_abs > 0.0]

    if not quiet:
        raise ValueError("no unpowered stalled segments found -- cannot fit sigma0")

    s0_vals = np.array([p.sigma_counts for p in quiet])
    sigma0 = float(np.sqrt(np.mean(s0_vals**2)))
    # crude CI from spread across segments (few segments -> wide and honest)
    lo, hi = float(s0_vals.min()), float(s0_vals.max())

    # raw-sample stats from noise_floor runs that carry s0..s3
    sig_sample = intra = None
    samp = []
    intra_l = []
    for run in runs:
        if run.experiment != "noise_floor":
            continue
        cols = sample_columns(run.df)
        if len(cols) == 4:
            s = run.df[cols].to_numpy(dtype=float)
            samp.append(float(s.std()))
            intra_l.append(float(s.std(axis=1).mean()))
    if samp:
        sig_sample = float(np.mean(samp))
        intra = float(np.mean(intra_l))

    c = c_ci = None
    sigma_pwm = ptp_pwm = None
    if powered:
        dd = np.array([p.duty_abs for p in powered])
        var_excess = np.array([p.sigma_counts**2 for p in powered]) - sigma0**2
        # linear through origin: var_excess = c * d, LSQ with covariance
        X = dd[:, None]
        theta, *_ = np.linalg.lstsq(X, var_excess, rcond=None)
        c = float(theta[0])
        resid = var_excess - X[:, 0] * c
        if len(dd) > 1:
            se = float(np.sqrt(np.sum(resid**2) / (len(dd) - 1) / np.sum(dd**2)))
            c_ci = (c - 2 * se, c + 2 * se)
            if c_ci[0] < 0:
                notes.append(
                    "coupling slope c not significant (CI crosses 0): sigma rises "
                    "only weakly with duty; the dominant PWM effect is the spike "
                    "tail (p-p 2-3x), not broadband noise"
                )
        sigma_pwm = float(np.sqrt(np.mean([p.sigma_counts**2 for p in powered])))
        ptp_pwm = float(np.median([p.ptp_counts for p in powered]))
        if c < 0:
            c = None
            c_ci = None
            notes.append("negative coupling slope fitted -> dropped; using pooled sigma_pwm")

    ptp0 = float(np.median([p.ptp_counts for p in quiet])) if quiet else None

    model = SensorModel(
        counts_per_percent=COUNTS_PER_PERCENT,
        sigma0_counts=sigma0,
        sigma0_ci=(lo, hi),
        sigma0_sample_counts=sig_sample,
        intra_tick_sigma_counts=intra,
        coupling_c_counts2=c,
        coupling_c_ci=c_ci,
        sigma_pwm_counts=sigma_pwm,
        ptp_pwm_counts=ptp_pwm,
        ptp0_counts=ptp0,
        notes=notes,
    )
    return model, pts
