"""Zero-phase offline velocity estimation -- the ground-truth reference.

Position first goes through a zero-phase Butterworth low-pass (``filtfilt``,
so no group delay), then velocity/acceleration come from central differences
(``np.gradient``). This is strictly offline (non-causal): it is the reference
that causal estimators (the observer, the legacy PT1 filter) are judged
against, never something the device could run in real time.

Default cutoff 40 Hz: the fastest transient in the 2026-07-16 data reaches
quasi-steady speed in ~40-60 ms, so 40 Hz keeps the transient shape while
killing the ~0.1 % sensor noise that would otherwise dominate a 1 kHz
finite difference (sigma_v = sigma_p * sqrt(2) * fs ~ 145 %/s raw).
"""

from __future__ import annotations

import numpy as np
from scipy import signal

FS_HZ = 1000.0  # recorder tick rate
DEFAULT_FC_HZ = 40.0


def zero_phase_lowpass(x, fs: float = FS_HZ, fc: float = DEFAULT_FC_HZ) -> np.ndarray:
    """Zero-phase 2nd-order Butterworth low-pass of ``x`` (1-D)."""
    x = np.asarray(x, dtype=float)
    if len(x) < 15:  # too short for filtfilt padding -- return unfiltered
        return x.copy()
    b, a = signal.butter(2, fc / (fs / 2.0))
    return signal.filtfilt(b, a, x)


def zero_phase_velocity(
    pos, fs: float = FS_HZ, fc: float = DEFAULT_FC_HZ
) -> tuple[np.ndarray, np.ndarray]:
    """(filtered position, velocity) from a position trace [%].

    Velocity is in %/s. Zero phase: peaks in v line up with the true motion,
    no lag -- but the first/last ~1/fc seconds carry filter edge effects.
    """
    pf = zero_phase_lowpass(pos, fs=fs, fc=fc)
    v = np.gradient(pf) * fs
    return pf, v


def zero_phase_accel(v, fs: float = FS_HZ, fc: float = DEFAULT_FC_HZ) -> np.ndarray:
    """Acceleration [%/s^2] from a velocity trace, same zero-phase smoothing.

    NOTE: two cascaded low-passes smear fast transients; accelerations of
    events shorter than ~1/fc are systematically underestimated. Fits that
    use this (the inertia estimate) must state that bias.
    """
    vf = zero_phase_lowpass(v, fs=fs, fc=fc)
    return np.gradient(vf) * fs


def edge_mask(n: int, fs: float = FS_HZ, fc: float = DEFAULT_FC_HZ, factor: float = 2.0) -> np.ndarray:
    """Boolean mask that is False inside the filter edge zones (~factor/fc)."""
    k = int(factor * fs / fc)
    m = np.ones(n, dtype=bool)
    m[: min(k, n)] = False
    m[max(0, n - k):] = False
    return m
