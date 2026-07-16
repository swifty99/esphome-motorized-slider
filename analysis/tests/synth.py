"""Synthetic slider_lab CSV generator -- mirrors the firmware CSV contract.

Used by the tests (and handy for CLI demos). Deliberately writes real files
through the same text format the firmware emits, so the loader is exercised
end to end instead of against in-memory frames.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

HEADER = ["t_ms", "adc_raw", "duty_milli", "seg", "flags", "tick_us"]
SAMPLE_HEADER = ["s0", "s1", "s2", "s3"]


def _broadcast(value, n: int, dtype=int) -> np.ndarray:
    arr = np.asarray(value)
    if arr.ndim == 0:
        return np.full(n, arr, dtype=dtype)
    assert len(arr) == n, f"length mismatch: {len(arr)} != {n}"
    return arr.astype(dtype)


def write_run(
    path: str | Path,
    *,
    n_ticks: int = 1000,
    experiment: str = "noise_floor",
    params: dict | None = None,
    adc=None,
    samples=None,
    duty_milli=0,
    seg=0,
    flags=0,
    tick_us=None,
    tick_us_nominal: int = 1000,
    gap_after: int | None = None,
    gap_dt_ms: int = 3,
    duplicate_at: int | None = None,
    seed: int = 0,
) -> Path:
    """Write one synthetic lab CSV. Returns the path.

    - ``samples``: (n, 4) array -> raw-sample mode; adc_raw = row mean.
    - ``adc``: (n,) array of adc_raw values (ignored if samples given).
    - default: white noise around 4000 counts.
    - ``gap_after``: index i such that t_ms[i+1] - t_ms[i] == gap_dt_ms
      (a gap_dt_ms of 3 seeds a 3 ms gap = 2 missed ticks).
    - ``duplicate_at``: index i such that t_ms[i] == t_ms[i-1].
    """
    rng = np.random.default_rng(seed)

    if samples is not None:
        samples = np.asarray(samples, dtype=float)
        n_ticks = len(samples)
        adc = samples.mean(axis=1)
    elif adc is not None:
        adc = np.asarray(adc, dtype=float)
        n_ticks = len(adc)
    else:
        adc = 4000.0 + rng.normal(0.0, 3.0, n_ticks)

    t_ms = np.arange(n_ticks, dtype=np.int64)
    if gap_after is not None:
        t_ms[gap_after + 1:] += gap_dt_ms - 1
    if duplicate_at is not None:
        t_ms[duplicate_at] = t_ms[duplicate_at - 1]

    if tick_us is None:
        # realistic tick WORK DURATIONS: ~280 us typical (200-400 us range),
        # comfortably under the 1000 us tick budget
        tick_us = rng.normal(280.0, 30.0, n_ticks)
    tick_us = _broadcast(tick_us, n_ticks, dtype=float)
    duty_milli = _broadcast(duty_milli, n_ticks)
    seg = _broadcast(seg, n_ticks)
    flags = _broadcast(flags, n_ticks)

    path = Path(path)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(f"# firmware: slider_lab 0.0.0-synth (2026-07-16)\n")
        fh.write(f"# experiment: {experiment}\n")
        fh.write(f"# params: {json.dumps(params if params is not None else {})}\n")
        fh.write(f"# tick_us_nominal: {tick_us_nominal}\n")
        fh.write(f"# adc: oneshot 13bit n=4 avg\n")
        header = HEADER + (SAMPLE_HEADER if samples is not None else [])
        fh.write(",".join(header) + "\n")
        for i in range(n_ticks):
            row = [
                str(int(t_ms[i])),
                f"{adc[i]:.2f}",
                str(int(duty_milli[i])),
                str(int(seg[i])),
                str(int(flags[i])),
                f"{tick_us[i]:.1f}",
            ]
            if samples is not None:
                row += [str(int(v)) for v in samples[i]]
            fh.write(",".join(row) + "\n")
    return path
