"""Load one slider_lab CSV dump -> (meta: dict, df: pandas.DataFrame).

CSV contract (fixed -- the firmware is built against the same one):

- Leading ``#``-prefixed meta lines, format ``# key: value`` (first colon splits).
  Every value is parsed as JSON when possible (numbers become numbers, the
  ``params`` line becomes a dict), otherwise kept as the raw string.
  ``#`` lines without a colon are collected under ``meta["comments"]``.
  Expected keys: ``firmware``, ``experiment``, ``params``, ``tick_us_nominal``,
  ``adc``.
- Then a normal CSV header: ``t_ms,adc_raw,duty_milli,seg,flags,tick_us``
  plus optional ``s0,s1,s2,s3`` (individual ADC samples, raw-sample mode).
- ``adc_raw``    mean of 4 ADC samples (float, counts)
- ``duty_milli`` signed applied torque x1000
- ``seg``        segment index
- ``flags``      bitfield, decoded into boolean columns (see FLAG_BITS)
- ``tick_us``    measured tick execution time, nominal 1000 us

All analysis stays offline -- this loader never filters or resamples.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

FLAG_BITS = {
    "segment_start": 1,
    "ilk_endzone": 2,
    "ilk_thermal": 4,
    "abort": 8,
    "goto_active": 16,
}

SAMPLE_COLUMNS = ("s0", "s1", "s2", "s3")

# Provisional travel endpoints (device self-cal replaces these eventually).
# The pot is inverted: high counts = slider at 0 %.
RAW_AT_0PCT = 8000.0
RAW_AT_100PCT = 100.0
COUNTS_PER_PERCENT = abs(RAW_AT_100PCT - RAW_AT_0PCT) / 100.0  # 79 counts per % travel


def parse_meta_lines(lines: list[str]) -> dict:
    """Parse '#'-prefixed meta lines into a dict (values JSON-parsed if possible)."""
    meta: dict = {}
    comments: list[str] = []
    for line in lines:
        body = line.lstrip("#").strip()
        if not body:
            continue
        key, sep, value = body.partition(":")
        if not sep:
            comments.append(body)
            continue
        key = key.strip()
        value = value.strip()
        try:
            meta[key] = json.loads(value)
        except (json.JSONDecodeError, ValueError):
            meta[key] = value
    if comments:
        meta["comments"] = comments
    return meta


def decode_flags(df: pd.DataFrame) -> pd.DataFrame:
    """Add one boolean column per FLAG_BITS entry, decoded from ``flags``."""
    flags = df["flags"].astype(np.int64)
    for name, bit in FLAG_BITS.items():
        df[name] = (flags & bit) != 0
    return df


def load_run(path: str | Path) -> tuple[dict, pd.DataFrame]:
    """Load one lab CSV. Returns (meta, df) with flags decoded into bool columns."""
    path = Path(path)
    meta_lines: list[str] = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if line.startswith("#"):
                meta_lines.append(line)
            else:
                break
    meta = parse_meta_lines(meta_lines)
    df = pd.read_csv(path, skiprows=len(meta_lines))
    decode_flags(df)
    return meta, df


def sample_columns(df: pd.DataFrame) -> list[str]:
    """The s0..s3 raw-sample columns present in this run (may be empty)."""
    return [c for c in SAMPLE_COLUMNS if c in df.columns]


def raw_to_percent(raw, raw_at_0: float = RAW_AT_0PCT, raw_at_100: float = RAW_AT_100PCT):
    """ADC counts -> % of travel, linear between the provisional endpoints.

    raw 8000 -> 0 %, raw 100 -> 100 % (pot is inverted). Works on scalars,
    numpy arrays and pandas Series.
    """
    return (np.asarray(raw, dtype=float) - raw_at_0) / (raw_at_100 - raw_at_0) * 100.0


def counts_to_percent(counts):
    """Amplitude conversion (no offset): ADC counts -> % of travel."""
    return np.asarray(counts, dtype=float) / COUNTS_PER_PERCENT
