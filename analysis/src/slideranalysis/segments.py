"""Campaign catalogue: load a data directory, split runs into typed segments.

The firmware records one continuous 1 kHz trace per run; ``seg`` increments at
every program-segment boundary (flag bit 1 marks the first tick). This module
gives the fitting code a uniform view:

- ``load_dir(path)``      -> [Run]      every ``*.csv`` in a session directory
- ``split_segments(run)`` -> [Segment]  per-segment slices in physical units

Positions are converted with the provisional endpoint calibration in
``loadrun`` (raw 8000 -> 0 %, raw 100 -> 100 %); duty is the signed applied
duty in [-1, 1] (``duty_milli``/1000 -- during an end-zone interlock cut the
recorded duty is the applied 0, not the commanded value).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

from .loadrun import load_run, raw_to_percent


@dataclass
class Run:
    path: Path
    meta: dict
    df: pd.DataFrame
    label: str | None = None   # e.g. "2026-07-17/02_breakaway_grid" for multi-dir

    @property
    def name(self) -> str:
        return self.label or self.path.stem

    @property
    def basename(self) -> str:
        return self.path.stem

    @property
    def experiment(self) -> str:
        return str(self.meta.get("experiment", "unknown"))

    @property
    def params(self) -> dict:
        p = self.meta.get("params", {})
        return p if isinstance(p, dict) else {}


@dataclass
class Segment:
    run: str
    experiment: str
    params: dict
    seg: int
    t: np.ndarray            # [s] since run start
    pos: np.ndarray          # [% travel]
    duty: np.ndarray         # signed, [-1, 1], as applied
    adc: np.ndarray          # raw counts (mean of 4)
    goto: bool               # any goto_active tick in this segment
    endzone_ticks: int
    index: np.ndarray = field(repr=False, default=None)  # row index into run df

    @property
    def n(self) -> int:
        return len(self.t)

    @property
    def duty_const(self) -> float | None:
        """The single nonzero duty value of this segment, or None.

        End-zone cuts record duty 0 mid-segment; those zeros are ignored when
        checking constancy (a constant-duty step that got cut still reports
        its plateau value).
        """
        nz = self.duty[self.duty != 0.0]
        if len(nz) == 0:
            return 0.0
        vals = np.unique(nz)
        if len(vals) == 1:
            return float(vals[0])
        return None

    @property
    def dpos(self) -> float:
        return float(self.pos[-1] - self.pos[0])


def load_dir(path: str | Path, label_prefix: str = "") -> list[Run]:
    """Load every lab CSV in a session directory, sorted by filename."""
    path = Path(path)
    runs: list[Run] = []
    for csv in sorted(path.glob("*.csv")):
        meta, df = load_run(csv)
        label = f"{label_prefix}{csv.stem}" if label_prefix else None
        runs.append(Run(path=csv, meta=meta, df=df, label=label))
    return runs


def load_dirs(paths: list[str | Path]) -> list[Run]:
    """Load several session directories; run names get 'dirname/' prefixes
    when more than one directory is given (basenames repeat across sessions)."""
    paths = [Path(p) for p in paths]
    if len(paths) == 1:
        return load_dir(paths[0])
    runs: list[Run] = []
    for p in paths:
        runs.extend(load_dir(p, label_prefix=f"{p.name}/"))
    return runs


def run_position(run: Run) -> np.ndarray:
    """Full-run position trace [%]."""
    return raw_to_percent(run.df["adc_raw"].to_numpy(dtype=float))


def split_segments(run: Run) -> list[Segment]:
    """Slice one run into per-segment views (physical units)."""
    df = run.df
    out: list[Segment] = []
    for seg, g in df.groupby("seg", sort=True):
        out.append(
            Segment(
                run=run.name,
                experiment=run.experiment,
                params=run.params,
                seg=int(seg),
                t=g["t_ms"].to_numpy(dtype=float) / 1000.0,
                pos=raw_to_percent(g["adc_raw"].to_numpy(dtype=float)),
                duty=g["duty_milli"].to_numpy(dtype=float) / 1000.0,
                adc=g["adc_raw"].to_numpy(dtype=float),
                goto=bool(g["goto_active"].any()),
                endzone_ticks=int(g["ilk_endzone"].sum()),
                index=g.index.to_numpy(),
            )
        )
    return out


def by_experiment(runs: list[Run]) -> dict[str, list[Run]]:
    d: dict[str, list[Run]] = {}
    for r in runs:
        d.setdefault(r.experiment, []).append(r)
    return d
