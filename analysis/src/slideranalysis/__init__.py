"""Analysis tooling for the motorslider lab rig (slider_lab CSV dumps).

Phase-3 modeling stack (see docs/plant-model.md):
``segments`` (catalogue) -> ``sensorfit``/``frictionfit`` (fits with
covariance) -> ``plantsim`` (validated simulator) -> ``observer`` (KF
prototype); ``plantreport`` is the re-runnable CLI over session directories.
"""

from .loadrun import (
    COUNTS_PER_PERCENT,
    FLAG_BITS,
    counts_to_percent,
    load_run,
    raw_to_percent,
    sample_columns,
)
from .plantsim import PlantParams, simulate
from .segments import Run, Segment, load_dir, load_dirs, split_segments
from .velocity import zero_phase_velocity

__all__ = [
    "COUNTS_PER_PERCENT",
    "FLAG_BITS",
    "PlantParams",
    "Run",
    "Segment",
    "counts_to_percent",
    "load_dir",
    "load_dirs",
    "load_run",
    "raw_to_percent",
    "sample_columns",
    "simulate",
    "split_segments",
    "zero_phase_velocity",
]
