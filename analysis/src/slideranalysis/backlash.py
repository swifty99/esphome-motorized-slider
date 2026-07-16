"""M6 backlash analysis: reversal vs same-direction control dead times.

Design of the M6 runs (2026-07-17, Block 3): arrive at a position (goto sets
the arrival direction; the approach segment may be a sub-breakaway dummy),
settle, then drive at a fixed duty either AGAINST the arrival direction
(reversal -- must traverse the belt dead band first) or WITH it (control --
belt already engaged). Backlash shows as extra dead time before the pot
moves; at 1 kHz the resolution is ~1-2 ms.

Result on the lab unit: reversal-minus-control dead time is -1..+2 ms at
duty 0.75 across four positions -> no resolvable dead band; the ~6-9 ms
baseline delay is breakaway/force-buildup and appears in both cases. The
plant simulator's backlash stub (width 0) is thereby justified until finer
instrumentation (motor-side sensing, M6 with current signature) says
otherwise.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .segments import Run, split_segments


@dataclass
class ReversalTrial:
    run: str
    pos: float
    arrival: int              # +1 slider arrived moving fwd, -1 rev
    rev_dir: int              # direction of the test drive
    duty: float
    dead_time_ms: int         # -1 = never moved
    is_reversal: bool         # rev_dir != arrival

    @property
    def moved(self) -> bool:
        return self.dead_time_ms >= 0


def _dead_time(pos: np.ndarray, p0: float, direction: int,
               thresh: float = 0.5) -> int:
    med5 = np.array([np.median(pos[max(0, i - 2):i + 3]) for i in range(len(pos))])
    crossed = (med5 - p0) * direction > thresh
    idx = np.flatnonzero(crossed)
    return int(idx[0]) if len(idx) else -1


def collect_reversal_trials(runs: list[Run]) -> list[ReversalTrial]:
    trials: list[ReversalTrial] = []
    for run in runs:
        if run.experiment != "reversal":
            continue
        segs = split_segments(run)
        # test drive = last long constant-duty active segment
        rev_idx = None
        for i, s in enumerate(segs):
            d = s.duty_const
            if not s.goto and d not in (None, 0.0) and s.n >= 600:
                rev_idx = i
        if rev_idx is None or rev_idx == 0:
            continue
        rev = segs[rev_idx]
        d = rev.duty_const
        # arrival direction: last segment before the test drive that moved > 2 %
        arrival = 0
        for s in segs[:rev_idx]:
            if abs(s.dpos) > 2.0:
                arrival = 1 if s.dpos > 0 else -1
        if arrival == 0:
            continue
        prev = segs[rev_idx - 1]
        p0 = float(np.median(prev.pos[-100:])) if prev.n >= 100 else float(rev.pos[0])
        trials.append(ReversalTrial(
            run=run.name, pos=p0, arrival=arrival,
            rev_dir=1 if d > 0 else -1, duty=abs(d),
            dead_time_ms=_dead_time(rev.pos, p0, 1 if d > 0 else -1),
            is_reversal=(np.sign(d) != arrival)))
    return trials


def backlash_pairs(trials: list[ReversalTrial], pos_tol: float = 4.0):
    """Match reversal/control trials at the same position and duty.

    Returns rows (pos, duty, dead_rev_ms, dead_ctrl_ms, delta_ms) for pairs
    where both moved; unmoved trials are reported separately by the caller.
    """
    rows = []
    revs = [t for t in trials if t.is_reversal and t.moved]
    ctrls = [t for t in trials if not t.is_reversal and t.moved]
    used = set()
    for r in revs:
        best = None
        for k, c in enumerate(ctrls):
            if k in used or abs(c.pos - r.pos) > pos_tol or abs(c.duty - r.duty) > 0.05:
                continue
            if best is None or abs(c.pos - r.pos) < abs(ctrls[best].pos - r.pos):
                best = k
        if best is not None:
            used.add(best)
            c = ctrls[best]
            rows.append((r.pos, r.duty, r.dead_time_ms, c.dead_time_ms,
                         r.dead_time_ms - c.dead_time_ms))
    return rows
