"""M7 impulse-ladder analysis: measured travel per torque pulse.

An impulse event is a short constant-duty pulse segment framed by duty-0
settle segments. Travel is measured settle-to-settle (median of the last
100 ms each) -- the slider keeps creeping for a few hundred ms after a pulse,
and that relaxation belongs to the pulse's net effect.

This table is the direct empirical basis for the open-loop impulse-move
recipe (plant-model.md §7) and the on-device impulse ident (auto-ident-spec
#8): no model in between, just what the plant did.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .segments import Run, split_segments

MAX_PULSE_MS = 150
MIN_PULSE_DUTY = 0.3
MIN_SETTLE_MS = 100


@dataclass
class ImpulseEvent:
    run: str
    seg: int
    width_ms: int
    duty: float               # signed
    pos_start: float
    travel: float             # signed, settle-to-settle [%]
    cut: bool                 # end-zone interference (travel unreliable)


def collect_impulses(runs: list[Run],
                     experiments: tuple = ("impulse_ladder",)) -> list[ImpulseEvent]:
    events: list[ImpulseEvent] = []
    for run in runs:
        if run.experiment not in experiments:
            continue
        segs = split_segments(run)
        for i in range(1, len(segs) - 1):
            pulse, before, after = segs[i], segs[i - 1], segs[i + 1]
            if pulse.goto or before.goto or after.goto:
                continue
            d = pulse.duty_const
            if d is None or abs(d) < MIN_PULSE_DUTY or pulse.n > MAX_PULSE_MS:
                continue
            if (before.duty_const != 0.0 or before.n < MIN_SETTLE_MS
                    or after.duty_const != 0.0 or after.n < MIN_SETTLE_MS):
                continue
            p0 = float(np.median(before.pos[-100:]))
            p1 = float(np.median(after.pos[-100:]))
            cut = bool(pulse.endzone_ticks or after.endzone_ticks
                       or not (3.0 < p1 < 97.0))
            events.append(ImpulseEvent(
                run=run.name, seg=pulse.seg, width_ms=pulse.n, duty=float(d),
                pos_start=p0, travel=p1 - p0, cut=cut))
    return events


def impulse_table(events: list[ImpulseEvent], include_cut: bool = False):
    """Group by (sign, width): list of rows
    (direction, width_ms, n, travel_median, travel_min, travel_max)."""
    rows = []
    ev = [e for e in events if include_cut or not e.cut]
    for direction in (+1, -1):
        widths = sorted({e.width_ms for e in ev if np.sign(e.duty) == direction})
        for w in widths:
            tr = [e.travel for e in ev
                  if e.width_ms == w and np.sign(e.duty) == direction]
            rows.append((direction, w, len(tr), float(np.median(tr)),
                         float(min(tr)), float(max(tr))))
    return rows
