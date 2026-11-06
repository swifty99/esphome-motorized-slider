"""Tests for the M6 (backlash) and M7 (impulse) analyzers."""

import numpy as np

from slideranalysis.backlash import backlash_pairs, collect_reversal_trials
from slideranalysis.impulses import collect_impulses, impulse_table
from slideranalysis.segments import load_dir

from synth import write_run
from test_plantfits import pos_to_adc


def test_impulse_events_settle_to_settle(tmp_path):
    # settle @50, 20 ms pulse fwd (+3 % incl. post-pulse creep), settle, pulse cut in endzone
    pos = np.concatenate([
        np.full(300, 50.0),                    # seg0 settle
        np.linspace(50, 52, 20),               # seg1 pulse 20 ms
        np.linspace(52, 53, 100), np.full(300, 53.0),  # seg2 settle (creep then rest)
        np.linspace(53, 97.5, 30),             # seg3 pulse -> endzone
        np.full(300, 97.5),                    # seg4 settle in endzone
    ])
    duty = np.concatenate([np.zeros(300), np.full(20, 0.8), np.zeros(400),
                           np.full(30, 0.8), np.zeros(300)])
    seg = np.concatenate([np.zeros(300), np.ones(20), np.full(400, 2),
                          np.full(30, 3), np.full(300, 4)])
    write_run(tmp_path / "01_impulse_ladder.csv", experiment="impulse_ladder",
              params={"duty": 0.8}, adc=pos_to_adc(pos),
              duty_milli=np.round(duty * 1000).astype(int), seg=seg)
    runs = load_dir(tmp_path)
    ev = collect_impulses(runs)
    assert len(ev) == 2
    good = [e for e in ev if not e.cut]
    assert len(good) == 1
    assert good[0].width_ms == 20
    assert good[0].travel == 3.0  # settle-to-settle, includes the creep
    rows = impulse_table(ev)
    assert rows == [(1, 20, 1, 3.0, 3.0, 3.0)]


def _reversal_run(path, p0, arrival_dir, rev_duty, dead_ms):
    """goto moves 20 % in arrival_dir to p0; settle; dummy 0.3; settle;
    test drive with dead_ms of standstill then motion."""
    n_goto, n_settle, n_dummy, n_rev = 60, 500, 300, 1000
    goto_pos = np.linspace(p0 - 20 * arrival_dir, p0, n_goto)
    rev_pos = np.concatenate([
        np.full(dead_ms, p0),
        p0 + np.sign(rev_duty) * np.linspace(0, 30, n_rev - dead_ms)])
    pos = np.concatenate([goto_pos, np.full(n_settle, p0), np.full(n_dummy, p0),
                          np.full(n_settle, p0), rev_pos, np.full(300, rev_pos[-1])])
    duty = np.concatenate([np.full(n_goto, 0.8 * arrival_dir), np.zeros(n_settle),
                           np.full(n_dummy, 0.3), np.zeros(n_settle),
                           np.full(n_rev, rev_duty), np.zeros(300)])
    seg = np.concatenate([np.zeros(n_goto), np.ones(n_settle), np.full(n_dummy, 2),
                          np.full(n_settle, 3), np.full(n_rev, 4), np.full(300, 5)])
    flags = np.concatenate([np.full(n_goto, 16), np.zeros(len(pos) - n_goto)])
    write_run(path, experiment="reversal", params={"duty": abs(rev_duty)},
              adc=pos_to_adc(pos), duty_milli=np.round(duty * 1000).astype(int),
              seg=seg, flags=flags.astype(int))


def test_backlash_pairs_delta(tmp_path):
    # arrival fwd; reversal (rev drive) has 12 ms dead time, control 7 ms
    _reversal_run(tmp_path / "01_reversal.csv", 50.0, +1, -0.75, 12)
    _reversal_run(tmp_path / "02_reversal.csv", 50.0, +1, +0.75, 7)
    runs = load_dir(tmp_path)
    trials = collect_reversal_trials(runs)
    assert len(trials) == 2
    assert sorted(t.is_reversal for t in trials) == [False, True]
    pairs = backlash_pairs(trials)
    assert len(pairs) == 1
    pos, duty, tr, tc, delta = pairs[0]
    # detector adds the same threshold-crossing offset to both -> delta exact
    assert delta == 5
