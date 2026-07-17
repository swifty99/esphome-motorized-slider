"""Controller/SIL invariant tests -- the cheap-to-pin guarantees.

The full acceptance suite lives in ``uv run acceptance``; these tests pin the
invariants that must never regress silently.
"""

import numpy as np
import pytest

from slideranalysis.controller import Controller, ControllerConfig, Mode, nominal_plant
from slideranalysis.silharness import Grab, perturbed_plant, run_move


RNG = lambda k=0: np.random.default_rng(k)


def test_nominal_move_lands_and_settles():
    res = run_move(nominal_plant(), 30.0, 70.0, RNG(1))
    assert res.settled_tick is not None
    assert abs(res.final_err) <= 0.5
    assert not res.ctrl.log.gave_up


def test_pwm_identically_zero_after_settle():
    res = run_move(nominal_plant(), 25.0, 60.0, RNG(2), rest_s=2.5)
    w = res.rest_window()
    assert w is not None and (len(res.duty) - w.start) >= 2000
    assert np.all(res.duty[w.start:] == 0.0)          # exactly 0, every tick
    assert float(np.ptp(res.pos[w.start:])) <= 0.3    # no limit cycle (truth)


def test_never_commands_creep_velocity():
    """Trajectories must never command 0 < |v_des| < the cliff floor."""
    plant = nominal_plant()
    ctrl = Controller(plant)
    assert ctrl.cfg.v_min_cmd >= 120.0
    res = run_move(plant, 20.0, 85.0, RNG(3))
    v = abs(getattr(res.ctrl, "last_v_des", 0.0))
    assert v == 0.0 or v >= res.ctrl.cfg.v_min_cmd
    # and throughout: MOVE-mode duty magnitudes sit at breakaway level or 0,
    # never in the dead 0.05..0.45 creep-command region
    move_duty = np.abs(res.duty[[m == Mode.MOVE for m in res.mode]])
    active = move_duty[move_duty > 0.01]
    assert np.all(active >= 0.45)


def test_unidirectional_final_approach():
    """The last motion-causing pulse before settle is forward -- for both
    move directions (backlash-canceling landing)."""
    for p0, tgt in ((20.0, 70.0), (80.0, 30.0)):
        res = run_move(nominal_plant(), p0, tgt, RNG(4))
        assert res.settled_tick is not None
        pre = res.duty[: res.settled_tick]
        nz = np.flatnonzero(pre)
        assert len(nz) > 0
        assert pre[nz[-1]] > 0.0, f"landed rev on move {p0}->{tgt}"


def test_grab_released_within_two_ticks_and_silent():
    grab = Grab(start_tick=140, duration=1000)
    res = run_move(perturbed_plant(RNG(5)), 20.0, 80.0, RNG(6), grab=grab)
    log = res.ctrl.log
    assert log.grab_detect_tick is not None
    detect_ms = (log.grab_detect_tick - grab.start_tick)
    assert detect_ms <= 600
    # torque identically zero from the tick after detection until release
    after = res.duty[log.grab_detect_tick + 1: grab.start_tick + grab.duration]
    assert np.all(after == 0.0)
    # and it stays silent at the adopted position afterwards
    assert np.all(res.duty[-1000:] == 0.0)


def test_hand_move_at_rest_is_adopted_silently():
    res = run_move(nominal_plant(), 40.0, 55.0, RNG(7), rest_s=0.3)
    assert res.settled_tick is not None
    # continue the loop manually: force the slider 5 % away while IDLE
    ctrl = res.ctrl
    hold0 = ctrl.hold
    for _ in range(400):
        u = ctrl.tick(hold0 + 5.0, 0.0)
        assert u == 0.0                      # never fights the hand at rest
    assert ctrl.hold == pytest.approx(hold0 + 5.0, abs=0.2)


def test_stress_band_move_completes():
    plant = perturbed_plant(RNG(8), friction_scale=(0.07, 0.07))
    res = run_move(plant, 90.0, 65.0, RNG(9))
    assert res.settled_tick is not None
    assert abs(res.final_err) <= 1.0
    assert not res.ctrl.log.gave_up
