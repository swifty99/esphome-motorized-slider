"""Controller prototype (Phase-3, sim-only): feedforward-dominant cascade.

Strategy per plan.md, constrained by what the identified plant actually
permits (plant-model.md):

- torque = stiction_FF(sign v_des, p) + Kv*v_des + Ka*a_des + feedback(residual)
  with the friction FF keyed to the DESIRED velocity sign -- never to a noisy
  torque or velocity estimate sign (the legacy failure).
- Trajectories NEVER command creep: the duty->velocity map is a cliff
  (plant-model 2b), so any nonzero commanded velocity is >= cfg.v_min_cmd
  (~120 %/s). Slow motion does not exist open-loop on this plant.
- Endgame: calibrated impulses from the MEASURED M7 table inside the last
  ~2 % (plus the deliberate approach margin), iterated until |err| <= tol.
- Unidirectional final approach: the main move aims BELOW the target (in the
  forward sense) and the endgame lands with forward impulses only; a rev
  correction is used solely after an overshoot, and it re-arms a forward
  approach.
- Stall escalation: the breakaway spread is +-0.04 and the 55-80 % rev band
  re-sticks against 0.7 -- when measured PROGRESS stops during a move, the
  stiction FF is boosted stepwise up to u_max. Progress is judged on the
  position estimate over a 100 ms window, NOT on v_hat: under an exogenous
  clamp the duty-driven KF's velocity stays high (the model keeps predicting
  motion; only position is measured), so v_hat is blind exactly there.
- Grab/override: a friction stall and a grabbing hand look identical to the
  observer -- what separates them is that TORQUE FIXES A STALL: the worst
  measured breakaway incl. spread is ~0.80 < u_max 0.9. Commanding u_max for
  grab_sat_ms with zero measured progress therefore cannot be friction; it is
  declared a grab, torque drops to zero within one tick, and once the slider
  is still the grabbed position becomes the new hold (fader UX). At rest a
  hand-moved slider is silently adopted (no fight, no re-trigger). While IDLE
  the output is IDENTICALLY zero -- rest silence is a latched state, not an
  equilibrium (A2/A3).

The controller runs on OBSERVER state (p_hat, v_hat, innovation), never on
sim truth. All units: % travel, %/s, duty.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from .plantsim import PlantParams

FS_HZ = 1000.0
DT = 1.0 / FS_HZ

# Nominal identified plant (snapshot of docs/findings/plant-params.json,
# 2026-07-17 fit; the harness may load the JSON instead -- this constant keeps
# tests hermetic). Duty units.
NOMINAL = dict(
    m=1.22e-05, b=5.47e-04,
    nodes=[0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100],
    fk_fwd=[0.393, 0.609, 0.568, 0.577, 0.546, 0.508, 0.576, 0.539, 0.528,
            0.532, 0.580],
    fk_rev=[0.501, 0.623, 0.554, 0.586, 0.548, 0.560, 0.591, 0.597, 0.583,
            0.718, 0.386],
    grid_s=[0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50,
            55, 60, 65, 70, 75, 80, 85, 90, 95, 100],
    fs_fwd=[0.675, 0.575, 0.600, 0.650, 0.660, 0.660, 0.660, 0.406, 0.556,
            0.556, 0.574, 0.574, 0.577, 0.577, 0.453, 0.453, 0.453, 0.532,
            0.532, 0.532, 0.550],
    fs_rev=[0.695, 0.695, 0.675, 0.675, 0.500, 0.500, 0.500, 0.629, 0.602,
            0.602, 0.611, 0.700, 0.700, 0.700, 0.720, 0.720, 0.720, 0.720,
            0.720, 0.720, 0.700],
    sigma0_counts=8.5, sigma_pwm_counts=11.3,
)


def nominal_plant() -> PlantParams:
    return PlantParams(**{k: v for k, v in NOMINAL.items()})


@dataclass
class ControllerConfig:
    # trajectory
    v_cruise: float = 180.0        # %/s, inside the validated regime
    v_min_cmd: float = 120.0       # %/s, cliff floor: never command less (nonzero)
    a_traj: float = 5000.0         # %/s^2 trajectory accel (plant can ~15000)
    # cascade gains
    kp: float = 0.05               # duty per % position-reference error
    kd: float = 2.5e-4             # duty per %/s velocity error
    e_p_clamp: float = 3.0         # % anti-windup on the reference error
    u_max: float = 0.9
    # approach / endgame
    approach_margin: float = 1.5   # % below target the main move aims (fwd sense)
    endgame_zone: float = 3.5      # % : moves shorter than this are impulse-only
    tol: float = 0.30              # % landing tolerance
    reapproach: float = 1.2        # % below target after an overshoot
    impulse_duty: float = 0.8      # the duty the M7 table was measured at
    impulse_aim: float = 0.90      # fraction of the gap to aim for (undershoot bias)
    impulse_settle_ms: int = 120   # after long pulses / brake
    impulse_settle_short: int = 70  # after pulses <= 12 ms (sim has NO post-pulse
                                    # creep; on hardware both need re-tuning up)
    impulse_max: int = 8
    grab_pulses: int = 3           # pulses with zero measured travel = grab
    # stall escalation (progress-based; see docstring on why not v_hat)
    stall_v: float = 25.0          # %/s equivalent progress threshold
    progress_win: int = 100        # ticks of the progress window
    stall_ms: int = 80
    kick_boost: float = 0.08       # added to stiction FF per escalation step
    kick_period_ms: int = 60
    # grab detection (torque-saturation principle, see module docstring)
    grab_sat_ms: int = 120         # u_max + no progress this long = grab
    release_still_ms: int = 300
    adopt_err: float = 1.5         # % : at rest, adopt a hand-moved position
    adopt_ms: int = 120
    # measured M7 impulse tables (width ms -> travel %) at duty 0.8
    fwd_widths: tuple = (2, 3, 5, 8, 10, 20, 30, 50)
    fwd_travel: tuple = (0.05, 0.20, 0.56, 1.75, 2.64, 6.21, 10.26, 21.66)
    rev_widths: tuple = (3, 5, 10, 20, 30, 50)
    rev_travel: tuple = (0.28, 0.62, 1.02, 4.48, 8.15, 20.08)


class Mode(Enum):
    IDLE = "idle"
    MOVE = "move"
    BRAKE = "brake"
    PULSE = "pulse"
    WAIT = "wait"
    GRABBED = "grabbed"


@dataclass
class ControllerLog:
    kicks: int = 0
    pulses: int = 0
    overshoots: int = 0
    removes: int = 0               # endgame found itself far off -> re-armed MOVE
    gave_up: bool = False
    grab_detect_tick: int | None = None
    grab_release_tick: int | None = None
    settled_tick: int | None = None


class Controller:
    """1 kHz cascade controller. Call ``tick(p_hat, v_hat, innov)`` -> duty."""

    def __init__(self, plant: PlantParams | None = None,
                 cfg: ControllerConfig | None = None):
        self.plant = plant or nominal_plant()
        self.cfg = cfg or ControllerConfig()
        self.mode = Mode.IDLE
        self.target: float | None = None
        self.hold: float | None = None
        self.log = ControllerLog()
        self._t = 0                    # tick counter
        self._p_ref = 0.0
        self._v_des_prev = 0.0
        self._dir = 1
        self._pulse_left = 0
        self._pulse_dir = 1
        self._wait_left = 0
        self._stall_ticks = 0
        self._kick = 0.0
        self._kick_cool = 0
        self._sat_ticks = 0
        self._still_ticks = 0
        self._adopt_ticks = 0
        self._last_pulse_dir = 0
        self._last_width = 0
        self._p_at_pulse: float | None = None
        self._nomove_pulses = 0
        self._waypoint = 0.0
        self._ph_hist: list[float] = []

    # ------------------------------------------------------------- API ----
    def set_target(self, target: float, p_hat: float) -> None:
        cfg = self.cfg
        self.target = float(np.clip(target, 4.0, 96.0))
        self.hold = None
        self.log = ControllerLog()
        err = self.target - p_hat
        if abs(err) <= cfg.tol:
            self._settle(p_hat)
            return
        if abs(err) <= cfg.endgame_zone:
            # short move: impulse-only (a rev short move starts with one rev
            # pulse below the target, then lands forward -- by design, not
            # counted as an overshoot)
            self._enter_wait(1)
            return
        # main move: aim below the target (unidirectional forward landing)
        self._restart_move(p_hat)

    @property
    def settled(self) -> bool:
        return self.mode == Mode.IDLE and self.hold is not None

    # ------------------------------------------------------------ tick ----
    def tick(self, p_hat: float, v_hat: float, innov: float = 0.0) -> float:
        self._t += 1
        cfg = self.cfg

        if self.mode == Mode.IDLE:
            # touchable at rest: silently adopt a hand-moved position
            if self.hold is not None and abs(p_hat - self.hold) > cfg.adopt_err:
                self._adopt_ticks += 1
                if self._adopt_ticks >= cfg.adopt_ms:
                    self.hold = p_hat
                    self._adopt_ticks = 0
            else:
                self._adopt_ticks = 0
            return 0.0

        if self.mode == Mode.GRABBED:
            # torque released; wait for stillness, then adopt the position
            if abs(v_hat) < 5.0:
                self._still_ticks += 1
            else:
                self._still_ticks = 0
            if self._still_ticks >= cfg.release_still_ms:
                self.log.grab_release_tick = self._t
                self._settle(p_hat)
            return 0.0

        if self.mode == Mode.MOVE:
            return self._tick_move(p_hat, v_hat)
        if self.mode == Mode.BRAKE:
            if abs(v_hat) < cfg.stall_v:
                self._enter_wait(cfg.impulse_settle_ms)
            return 0.0
        if self.mode == Mode.PULSE:
            self._pulse_left -= 1
            if self._pulse_left <= 0:
                self._enter_wait(cfg.impulse_settle_ms if self._last_width > 12
                                 else cfg.impulse_settle_short)
            return cfg.impulse_duty * self._pulse_dir
        if self.mode == Mode.WAIT:
            self._wait_left -= 1
            if self._wait_left <= 0:
                self._endgame_decide(p_hat)
            return 0.0
        return 0.0

    # ------------------------------------------------------- internals ----
    def _settle(self, p_hat: float) -> None:
        self.mode = Mode.IDLE
        self.hold = p_hat
        if self.log.settled_tick is None:
            self.log.settled_tick = self._t

    def _enter_wait(self, ticks: int) -> None:
        self.mode = Mode.WAIT
        self._wait_left = max(int(ticks), 1)

    def _tick_move(self, p_hat: float, v_hat: float) -> float:
        cfg = self.cfg
        plant = self.plant
        d = self._dir
        remaining = (self._waypoint - p_hat) * d

        # stop anticipation: brake distance at current speed + 3 ticks of lag
        stop_dist = (v_hat**2) * plant.m / (2.0 * 0.55) + abs(v_hat) * 0.003
        if remaining <= max(stop_dist, 0.15):
            self.mode = Mode.BRAKE
            return 0.0

        # velocity schedule with the cliff floor: never command creep
        v_des = d * float(np.clip(np.sqrt(2.0 * cfg.a_traj * max(remaining, 0.0)),
                                  cfg.v_min_cmd, cfg.v_cruise))
        self.last_v_des = v_des     # exposed for the no-creep invariant test
        a_des = float(np.clip((v_des - self._v_des_prev) / DT,
                              -cfg.a_traj * 3, cfg.a_traj * 3))
        self._v_des_prev = v_des

        # reference integration + anti-windup freeze
        e_p = self._p_ref - p_hat
        if abs(e_p) < cfg.e_p_clamp:
            self._p_ref += v_des * DT
        e_p = float(np.clip(self._p_ref - p_hat, -cfg.e_p_clamp, cfg.e_p_clamp))

        # stall detection on measured PROGRESS (p_hat window), never on v_hat:
        # under an exogenous clamp the duty-driven KF velocity stays high
        self._ph_hist.append(p_hat)
        if len(self._ph_hist) > cfg.progress_win:
            self._ph_hist.pop(0)
        win_full = len(self._ph_hist) == cfg.progress_win
        progress = (abs(self._ph_hist[-1] - self._ph_hist[0]) * FS_HZ
                    / cfg.progress_win) if win_full else cfg.stall_v + 1.0
        stalled = progress < cfg.stall_v
        if stalled:
            self._stall_ticks += 1
        else:
            self._stall_ticks = 0
            self._kick = 0.0
            self._sat_ticks = 0
        self._kick_cool = max(0, self._kick_cool - 1)
        if self._stall_ticks >= cfg.stall_ms and self._kick_cool == 0:
            self._kick += cfg.kick_boost
            self._kick_cool = cfg.kick_period_ms
            self.log.kicks += 1

        # feedforward: stiction keyed to DESIRED direction; Kv=b; Ka=m
        if stalled or abs(v_hat) < cfg.stall_v:
            f_ff = plant.fs(p_hat, d) + self._kick
        else:
            f_ff = plant.fk(p_hat, d)
        u_raw = (d * f_ff + plant.b * v_des + plant.m * a_des
                 + cfg.kp * e_p + cfg.kd * (v_des - v_hat))
        u = float(np.clip(u_raw, -cfg.u_max, cfg.u_max))

        # grab: full torque with zero progress cannot be friction (worst
        # measured breakaway incl. spread ~0.80 < u_max)
        if stalled and abs(u_raw) >= cfg.u_max:
            self._sat_ticks += 1
            if self._sat_ticks >= cfg.grab_sat_ms:
                self.mode = Mode.GRABBED
                self.log.grab_detect_tick = self._t
                self._still_ticks = 0
                return 0.0
        else:
            self._sat_ticks = 0
        return u

    def _pulse_width(self, travel: float, direction: int) -> int:
        cfg = self.cfg
        if direction > 0:
            w = np.interp(travel, cfg.fwd_travel, cfg.fwd_widths)
            return int(np.clip(round(w), cfg.fwd_widths[0], cfg.fwd_widths[-1]))
        w = np.interp(travel, cfg.rev_travel, cfg.rev_widths)
        return int(np.clip(round(w), cfg.rev_widths[0], cfg.rev_widths[-1]))

    def _restart_move(self, p_hat: float) -> None:
        cfg = self.cfg
        self._waypoint = self.target - cfg.approach_margin
        self._dir = 1 if self._waypoint >= p_hat else -1
        self._p_ref = p_hat
        self._v_des_prev = 0.0
        self._kick = 0.0
        self._stall_ticks = 0
        self._sat_ticks = 0
        self._ph_hist = []
        self.mode = Mode.MOVE

    def _endgame_decide(self, p_hat: float) -> None:
        cfg = self.cfg
        err = self.target - p_hat          # >0: forward pulse needed
        # grab detection in the endgame: M7 proved 0.8-duty pulses move the
        # slider at every position/direction -- N pulses with zero measured
        # travel mean something is holding it
        if self._p_at_pulse is not None:
            if abs(p_hat - self._p_at_pulse) < 0.15:
                self._nomove_pulses += 1
                if self._nomove_pulses >= cfg.grab_pulses:
                    self.mode = Mode.GRABBED
                    self.log.grab_detect_tick = self._t
                    self._still_ticks = 0
                    self._nomove_pulses = 0
                    self._p_at_pulse = None
                    return
            else:
                self._nomove_pulses = 0
        if abs(err) <= cfg.tol:
            self._settle(p_hat)
            return
        if self.log.pulses >= cfg.impulse_max or self.log.removes > 3:
            self.log.gave_up = True
            self._settle(p_hat)
            return
        if abs(err) > cfg.endgame_zone + 2.0:
            # far off (escalation breakthrough overshoot etc.): full re-approach
            self.log.removes += 1
            self._restart_move(p_hat)
            return
        if err > 0:
            width = self._pulse_width(err * cfg.impulse_aim, +1)
            self._pulse_dir = +1
        else:
            # went past the target: back BELOW it, re-arm the forward approach.
            # Counted as an overshoot only if a forward pulse caused it (a rev
            # short move legitimately starts with a rev pulse).
            if self._last_pulse_dir == +1:
                self.log.overshoots += 1
            width = self._pulse_width((-err + cfg.reapproach) * cfg.impulse_aim, -1)
            self._pulse_dir = -1
        self._pulse_left = width
        self._last_width = width
        self._last_pulse_dir = self._pulse_dir
        self._p_at_pulse = p_hat
        self.log.pulses += 1
        self.mode = Mode.PULSE
