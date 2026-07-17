// L3/L4 implementation. Source of truth: docs/controller-spec.md §1–§5.
// Behavior numerics validated by the Phase-3 prototype (analysis/.../controller.py);
// CRAWL added per Gate answer 1 (§2 rows 26–29).

#include "controller.h"

#include <cmath>

#include "plant_model.h"

namespace slider_servo {

namespace {
inline float clipf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
inline float signf(float x) { return x > 0 ? 1.0f : (x < 0 ? -1.0f : 0.0f); }
}  // namespace

void Controller::init(const UnitCalib *calib, const ServoConfig *cfg, float p0,
                      SeededBug bug) {
  calib_ = calib;
  cfg_ = cfg;
  bug_ = bug;
  state_ = State::IDLE;
  hold_ = p0;
  has_hold_ = true;
  t_ = 0;
  log_ = ControllerLog{};
  settle_flag_ = SettleFlag::NONE;
}

// --------------------------------------------------------------- commands --

void Controller::set_target(float target, float p_hat, float v_req) {
  const float clamped = clipf(target, calib_->span_lo, calib_->span_hi);
  const bool was_clamped = clamped != target;

  if (state_ == State::RELEASED) {  // row 22: cannot move against a hand
    pending_ = true;
    pending_target_ = clamped;
    pending_v_req_ = v_req;
    return;
  }

  target_ = clamped;
  has_target_ = true;
  log_ = ControllerLog{};
  log_.target_clamped = was_clamped;
  settle_flag_ = SettleFlag::NONE;
  endgame_pulses_ = 0;
  last_pulse_dir_ = 0;  // no motion under this command yet

  // normalize the speed request (spec §5: gap rounds up to a fast move)
  v_req_ = 0.0f;
  if (v_req > 0.0f && v_req <= cfg_->v_crawl_max)
    v_req_ = clipf(v_req, 0.5f, cfg_->v_crawl_max);

  arm_watchdog(std::fabs(target_ - p_hat));

  if (state_ != State::IDLE) {
    // rows 8/17: retarget mid-motion -> BRAKE, replan at stillness (9 -> 10)
    state_ = State::BRAKE;
    return;
  }

  const float err = target_ - p_hat;
  if (std::fabs(err) <= cfg_->tol) {  // row 1: re-latch
    settle(p_hat, SettleFlag::REACHED);
    return;
  }
  if (std::fabs(err) <= cfg_->endgame_zone) {  // row 2: impulse-only move
    p_at_pulse_valid_ = false;
    nomove_pulses_ = 0;
    enter_wait(1);
    return;
  }
  restart_motion(p_hat);  // rows 3 / 26
}

void Controller::release(ReleaseReason reason) {
  state_ = State::RELEASED;
  release_reason_ = reason;
  still_ticks_ = 0;
  watchdog_left_ = -1;
  has_target_ = false;
}

void Controller::notify_interlock(bool clamped) {
  // row 18: a persistent L1 clamp is not fought, it releases
  if (clamped && (state_ == State::MOVE || state_ == State::PULSE ||
                  state_ == State::CRAWL)) {
    ilk_ticks_++;
    if (ilk_ticks_ > cfg_->ilk_yield_ms) release(ReleaseReason::INTERLOCK);
  } else {
    ilk_ticks_ = 0;
  }
}

void Controller::declare_grab() {
  log_.grab_detect_tick = t_;
  release(ReleaseReason::GRAB);
}

// ------------------------------------------------------------------- tick --

float Controller::tick(float p_hat, float v_hat) {
  t_++;
  if (state_ != State::IDLE && state_ != State::RELEASED &&
      watchdog_left_ >= 0 && !SIL_BUG(bug_, T9_NO_WATCHDOG)) {
    if (--watchdog_left_ <= 0) {  // row 19
      log_.watchdog_tripped = true;
      release(ReleaseReason::WATCHDOG);
      return 0.0f;
    }
  }
  switch (state_) {
    case State::IDLE:
      return tick_idle(p_hat);
    case State::RELEASED:
      return tick_released(p_hat, v_hat);
    case State::MOVE:
      return tick_move(p_hat, v_hat);
    case State::BRAKE:
      return tick_brake(v_hat);
    case State::PULSE:
      return tick_pulse();
    case State::WAIT:
      return tick_wait(p_hat);
    case State::CRAWL:
      return tick_crawl(p_hat);
  }
  return 0.0f;
}

float Controller::tick_idle(float p_hat) {
  // row 4: hand move at rest is adopted silently, never fought
  if (has_hold_ && std::fabs(p_hat - hold_) > cfg_->adopt_err) {
    adopt_ticks_++;
    if (adopt_ticks_ >= cfg_->adopt_ms) {
      hold_ = p_hat;
      settle_flag_ = SettleFlag::ADOPTED;
      adopt_ticks_ = 0;
    }
  } else {
    adopt_ticks_ = 0;
  }
  if (SIL_BUG(bug_, T1_HOLD_TERM))
    return 0.02f * (hold_ - p_hat);  // the forbidden P-hold (T1 negative control)
  return 0.0f;
}

float Controller::tick_released(float p_hat, float v_hat) {
  if (std::fabs(v_hat) < cfg_->still_v)
    still_ticks_++;
  else
    still_ticks_ = 0;
  if (still_ticks_ >= cfg_->release_still_ms) {
    if (pending_) {  // row 21: execute the stored command
      pending_ = false;
      state_ = State::IDLE;  // re-enter via the normal command rules
      release_reason_ = ReleaseReason::NONE;
      set_target(pending_target_, p_hat, pending_v_req_);
    } else {  // row 20: adopt
      release_reason_ = ReleaseReason::NONE;
      settle(p_hat, SettleFlag::ADOPTED);
    }
    return 0.0f;
  }
  if (SIL_BUG(bug_, T6_NO_RELEASE) && release_reason_ == ReleaseReason::GRAB)
    return 0.6f * static_cast<float>(dir_);  // fights the hand (T6 negative)
  return 0.0f;
}

bool Controller::progress_update(float p_hat) {
  const int win = cfg_->progress_win < PROGRESS_WIN_MAX ? cfg_->progress_win
                                                        : PROGRESS_WIN_MAX;
  ph_ring_[ph_head_] = p_hat;
  ph_head_ = (ph_head_ + 1) % win;
  if (ph_count_ < win) {
    ph_count_++;
    return false;  // window not full yet: not stalled (prototype semantics)
  }
  const float oldest = ph_ring_[ph_head_];
  const float progress = std::fabs(p_hat - oldest) * FS_HZ / static_cast<float>(win);
  return progress < cfg_->stall_v;
}

float Controller::tick_move(float p_hat, float v_hat) {
  const ServoConfig &cfg = *cfg_;
  const UnitCalib &ucal = *calib_;
  const float d = static_cast<float>(dir_);
  const float remaining = (waypoint_ - p_hat) * d;

  // stop anticipation: brake distance at v_hat (brake decel ~= typical F_k
  // 0.55 duty acting through m) + 3 ticks of pipeline lag
  const float stop_dist =
      v_hat * v_hat * ucal.m / (2.0f * 0.55f) + std::fabs(v_hat) * 0.003f;
  if (remaining <= (stop_dist > 0.15f ? stop_dist : 0.15f)) {  // row 5
    state_ = State::BRAKE;
    last_v_des_ = 0.0f;
    return 0.0f;
  }

  // velocity schedule with the cliff floor (§4 forbidden #3): never creep
  const float v_floor = SIL_BUG(bug_, T2_LOW_VMIN) ? 30.0f : cfg.v_min_cmd;
  float v_mag = std::sqrt(2.0f * cfg.a_traj * remaining);
  const float v_des = d * clipf(v_mag, v_floor, cfg.v_cruise);
  last_v_des_ = v_des;
  const float a_des =
      clipf((v_des - v_des_prev_) / DT, -3.0f * cfg.a_traj, 3.0f * cfg.a_traj);
  v_des_prev_ = v_des;

  // reference integration with clamp + freeze (the only I-action, §4 #8)
  float e_p = p_ref_ - p_hat;
  if (std::fabs(e_p) < cfg.e_p_clamp) p_ref_ += v_des * DT;
  e_p = clipf(p_ref_ - p_hat, -cfg.e_p_clamp, cfg.e_p_clamp);

  // stall on measured PROGRESS, never v_hat (row 6 / T4): under an exogenous
  // clamp the duty-driven KF keeps predicting motion — v_hat is blind there
  bool stalled = progress_update(p_hat);
  if (SIL_BUG(bug_, T4_STALL_FROM_VHAT)) stalled = std::fabs(v_hat) < cfg.stall_v;
  if (stalled) {
    stall_ticks_++;
  } else {
    stall_ticks_ = 0;
    kick_ = 0.0f;
    sat_ticks_ = 0;
  }
  if (kick_cool_ > 0) kick_cool_--;
  if (stall_ticks_ >= cfg.stall_ms && kick_cool_ == 0) {
    kick_ += cfg.kick_boost;
    kick_cool_ = cfg.kick_period_ms;
    log_.kicks++;
  }

  // feedforward keyed to DESIRED direction only (§4 forbidden #2, T3)
  float f_ff;
  if (stalled || std::fabs(v_hat) < cfg.stall_v)
    f_ff = fs_map(ucal, p_hat, d) + kick_;
  else
    f_ff = fk_map(ucal, p_hat, d);
  float ff_dir = d;
  if (SIL_BUG(bug_, T3_FF_TORQUE_SIGN)) ff_dir = u_prev_sign_;  // the legacy bug
  last_ff_ = ff_dir * f_ff;

  const float u_raw = ff_dir * f_ff + ucal.b * v_des + ucal.m * a_des +
                      cfg.kp * e_p + cfg.kd * (v_des - v_hat);
  float u = clipf(u_raw, -cfg.u_max, cfg.u_max);
  if (SIL_BUG(bug_, T11_KICK_UNCAPPED) && kick_ > 0.0f) u = u_raw;

  // grab by torque saturation (row 7): full torque + zero progress cannot be
  // friction (worst measured breakaway incl. spread ~0.80 < u_max 0.9)
  if (stalled && std::fabs(u_raw) >= cfg.u_max) {
    sat_ticks_++;
    if (sat_ticks_ >= cfg.grab_sat_ms && !SIL_BUG(bug_, T9_NO_WATCHDOG)) {
      declare_grab();
      return 0.0f;
    }
  } else {
    sat_ticks_ = 0;
  }
  u_prev_sign_ = signf(u) != 0.0f ? signf(u) : u_prev_sign_;
  return u;
}

float Controller::tick_brake(float v_hat) {
  if (std::fabs(v_hat) < cfg_->stall_v) {  // row 9: still -> WAIT (long)
    p_at_pulse_valid_ = false;
    nomove_pulses_ = 0;
    if (SIL_BUG(bug_, T15_ROGUE_TRANSITION)) {
      // undocumented BRAKE -> PULSE shortcut (T15 negative control)
      start_pulse(cfg_->pulse_min_ms, +1, State::WAIT, 0.0f);
      return 0.0f;
    }
    enter_wait(cfg_->impulse_settle_ms);
  }
  return 0.0f;
}

float Controller::tick_pulse() {
  if (pulse_left_ > 0) {  // still emitting: exactly `width` ticks at +-0.8
    pulse_left_--;
    return cfg_->impulse_duty * static_cast<float>(pulse_dir_);
  }
  // row 16: width elapsed — back to the origin state, settle timer running
  const int settle_ticks = last_width_ms_ > 12 ? cfg_->impulse_settle_ms
                                               : cfg_->impulse_settle_short_ms;
  if (pulse_origin_ == State::CRAWL) {
    state_ = State::CRAWL;
    crawl_settle_left_ = settle_ticks;
  } else {
    enter_wait(settle_ticks);
  }
  return 0.0f;
}

float Controller::tick_wait(float p_hat) {
  wait_left_--;
  if (wait_left_ <= 0) endgame_decide(p_hat);
  return 0.0f;
}

// ---------------------------------------------------------------- endgame --

void Controller::settle(float p_hat, SettleFlag flag) {
  state_ = State::IDLE;
  hold_ = p_hat;
  has_hold_ = true;
  settle_flag_ = flag;
  watchdog_left_ = -1;
  has_target_ = false;
  if (log_.settled_tick < 0) log_.settled_tick = t_;
}

void Controller::enter_wait(int ticks) {
  state_ = State::WAIT;
  wait_left_ = ticks > 0 ? ticks : 1;
}

void Controller::restart_motion(float p_hat) {
  p_ref_ = p_hat;
  v_des_prev_ = 0.0f;
  kick_ = 0.0f;
  stall_ticks_ = 0;
  sat_ticks_ = 0;
  ph_count_ = 0;
  ph_head_ = 0;
  nomove_pulses_ = 0;
  p_at_pulse_valid_ = false;
  endgame_pulses_ = 0;

  if (v_req_ > 0.0f) {  // row 26: CRAWL toward the target itself
    dir_ = target_ >= p_hat ? 1 : -1;
    ratchet_ = p_hat;
    crawl_rate_ = v_req_;
    const float w = crawl_pick_width(crawl_rate_);
    crawl_expected_ = pulse_travel(*calib_, w, dir_);
    crawl_settle_left_ = 0;
    crawl_win_left_ = cfg_->crawl_degraded_win;
    crawl_win_p0_ = p_hat;
    state_ = State::CRAWL;
    return;
  }
  // row 3: main move aims BELOW the target in the forward sense — the
  // endgame lands with forward pulses only (§4 forbidden #4)
  waypoint_ = target_ - cfg_->approach_margin;
  dir_ = waypoint_ >= p_hat ? 1 : -1;
  last_pulse_dir_ = dir_;  // the drive itself is the last motion (T7 seat rule)
  state_ = State::MOVE;
}

void Controller::start_pulse(float width_ms, int dir, State origin, float p_hat) {
  float w = clipf(width_ms, cfg_->pulse_min_ms, cfg_->pulse_max_ms);
  if (SIL_BUG(bug_, T10_LONG_PULSE)) w = 200.0f;
  pulse_left_ = static_cast<int>(w + 0.5f);
  last_width_ms_ = pulse_left_;
  pulse_dir_ = dir;
  last_pulse_dir_ = dir;
  pulse_origin_ = origin;
  p_at_pulse_ = p_hat;
  p_at_pulse_valid_ = true;
  log_.pulses++;
  if (origin == State::WAIT) endgame_pulses_++;
  state_ = State::PULSE;
}

void Controller::endgame_decide(float p_hat) {
  const ServoConfig &cfg = *cfg_;
  const float err = target_ - p_hat;  // > 0: forward pulse needed

  // row 12: zero-travel pulses = something holds the slider (M7: a 0.8-duty
  // pulse always moves a free slider)
  if (p_at_pulse_valid_) {
    if (std::fabs(p_hat - p_at_pulse_) < cfg.pulse_move_eps) {
      nomove_pulses_++;
      if (nomove_pulses_ >= cfg.grab_pulses && !SIL_BUG(bug_, T9_NO_WATCHDOG)) {
        nomove_pulses_ = 0;
        p_at_pulse_valid_ = false;
        declare_grab();
        return;
      }
    } else {
      nomove_pulses_ = 0;
    }
  }

  if (std::fabs(err) <= cfg.tol) {  // row 10
    if (last_pulse_dir_ >= 0 || SIL_BUG(bug_, T7_REV_LANDING)) {
      settle(p_hat, SettleFlag::REACHED);
      return;
    }
    // inside tolerance but the last motion was reverse: the settle latch must
    // be backlash-canceling (T7, §4 forbidden #4) — seat forward with the
    // minimum table pulse before latching
    start_pulse(cfg.pulse_min_ms, +1, State::WAIT, p_hat);
    return;
  }
  if (endgame_pulses_ >= cfg.impulse_max || log_.removes > cfg.remove_max) {  // row 11
    if (SIL_BUG(bug_, T8_CLEAR_GAVE_UP)) {
      settle(p_hat, SettleFlag::REACHED);  // the dishonest latch (T8 negative)
      return;
    }
    log_.gave_up = true;
    settle(p_hat, SettleFlag::GAVE_UP);
    return;
  }
  if (std::fabs(err) > cfg.endgame_zone + 2.0f) {  // row 13: full re-approach
    log_.removes++;
    restart_motion(p_hat);
    return;
  }
  if (err > 0.0f) {  // row 14
    start_pulse(pulse_width_for(*calib_, err * cfg.impulse_aim, +1), +1,
                State::WAIT, p_hat);
    return;
  }
  // row 15: overshoot -> rev pulse to BELOW the target, forward re-approach
  if (last_pulse_dir_ == +1) log_.overshoots++;
  start_pulse(
      pulse_width_for(*calib_, (-err + cfg.reapproach) * cfg.impulse_aim, -1),
      -1, State::WAIT, p_hat);
}

// ------------------------------------------------------------------ CRAWL --

float Controller::crawl_pick_width(float rate) const {
  // smallest table width whose expected travel sustains the rate through its
  // own settle time, floored at the noise-safe minimum step
  for (int i = 0; i < FWD_PULSE_POINTS; i++) {
    const float w = calib_->fwd_widths[i];
    if (w > cfg_->crawl_max_width_ms) break;
    const float settle = w <= 12.0f
                             ? static_cast<float>(cfg_->impulse_settle_short_ms)
                             : static_cast<float>(cfg_->impulse_settle_ms);
    const float needed = rate * (settle + w) * 0.001f;
    const float travel = pulse_travel(*calib_, w, dir_);
    const float floor_ = needed > cfg_->crawl_min_step ? needed : cfg_->crawl_min_step;
    if (travel >= floor_) return w;
  }
  return cfg_->crawl_max_width_ms;
}

float Controller::tick_crawl(float p_hat) {
  const ServoConfig &cfg = *cfg_;
  const float d = static_cast<float>(dir_);
  const float hold_duty = SIL_BUG(bug_, T16_CRAWL_HOLD) ? 0.1f * d : 0.0f;

  if ((target_ - p_hat) * d <= cfg.endgame_zone) {  // row 28: endgame handover
    p_at_pulse_valid_ = false;
    nomove_pulses_ = 0;
    enter_wait(1);
    return 0.0f;
  }

  // ratchet reference: advances at the requested rate, lead-clamped so a
  // short-delivering plant self-paces instead of winding up a false deficit
  ratchet_ += d * crawl_rate_ * DT;
  const float lead_max =
      cfg.crawl_lead_frac * crawl_expected_ > cfg.crawl_min_step
          ? cfg.crawl_lead_frac * crawl_expected_
          : cfg.crawl_min_step;
  if ((ratchet_ - p_hat) * d > lead_max) ratchet_ = p_hat + d * lead_max;

  // row 29 (drag): the hand moving the slider AHEAD of the bounded plan is
  // excess motion the controller did not command -> release, never fight
  if ((p_hat - ratchet_) * d > cfg.crawl_override_err &&
      !SIL_BUG(bug_, T17_NO_DRAG_CHECK)) {
    declare_grab();
    return 0.0f;
  }

  // honest pacing flag (T8 family): persistent under-delivery is reported
  if (--crawl_win_left_ <= 0) {
    const float achieved = std::fabs(p_hat - crawl_win_p0_) /
                           (static_cast<float>(cfg.crawl_degraded_win) * DT);
    if (achieved < cfg.crawl_degraded_frac * crawl_rate_)
      log_.crawl_degraded = true;
    crawl_win_left_ = cfg.crawl_degraded_win;
    crawl_win_p0_ = p_hat;
  }

  if (crawl_settle_left_ > 0) {  // post-pulse settle: output stays zero
    crawl_settle_left_--;
    return hold_duty;
  }

  const float deficit = (ratchet_ - p_hat) * d;
  if (deficit >= cfg.crawl_fire_frac * crawl_expected_) {
    // row 29 (blocking): repeated pulses with zero travel = a holding hand
    if (p_at_pulse_valid_ &&
        std::fabs(p_hat - p_at_pulse_) < cfg.pulse_move_eps) {
      nomove_pulses_++;
      if (nomove_pulses_ >= cfg.crawl_grab_pulses &&
          !SIL_BUG(bug_, T9_NO_WATCHDOG)) {
        declare_grab();
        return 0.0f;
      }
    } else if (p_at_pulse_valid_) {
      nomove_pulses_ = 0;
    }
    // row 27: width dosed from the per-unit table for the actual deficit
    float w = pulse_width_for(*calib_, deficit, dir_);
    w = clipf(w, cfg.pulse_min_ms, cfg.crawl_max_width_ms);
    start_pulse(w, dir_, State::CRAWL, p_hat);
  }
  return hold_duty;
}

// ---------------------------------------------------------------- helpers --

void Controller::arm_watchdog(float distance) {
  const float rate = v_req_ > 0.0f ? v_req_ : cfg_->v_cruise;
  const float expected_s = distance / rate + cfg_->watchdog_endgame_s;
  watchdog_left_ =
      static_cast<int32_t>((2.0f * expected_s + 1.0f) * FS_HZ);  // row 19
}

}  // namespace slider_servo
