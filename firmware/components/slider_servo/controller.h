#pragma once

// L3 Motion core + L4 Trajectory: the spec §2 trigger graph (rows 1–29),
// cascade law §1, endgame from the per-unit impulse table, CRAWL pulse-train
// pacing (Gate answer 1), progress-based stall escalation, grab declaration.
//
// Pure C++, zero IDF includes, no function-static state, no heap in tick()
// (the progress window is a fixed ring). Runs on OBSERVER state only —
// p_hat/v_hat in, duty command out; L1 clamps downstream.

#include <cstdint>

#include "servo_params.h"
#include "sil_bugs.h"

namespace slider_servo {

enum class State : uint8_t {
  IDLE = 0,
  MOVE = 1,
  BRAKE = 2,
  PULSE = 3,
  WAIT = 4,
  CRAWL = 5,
  RELEASED = 6,
};

enum class ReleaseReason : uint8_t {
  NONE = 0, GRAB = 1, INTERLOCK = 2, WATCHDOG = 3, FAULT = 4, COMMAND = 5,
};

// Honest terminal flag (T8): every settle says what actually happened.
enum class SettleFlag : uint8_t { NONE = 0, REACHED = 1, GAVE_UP = 2, ADOPTED = 3 };

struct ControllerLog {
  int kicks = 0;
  int pulses = 0;
  int overshoots = 0;
  int removes = 0;
  bool gave_up = false;
  bool crawl_degraded = false;
  bool target_clamped = false;
  bool watchdog_tripped = false;
  int32_t grab_detect_tick = -1;
  int32_t settled_tick = -1;
};

inline constexpr int PROGRESS_WIN_MAX = 256;  // ring capacity >= cfg.progress_win

class Controller {
 public:
  void init(const UnitCalib *calib, const ServoConfig *cfg, float p0,
            SeededBug bug = SeededBug::NONE);

  // CMD set_target (rows 1–3, 8, 17, 22, 26). v_req <= 0 = default fast move;
  // 0 < v_req < v_min_cmd requests CRAWL; v_req in (v_crawl_max, v_min_cmd)
  // rounds up to a fast move (spec §5).
  void set_target(float target, float p_hat, float v_req = 0.0f);
  // CMD release/abort (row 23) and forced releases (rows 18, 19, 25).
  void release(ReleaseReason reason);
  // L1 reports its clamp each tick (row 18: persistent -> RELEASED).
  void notify_interlock(bool clamped);

  // One 1 kHz evaluation. Returns the commanded duty (pre-L1).
  float tick(float p_hat, float v_hat);

  State state() const { return state_; }
  ReleaseReason release_reason() const { return release_reason_; }
  SettleFlag settle_flag() const { return settle_flag_; }
  const ControllerLog &log() const { return log_; }
  bool settled() const { return state_ == State::IDLE && has_hold_; }
  float hold() const { return hold_; }
  float target() const { return target_; }
  // exposed for the SIL trace (T2, T3 checks)
  float last_v_des() const { return last_v_des_; }
  float last_ff() const { return last_ff_; }

 private:
  float tick_idle(float p_hat);
  float tick_released(float p_hat, float v_hat);
  float tick_move(float p_hat, float v_hat);
  float tick_brake(float v_hat);
  float tick_pulse();
  float tick_wait(float p_hat);
  float tick_crawl(float p_hat);

  void settle(float p_hat, SettleFlag flag);
  void enter_wait(int ticks);
  void restart_motion(float p_hat);   // MOVE or CRAWL per v_req_ (rows 3, 13, 26)
  void endgame_decide(float p_hat);
  void start_pulse(float width_ms, int dir, State origin, float p_hat);
  float crawl_pick_width(float rate) const;
  void arm_watchdog(float distance);
  bool progress_update(float p_hat);  // returns stalled (window progress < stall_v)
  void declare_grab();

  const UnitCalib *calib_{nullptr};
  const ServoConfig *cfg_{nullptr};
  SeededBug bug_{SeededBug::NONE};

  State state_{State::IDLE};
  ReleaseReason release_reason_{ReleaseReason::NONE};
  SettleFlag settle_flag_{SettleFlag::NONE};
  ControllerLog log_;

  int32_t t_{0};
  float target_{50.0f};
  float v_req_{0.0f};            // 0 = fast; else requested crawl rate
  float hold_{50.0f};
  bool has_hold_{false};
  bool has_target_{false};

  // pending command while RELEASED (row 22) or braking retarget (row 8)
  bool pending_{false};
  float pending_target_{0.0f};
  float pending_v_req_{0.0f};

  // MOVE
  float waypoint_{0.0f};
  int dir_{1};
  float p_ref_{0.0f};
  float v_des_prev_{0.0f};
  float last_v_des_{0.0f};
  float last_ff_{0.0f};
  float u_prev_sign_{1.0f};      // only read by seeded bug T3

  // progress window (fixed ring — no heap in tick)
  float ph_ring_[PROGRESS_WIN_MAX];
  int ph_count_{0};
  int ph_head_{0};

  // stall / escalation / grab-by-saturation
  int stall_ticks_{0};
  float kick_{0.0f};
  int kick_cool_{0};
  int sat_ticks_{0};

  // PULSE / WAIT / endgame
  int pulse_left_{0};
  int pulse_dir_{1};
  int last_width_ms_{0};
  int last_pulse_dir_{0};
  int endgame_pulses_{0};   // row-11 budget: endgame only, CRAWL pulses excluded
  State pulse_origin_{State::WAIT};
  int wait_left_{0};
  float p_at_pulse_{-1.0f};
  bool p_at_pulse_valid_{false};
  int nomove_pulses_{0};

  // CRAWL
  float ratchet_{0.0f};
  float crawl_rate_{0.0f};
  float crawl_expected_{0.0f};   // travel of the pacing width
  int crawl_settle_left_{0};
  int crawl_win_left_{0};
  float crawl_win_p0_{0.0f};
  float crawl_w_scale_{1.0f};        // adaptive width multiplier (self-adjust)
  bool crawl_adapt_pending_{false};  // a CRAWL pulse just completed; adapt scale before next fire

  // RELEASED
  int still_ticks_{0};
  int adopt_ticks_{0};

  // interlock persistence (row 18) and watchdog (row 19)
  int ilk_ticks_{0};
  int32_t watchdog_left_{-1};
};

}  // namespace slider_servo
