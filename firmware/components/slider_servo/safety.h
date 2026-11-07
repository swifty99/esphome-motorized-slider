#pragma once

// L1 Safety: thermal duty budget (EMA) + abort switch. Gate answer 8:
// NO positional torque clamp — budget + thermal are the only current
// protection (hardware tolerates 100 % duty for 10 s; L9110 ~800 mA cont.).
// L1 clamps L3's output the same tick and reports the clamp (T14).

#include <cmath>

#include "servo_params.h"
#include "sil_bugs.h"

namespace slider_servo {

class Safety {
 public:
  void init(const ServoConfig *cfg, SeededBug bug = SeededBug::NONE) {
    cfg_ = cfg;
    bug_ = bug;
    ema_ = 0.0f;
    tripped_ = false;
    abort_ = false;
    alpha_ = DT / cfg->thermal_tau_s;
    prev_clamped_ = false;
  }

  void set_abort(bool on) { abort_ = on; }
  // Restore a (conservative) thermal state, e.g. after an unexpected reset the
  // wrapper may assume the motor is still warm. Never lowers the budget's
  // protection; also used by the SIL for the Ident row-4 'thermal' FAIL.
  void preload_thermal(float ema) { ema_ = ema > ema_ ? ema : ema_; }
  bool clamped() const { return clamped_; }
  float thermal_avg() const { return ema_; }
  bool tripped() const { return tripped_; }
  // Would clamp() cut THIS tick (same hysteresis update, no side effects)?
  // L6 Ident reads it before computing its duty (row 28, same-tick supremacy).
  bool will_cut() const {
    bool t = tripped_;
    if (t && ema_ < cfg_->thermal_resume) t = false;
    if (!t && ema_ > cfg_->thermal_avg_max) t = true;
    return abort_ || t;
  }

  // Same-tick clamp (T14). Returns the applied duty.
  float clamp(float u_cmd) {
    // thermal EMA over |applied duty| with trip/resume hysteresis
    if (tripped_ && ema_ < cfg_->thermal_resume) tripped_ = false;
    if (!tripped_ && ema_ > cfg_->thermal_avg_max) tripped_ = true;

    bool cut = abort_ || tripped_;
    if (SIL_BUG(bug_, T14_CLAMP_LATE)) {  // apply LAST tick's decision (negative)
      const bool cut_now = cut;
      cut = prev_clamped_;
      prev_clamped_ = cut_now;
    } else {
      prev_clamped_ = cut;
    }
    clamped_ = cut;
    const float u = cut ? 0.0f : u_cmd;
    ema_ += alpha_ * (std::fabs(u) - ema_);
    return u;
  }

 private:
  const ServoConfig *cfg_{nullptr};
  SeededBug bug_{SeededBug::NONE};
  float ema_{0.0f};
  float alpha_{0.0001f};
  bool tripped_{false};
  bool abort_{false};
  bool clamped_{false};
  bool prev_clamped_{false};
};

}  // namespace slider_servo
