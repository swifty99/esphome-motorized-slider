#pragma once

// Top-level tick pipeline (spec §1): sample ADC -> L2 predict(u[t-1])+update
// -> L3/L4 evaluate -> u[t] -> L1 clamp -> PWM + recorder. The recorder taps
// the pipeline read-only; control is recorder-blind (T13, §4 forbidden #5).
//
// Pure C++, zero IDF includes. The wrapper (firmware) or the SIL harness owns
// the HAL: it feeds z in % travel and applies the returned duty to the PWM.

#include <cstdint>
#include <vector>

#include "controller.h"
#include "observer.h"
#include "safety.h"
#include "servo_params.h"
#include "sil_bugs.h"

namespace slider_servo {

// One tick of trace, written by the recorder tap after L1.
struct TickRecord {
  float z;        // measurement [%]
  float p_hat;
  float v_hat;
  float u_cmd;    // L3/L4 output
  float u_applied;  // after L1
  uint8_t state;
  uint8_t settle_flag;
  uint8_t release_reason;
  uint8_t clamped;
  float v_des;
  float ff;
};

class Servo {
 public:
  // recorder_capacity 0 disables the recorder entirely (T13: control trace
  // must be bit-identical either way). Allocation happens here, never in tick.
  void init(const UnitCalib *calib, const ServoConfig *cfg, float p0,
            int recorder_capacity = 0, SeededBug bug = SeededBug::NONE) {
    calib_ = calib;
    cfg_ = cfg;
    bug_ = bug;
    observer_.init(calib, cfg, p0, bug);
    controller_.init(calib, cfg, p0, bug);
    safety_.init(cfg, bug);
    u_prev_ = 0.0f;
    ring_.clear();
    ring_.resize(recorder_capacity > 0 ? recorder_capacity : 0);
    rec_head_ = 0;
    rec_count_ = 0;
  }

  // Commands (L5 wrapper / SIL scenario)
  void set_target(float target, float v_req = 0.0f) {
    controller_.set_target(target, observer_.p_hat(), v_req);
  }
  void release() { controller_.release(ReleaseReason::COMMAND); }
  void set_abort(bool on) {
    safety_.set_abort(on);
    if (on) controller_.release(ReleaseReason::COMMAND);  // row 23
  }

  // One 1 kHz tick. z_pct: position sample in % travel. Returns applied duty.
  float tick(float z_pct) {
    if (SIL_BUG(bug_, T12_DUTY_IN_Z))
      z_pct += 20.0f * u_prev_;  // the forbidden duty echo (T12 negative)

    observer_.step(u_prev_, z_pct);
    if (observer_.fault())  // row 25
      controller_.release(ReleaseReason::FAULT);

    const float u_cmd = controller_.tick(observer_.p_hat(), observer_.v_hat());
    const float u_applied = safety_.clamp(u_cmd);
    controller_.notify_interlock(safety_.clamped());
    u_prev_ = u_applied;
    u_cmd_last_ = u_cmd;

    if (!ring_.empty()) {
      const bool full = rec_count_ >= static_cast<int>(ring_.size());
      if (full && SIL_BUG(bug_, T13_RECORDER_STALL)) {
        // seeded backpressure: a full recorder must NEVER touch control —
        // this bug makes it freeze the command (T13 negative control)
        u_prev_ = 0.0f;
      }
      TickRecord &r = ring_[rec_head_];
      r = TickRecord{z_pct, observer_.p_hat(), observer_.v_hat(), u_cmd,
                     u_applied, static_cast<uint8_t>(controller_.state()),
                     static_cast<uint8_t>(controller_.settle_flag()),
                     static_cast<uint8_t>(controller_.release_reason()),
                     static_cast<uint8_t>(safety_.clamped() ? 1 : 0),
                     controller_.last_v_des(), controller_.last_ff()};
      rec_head_ = (rec_head_ + 1) % static_cast<int>(ring_.size());
      if (!full) rec_count_++;
    }
    return u_applied;
  }

  const Observer &observer() const { return observer_; }
  const Controller &controller() const { return controller_; }
  const Safety &safety() const { return safety_; }
  float u_prev() const { return u_prev_; }
  float u_cmd_last() const { return u_cmd_last_; }

 private:
  const UnitCalib *calib_{nullptr};
  const ServoConfig *cfg_{nullptr};
  SeededBug bug_{SeededBug::NONE};
  Observer observer_;
  Controller controller_;
  Safety safety_;
  float u_prev_{0.0f};
  float u_cmd_last_{0.0f};
  std::vector<TickRecord> ring_;
  int rec_head_{0};
  int rec_count_{0};
};

}  // namespace slider_servo
