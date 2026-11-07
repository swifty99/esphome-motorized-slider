#pragma once

// Top-level tick pipeline (spec §1): sample ADC -> L2 predict(u[t−1])+update
// -> L3/L4 evaluate -> u[t] -> L1 clamp -> PWM + recorder. The recorder taps
// the pipeline read-only; control is recorder-blind (T13, §4 forbidden #5).
//
// Pure C++, zero IDF includes. The wrapper (firmware) or the SIL harness owns
// the HAL. Two entry points:
//   tick(z_pct)      legacy: z already in % travel, returned duty is applied
//                    to the PWM as-is (the wrapper does raw->% and polarity).
//   tick_raw(raw)    kueche-mief-spec: raw ADC counts in, MOTOR duty out. The
//                    core maps raw->% with UnitCalib.raw_at_0/100 and applies
//                    UnitCalib.dir_sign (R6). Required for L6 Ident.
//
// L6 Ident (kueche-mief-spec §3): start_ident() takes the motor exclusively
// (controller State::IDENT, set_target rejected with REJECTED_IDENT); L1 stays
// supreme. On I9 the core switches to the identified UnitCalib (own copy) and
// publishes it via ident_result()/ident_commit_seq() for the wrapper's NVS
// write (main task, never in the tick). FAIL/ABORT keep the previous calib
// and the previous committed result bit-identical.

#include <cstdint>
#include <vector>

#include "controller.h"
#include "ident.h"
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
  float u_cmd;    // L3/L4 output (L6 output while identing)
  float u_applied;  // after L1
  uint8_t state;
  uint8_t settle_flag;
  uint8_t release_reason;
  uint8_t clamped;
  float v_des;
  float ff;
  uint8_t ident_state;
  uint8_t unidentified;
};

class Servo {
 public:
  // recorder_capacity 0 disables the recorder entirely (T13: control trace
  // must be bit-identical either way). Allocation happens here, never in tick.
  // icfg == nullptr -> compiled-in IdentConfig defaults (rig hints).
  void init(const UnitCalib *calib, const ServoConfig *cfg, float p0,
            int recorder_capacity = 0, SeededBug bug = SeededBug::NONE,
            const IdentConfig *icfg = nullptr) {
    calib_ = calib;
    cfg_ = cfg;
    bug_ = bug;
    icfg_ = icfg != nullptr ? icfg : &icfg_default_;
    observer_.init(calib, cfg, p0, bug);
    controller_.init(calib, cfg, p0, bug);
    safety_.init(cfg, bug);
    ident_.init(icfg_, cfg, bug);
    active_stamp_ = IdentStamp{};
    u_prev_ = 0.0f;
    ring_.clear();
    ring_.resize(recorder_capacity > 0 ? recorder_capacity : 0);
    rec_head_ = 0;
    rec_count_ = 0;
  }

  // Commands (L5 wrapper / SIL scenario)
  CmdStatus set_target(float target, float v_req = 0.0f) {
    controller_.set_target(target, observer_.p_hat(), v_req);
    return controller_.last_cmd_status();
  }
  void release() {  // row 23; during L6 any stop command aborts the ident (row 26)
    if (ident_.active()) {
      ident_.abort();
      return;
    }
    controller_.release(ReleaseReason::COMMAND);
  }
  // L1 abort switch: clamps in L1 only — the controller is not informed
  // politely, it yields via the persistent-interlock path (row 18, T14); an
  // ident yields via row 28.
  void set_abort(bool on) { safety_.set_abort(on); }
  void preload_thermal(float ema) { safety_.preload_thermal(ema); }

  // ---- L6 Ident (kueche-mief-spec §3) ----
  // Row 1/2: accepted only from controller IDLE, no observer fault, and the
  // wrapper's arbiter free (R1: no other instance with torque != 0).
  // Requires the tick_raw() entry point. Returns false on rejection (row 2;
  // ident_outcome() == REJECTED) or while a run is active (no-op).
  bool start_ident(uint32_t date_counter = 0, bool arbiter_free = true) {
    const bool ok = controller_.state() == State::IDLE && !observer_.fault() &&
                    arbiter_free;
    if (!ident_.start(ok, *calib_, active_stamp_, date_counter)) return false;
    controller_.enter_ident();
    return true;
  }
  void abort_ident() { ident_.abort(); }  // row 26: duty 0 on the next tick
  bool ident_active() const { return ident_.active(); }
  IdentState ident_state() const { return ident_.state(); }
  IdentOutcome ident_outcome() const { return ident_.outcome(); }
  IdentReason ident_fail_reason() const { return ident_.reason(); }
  float ident_phase_progress() const { return ident_.phase_progress(); }
  // last COMMITTED result (valid only after an I9 or a successful adopt)
  const IdentResult &ident_result() const { return ident_.committed(); }
  uint32_t ident_commit_seq() const { return ident_.commit_seq(); }
  const Ident &ident() const { return ident_; }
  // Boot path: adopt a stored result iff its stamp is valid (§5) and no run is
  // active. raw_now seeds the observer at the current position.
  bool adopt_ident(const IdentResult &r, float raw_now) {
    if (ident_.active() || !ident_stamp_valid(r, icfg_->fw_major, icfg_->instance))
      return false;
    own_calib_ = r.calib;
    calib_ = &own_calib_;
    active_stamp_ = r.stamp;
    ident_.set_committed(r);
    const float p = raw_to_pct(raw_now);
    observer_.init(calib_, cfg_, p, bug_);
    controller_.init(calib_, cfg_, p, bug_);
    u_prev_ = 0.0f;
    return true;
  }
  const UnitCalib &calib() const { return *calib_; }
  float raw_to_pct(float raw) const {
    return (raw - calib_->raw_at_0) / (calib_->raw_at_100 - calib_->raw_at_0) * 100.0f;
  }

  // One 1 kHz tick on RAW counts (kueche-mief-spec). supply_v < 0 = no supply
  // ADC; adc_err = a oneshot read failed this tick. Returns MOTOR duty.
  float tick_raw(float raw, float supply_v = -1.0f, bool adc_err = false) {
    if (ident_.active()) {
      IdentInputs in;
      in.raw = raw;
      in.supply_v = supply_v;
      in.adc_err = adc_err;
      in.thermal_ema = safety_.thermal_avg();
      in.thermal_tripped = safety_.tripped();
      in.l1_cut = safety_.will_cut();
      const float u_id = ident_.tick(in);
      const float u_applied = safety_.clamp(u_id);  // L1 supreme (T14 / row 28)
      u_prev_ = 0.0f;
      u_cmd_last_ = u_id;
      record(ident_.pos_pct(), u_id, u_applied);
      if (ident_.just_finished()) finish_ident(raw);
      return u_applied;
    }
    const float u = tick(raw_to_pct(raw));
    return calib_->dir_sign < 0 ? -u : u;
  }

  // One 1 kHz tick. z_pct: position sample in % travel. Returns applied duty.
  float tick(float z_pct) {
    if (ident_.active()) {  // L6 needs raw counts: the legacy path aborts it
      ident_.abort();
      ident_.tick(IdentInputs{});
      ident_.tick(IdentInputs{});
      finish_ident_pct(z_pct);
    }
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
    record(z_pct, u_cmd, u_applied);
    return u_applied;
  }

  const Observer &observer() const { return observer_; }
  const Controller &controller() const { return controller_; }
  const Safety &safety() const { return safety_; }
  float u_prev() const { return u_prev_; }
  float u_cmd_last() const { return u_cmd_last_; }

 private:
  void finish_ident(float raw) {
    if (ident_.outcome() == IdentOutcome::OK) {
      own_calib_ = ident_.committed().calib;  // run on the IDENTIFIED calib
      calib_ = &own_calib_;
      active_stamp_ = ident_.committed().stamp;
      const float p = raw_to_pct(raw);
      observer_.init(calib_, cfg_, p, bug_);
      controller_.init(calib_, cfg_, p, bug_);
    } else {
      finish_ident_pct(raw_to_pct(raw));
    }
    u_prev_ = 0.0f;
  }
  void finish_ident_pct(float p) {  // row 31: previous calib, hold := p, torque 0
    observer_.init(calib_, cfg_, p, bug_);
    controller_.exit_ident(p);
    u_prev_ = 0.0f;
  }

  void record(float z_pct, float u_cmd, float u_applied) {
    if (ring_.empty()) return;
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
                   controller_.last_v_des(), controller_.last_ff(),
                   static_cast<uint8_t>(ident_.state()),
                   static_cast<uint8_t>(controller_.unidentified_flag() ? 1 : 0)};
    rec_head_ = (rec_head_ + 1) % static_cast<int>(ring_.size());
    if (!full) rec_count_++;
  }

  const UnitCalib *calib_{nullptr};
  const ServoConfig *cfg_{nullptr};
  const IdentConfig *icfg_{nullptr};
  IdentConfig icfg_default_;
  SeededBug bug_{SeededBug::NONE};
  Observer observer_;
  Controller controller_;
  Safety safety_;
  Ident ident_;
  UnitCalib own_calib_;     // the identified calib once adopted / committed
  IdentStamp active_stamp_;
  float u_prev_{0.0f};
  float u_cmd_last_{0.0f};
  std::vector<TickRecord> ring_;
  int rec_head_{0};
  int rec_count_{0};
};

}  // namespace slider_servo
