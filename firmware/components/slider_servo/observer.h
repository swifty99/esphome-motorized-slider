#pragma once

// L2 Observer: 2-state duty-driven EKF (position %, velocity %/s).
//
// Port of the Phase-3 prototype (analysis/src/slideranalysis/observer.py,
// validated 3–5x better than the legacy velocity path on real recordings).
// Prediction runs the identified plant model on the APPLIED duty; the raw
// ADC position sample (already in %) is the only measurement — commanded
// duty is model input, never z (spec §4 #6, T12). Innovation gate with a
// bounded consecutive-reject escape (T5).

#include <cmath>

#include "plant_model.h"
#include "servo_params.h"
#include "sil_bugs.h"

namespace slider_servo {

class Observer {
 public:
  void init(const UnitCalib *calib, const ServoConfig *cfg, float p0,
            SeededBug bug = SeededBug::NONE) {
    calib_ = calib;
    cfg_ = cfg;
    bug_ = bug;
    p_ = p0;
    v_ = 0.0f;
    P00_ = 1.0f;
    P01_ = 0.0f;
    P11_ = 100.0f;
    stuck_ = true;
    gate_rejects_ = 0;
    fault_ = false;
    // process noise from the fit residual, whitened over the error
    // correlation time (observer.py from_plant) — sigma_a in %/s^2
    const float sigma_a = cfg->obs_model_err_duty / calib->m *
                          std::sqrt(DT / cfg->obs_err_corr_tau_s);
    const float s2 = sigma_a * sigma_a;
    q00_ = s2 * DT * DT * DT * DT / 4.0f;
    q01_ = s2 * DT * DT * DT / 2.0f;
    q11_ = s2 * DT * DT;
    r_quiet_ = calib->sigma0_counts / calib->counts_per_percent;
    r_pwm_ = calib->sigma_pwm_counts / calib->counts_per_percent;
  }

  // One tick: predict on u[t-1], correct with z [%]. Sets fault() on
  // NaN/divergence (spec §2 row 25).
  void step(float u, float z_pct) {
    predict(u);
    update(z_pct, u);
    if (std::isnan(p_) || std::isnan(v_) || P00_ > 100.0f) fault_ = true;
  }

  float p_hat() const { return p_; }
  float v_hat() const { return v_; }
  float innovation() const { return innov_; }
  bool fault() const { return fault_; }
  bool model_stuck() const { return stuck_; }
  int gate_rejects() const { return gate_rejects_; }
  float p00() const { return P00_; }

 private:
  void predict(float u) {
    const UnitCalib &ucal = *calib_;
    const float dir = u > 0 ? 1.0f : (u < 0 ? -1.0f : 1.0f);
    stuck_ = std::fabs(v_) < cfg_->obs_v_stick &&
             std::fabs(u) <= fs_map(ucal, p_, dir);
    float a = 0.0f;
    float f00 = 1.0f, f01 = 0.0f, f11 = 0.0f;  // stuck: F = [[1,0],[0,0]]
    if (stuck_) {
      v_ = 0.0f;
    } else {
      const float sdir = v_ != 0.0f ? (v_ > 0 ? 1.0f : -1.0f)
                                    : (u > 0 ? 1.0f : (u < 0 ? -1.0f : 0.0f));
      float fk = fk_map(ucal, p_, sdir);
      if (std::fabs(v_) < cfg_->obs_v_stick) {
        // unstick cap: kinetic friction cannot exceed the drive that just
        // broke static contact (same rule as the validated sim)
        const float cap = std::fabs(u) * 0.95f;
        if (fk > cap) fk = cap;
      }
      a = (u - sdir * fk - ucal.b * v_) / ucal.m;
      f01 = DT;
      f11 = 1.0f - (ucal.b / ucal.m) * DT;
    }
    float p_new = p_ + v_ * DT;
    float v_new = v_ + a * DT;
    if (!stuck_ && v_ != 0.0f && (v_new > 0.0f) != (v_ > 0.0f)) {
      // decelerated through zero: re-stick unless the drive re-breaks static
      // contact — same rule as the identified plant. Without this the
      // velocity state rings ±(friction decel) around zero at rest forever
      // (the prototype observer had this flaw; found by SIL tripwire T5).
      const float rdir = u != 0.0f ? (u > 0 ? 1.0f : -1.0f)
                                   : (v_ > 0 ? 1.0f : -1.0f);
      if (std::fabs(u) <= fs_map(ucal, p_, rdir)) v_new = 0.0f;
    }
    if (p_new <= 0.0f) {          // hard stops are part of the plant
      p_new = 0.0f;
      if (v_new < 0.0f) v_new = 0.0f;
    } else if (p_new >= 100.0f) {
      p_new = 100.0f;
      if (v_new > 0.0f) v_new = 0.0f;
    }
    p_ = p_new;
    v_ = v_new;
    // P = F P F' + Q for F = [[f00, f01], [0, f11]]
    const float a00 = f00 * P00_ + f01 * P01_;
    const float a01 = f00 * P01_ + f01 * P11_;
    const float a11 = f11 * P11_;
    P00_ = a00 * f00 + a01 * f01 + q00_;
    P01_ = a01 * f11 + q01_;
    P11_ = f11 * a11 + q11_;
  }

  void update(float z, float u) {
    const float r = std::fabs(u) > 0.0f ? r_pwm_ : r_quiet_;
    float S = P00_ + r * r;
    innov_ = z - p_;
    if (stuck_ && innov_ * innov_ > cfg_->obs_gate_sigma * cfg_->obs_gate_sigma * S) {
      // isolated spike rejection at rest; consecutive rejections mean the
      // offset is real (hand move while model-frozen) -> accept (T5)
      gate_rejects_++;
      if (SIL_BUG(bug_, T5_NO_GATE_ESCAPE)) return;  // lockout forever (negative)
      if (gate_rejects_ <= cfg_->obs_gate_max_consecutive) return;
      // escape: the model-frozen assumption failed — covariance reset so the
      // filter SNAPS to the real offset (T5: converged <= 200 ms) instead of
      // crawling there through the tiny rest-state gain. gate_rejects_ stays
      // saturated until the innovation is back inside the gate.
      P00_ += innov_ * innov_;
      S = P00_ + r * r;
    } else {
      gate_rejects_ = 0;
    }
    const float k0 = P00_ / S;
    const float k1 = P01_ / S;
    p_ += k0 * innov_;
    v_ += k1 * innov_;
    // P = (I - K H) P, H = [1 0]
    const float p00 = P00_, p01 = P01_;
    P00_ = (1.0f - k0) * p00;
    P01_ = (1.0f - k0) * p01;
    P11_ = P11_ - k1 * p01;
  }

  const UnitCalib *calib_{nullptr};
  const ServoConfig *cfg_{nullptr};
  SeededBug bug_{SeededBug::NONE};
  float p_{50.0f}, v_{0.0f};
  float P00_{1.0f}, P01_{0.0f}, P11_{100.0f};
  float q00_{0}, q01_{0}, q11_{0};
  float r_quiet_{0.1f}, r_pwm_{0.15f};
  float innov_{0.0f};
  bool stuck_{true};
  int gate_rejects_{0};
  bool fault_{false};
};

}  // namespace slider_servo
