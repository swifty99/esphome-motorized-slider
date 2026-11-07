#pragma once

// SIL plant + sensor simulation: C++ port of the validated Python models
// (analysis/src/slideranalysis/plantsim.py PlantStepper, silharness.py
// SensorSim). Physics semantics are IDENTICAL — stick-slip with re-stick,
// unstick kinetic cap, brake damping always active, hard stops. The truth
// plant here is PERTURBED (friction/m/b factors from the scenario); the
// servo core always runs on the nominal UnitCalib. A controller that only
// works on the nominal plant is decoration (silharness docstring).

#include <cmath>
#include <cstdint>
#include <random>

#include "../firmware/components/slider_servo/plant_model.h"
#include "../firmware/components/slider_servo/servo_params.h"

namespace slider_sil {

using slider_servo::UnitCalib;

// Perturbation factors applied to the nominal calib to form the truth plant.
struct PlantPerturb {
  float d_fwd = 0.0f;   // friction scale fwd: x(1 + d_fwd), fs AND fk
  float d_rev = 0.0f;
  float d_m = 0.0f;     // inertia x(1 + d_m)
  float d_b = 0.0f;     // damping x(1 + d_b)
};

struct TruthPlant {
  UnitCalib u;   // perturbed copy
  float v_stick = 3.0f;  // %/s (plantsim.py default)

  static TruthPlant from(const UnitCalib &nominal, const PlantPerturb &pp) {
    TruthPlant t;
    t.u = nominal;
    for (int i = 0; i < slider_servo::KINETIC_NODES; i++) {
      t.u.fk_fwd[i] *= (1.0f + pp.d_fwd);
      t.u.fk_rev[i] *= (1.0f + pp.d_rev);
    }
    for (int i = 0; i < slider_servo::STATIC_NODES; i++) {
      t.u.fs_fwd[i] *= (1.0f + pp.d_fwd);
      t.u.fs_rev[i] *= (1.0f + pp.d_rev);
    }
    t.u.m *= (1.0f + pp.d_m);
    t.u.b *= (1.0f + pp.d_b);
    return t;
  }
};

// kueche-mief-spec §4 "SIL-Plant-Schar": the unit variants the ident must
// survive. Defaults = the legacy plant exactly (every factor multiplies by 1
// or adds 0, so the legacy trace stays byte-identical).
struct PlantVariant {
  float friction_scale = 1.0f;  // fs AND fk x scale (breakaway family 0.4–1.25)
  int motor_pol = 1;            // +1 / -1 swapped wiring / 0 not connected
  bool pot_inv = true;          // true = rig: raw HIGH at plant x = 0
  float raw_lo = 100.0f;        // wiper counts at the two rails
  float raw_hi = 8000.0f;
  float noise_scale = 1.0f;     // sigma x1..x3
  float backlash = 0.0f;        // % play between carriage and wiper (<= 2)
  float supply_scale = 1.0f;    // VM / VM_nom: effective drive x (±15 %)
  float fs_spread = 0.0f;       // relative sigma of the breakaway per stick
                                // event (rig trial spread ±0.04 ~ 2 sigma)
  float clip_hi = 8191.0f;      // ADC clamps: a wiper beyond reads exactly the
  float clip_lo = 0.0f;         // clamp (sigma -> 0), Küche Slider 2 2026-11-07
};

// friction-scaled copy of a truth plant (scenario variants / runtime events)
inline void scale_friction(TruthPlant &t, const TruthPlant &base, float k) {
  for (int i = 0; i < slider_servo::KINETIC_NODES; i++) {
    t.u.fk_fwd[i] = base.u.fk_fwd[i] * k;
    t.u.fk_rev[i] = base.u.fk_rev[i] * k;
  }
  for (int i = 0; i < slider_servo::STATIC_NODES; i++) {
    t.u.fs_fwd[i] = base.u.fs_fwd[i] * k;
    t.u.fs_rev[i] = base.u.fs_rev[i] * k;
  }
}

class PlantStepper {
 public:
  void init(const TruthPlant *tp, float p0, float dt = 0.001f) {
    tp_ = tp;
    p_ = p0;
    v_ = 0.0f;
    dt_ = dt;
    stuck_ = true;
  }
  // variant hooks (defaults keep the legacy physics bit-for-bit)
  void set_drive_gain(float g) { gain_ = g; }  // motor_pol * supply_scale
  void set_fs_spread(float rel, uint32_t seed) {
    spread_ = rel;
    rng_.seed(seed);
    redraw();
  }

  // grab / disturbance injection: infinitely stiff hand (stated limitation)
  void force(float p, float v = 0.0f, bool stuck = true) {
    p_ = p;
    v_ = v;
    stuck_ = stuck;
  }

  // One 1 kHz tick with applied duty. No end-zone cut here: Gate answer 8 —
  // the servo has no positional clamp; the rail itself is the hard stop.
  void step(float u) {
    u *= gain_;
    const UnitCalib &pu = tp_->u;
    const bool was_stuck = stuck_;
    const float m = pu.m, b = pu.b;
    float p = p_, v = v_;
    bool stuck = stuck_;

    if (stuck) {
      const float sdir = u > 0 ? 1.0f : (u < 0 ? -1.0f : 0.0f);
      if (sdir != 0.0f && std::fabs(u) > fs_eff(pu, p, sdir)) {
        stuck = false;
        v = 0.0f;
      }
    }
    if (!stuck) {
      float sdir = v != 0.0f ? (v > 0 ? 1.0f : -1.0f)
                             : (u > 0 ? 1.0f : (u < 0 ? -1.0f : 0.0f));
      if (sdir == 0.0f) {
        stuck = true;
      } else {
        float fk_tot = slider_servo::fk_map(pu, p, sdir);
        if (v == 0.0f) {
          // unstick tick: kinetic friction cannot exceed the drive that just
          // broke static contact (measured "poor sustain" reproduction)
          const float cap = std::fabs(u) * 0.95f;
          if (fk_tot > cap) fk_tot = cap;
        }
        const float acc = (u - sdir * fk_tot - b * v) / m;
        float v_new = v + acc * dt_;
        if (v != 0.0f && ((v_new > 0) != (v > 0) || v_new == 0.0f)) {
          // decelerated through zero: re-stick unless u re-breaks
          const float rdir = u != 0.0f ? (u > 0 ? 1.0f : -1.0f) : sdir;
          if (std::fabs(u) <= fs_eff(pu, p, rdir)) {
            v_new = 0.0f;
            stuck = true;
          }
        } else if (v == 0.0f && u != 0.0f && (v_new > 0) != (u > 0)) {
          v_new = 0.0f;  // friction-map inconsistency guard
          stuck = true;
        }
        v = v_new;
        if (std::fabs(v) < tp_->v_stick &&
            std::fabs(u) <= fs_eff(pu, p, sdir)) {
          v = 0.0f;
          stuck = true;
        }
      }
    }
    p += v * dt_;
    if (p <= 0.0f) {
      p = 0.0f; v = 0.0f; stuck = true;
    } else if (p >= 100.0f) {
      p = 100.0f; v = 0.0f; stuck = true;
    }
    p_ = p; v_ = v; stuck_ = stuck;
    if (stuck && !was_stuck && spread_ > 0.0f) redraw();  // new contact, new F_s
  }

  float p() const { return p_; }
  float v() const { return v_; }
  bool stuck() const { return stuck_; }

 private:
  float fs_eff(const UnitCalib &pu, float p, float dir) const {
    return slider_servo::fs_map(pu, p, dir) * mult_;
  }
  void redraw() {
    if (spread_ <= 0.0f) {
      mult_ = 1.0f;
      return;
    }
    float e = gauss_(rng_);
    if (e > 2.0f) e = 2.0f;
    if (e < -2.0f) e = -2.0f;
    mult_ = 1.0f + spread_ * e;
  }

  const TruthPlant *tp_{nullptr};
  float p_{50.0f}, v_{0.0f}, dt_{0.001f};
  bool stuck_{true};
  float gain_{1.0f};
  float spread_{0.0f};
  float mult_{1.0f};
  std::mt19937 rng_;
  std::normal_distribution<float> gauss_{0.0f, 1.0f};
};

// ADC model (silharness SensorSim): gaussian floor (duty-dependent) + PWM
// spike tail + 0.25-count quantization, returned in % travel. The raw scale
// is inverted (8000 counts at 0 %, 100 at 100 %) like the real wiper path.
class SensorSim {
 public:
  static constexpr float RAW_AT_0PCT = 8000.0f;
  static constexpr float RAW_AT_100PCT = 100.0f;
  static constexpr float SPIKE_PROB_PWM = 0.004f;

  void init(const UnitCalib *nominal, uint32_t seed, bool freeze = false) {
    nom_ = nominal;
    rng_.seed(seed);
    scale_ = (RAW_AT_100PCT - RAW_AT_0PCT) / 100.0f;
    frozen_ = false;
    freeze_val_ = 0.0f;
    (void)freeze;
  }

  void set_frozen(bool on, float at_pct) {
    frozen_ = on;
    freeze_val_ = at_pct;
  }

  float sample(float pos, float duty) {
    if (frozen_) return freeze_val_;
    const float sigma =
        std::fabs(duty) > 0 ? nom_->sigma_pwm_counts : nom_->sigma0_counts;
    float raw = RAW_AT_0PCT + pos * scale_ + gauss_(rng_) * sigma;
    if (std::fabs(duty) > 0 && uni_(rng_) < SPIKE_PROB_PWM) {
      const float mag = 60.0f + uni_(rng_) * 180.0f;
      raw += (uni_(rng_) < 0.5f ? -mag : mag);
    }
    raw = std::round(raw * 4.0f) / 4.0f;
    return (raw - RAW_AT_0PCT) / scale_;
  }

 private:
  const UnitCalib *nom_{nullptr};
  std::mt19937 rng_;
  std::normal_distribution<float> gauss_{0.0f, 1.0f};
  std::uniform_real_distribution<float> uni_{0.0f, 1.0f};
  float scale_{-79.0f};
  bool frozen_{false};
  float freeze_val_{0.0f};
};

// Raw-count wiper model for the ident family (the legacy SensorSim above is
// untouched): same gaussian floor + PWM spike tail + 0.25-count quantization,
// in COUNTS, with orientation, span, noise scale, carriage/wiper backlash and
// fault injection. The ident sees nothing but this number.
class RawSensor {
 public:
  enum class Fault : uint8_t { NONE, FLAT, RAIL, STUCK };
  static constexpr float FULL_SCALE = 8191.0f;

  void init(const UnitCalib *nominal, uint32_t seed, const PlantVariant &v, float x0) {
    nom_ = nominal;
    v_ = v;
    rng_.seed(seed);
    xs_ = x0;
  }
  void set_fault(Fault f, float stuck_val = 0.0f) {
    fault_ = f;
    stuck_val_ = stuck_val;
  }
  void set_frozen(bool on) { frozen_ = on; }
  float raw_at_x(float x) const {
    const float r0 = v_.pot_inv ? v_.raw_hi : v_.raw_lo;
    const float r100 = v_.pot_inv ? v_.raw_lo : v_.raw_hi;
    return r0 + x * (r100 - r0) / 100.0f;
  }
  float wiper_x() const { return xs_; }
  // noise-free reading at x incl. the ADC clamps (truth for the calibrated %)
  float clean_clipped(float x) const {
    float r = raw_at_x(x);
    if (r < v_.clip_lo) r = v_.clip_lo;
    if (r > v_.clip_hi) r = v_.clip_hi;
    return r;
  }

  float sample_raw(float x, float duty) {
    // backlash: the wiper follows the carriage only beyond half the play
    const float h = 0.5f * v_.backlash;
    if (x - xs_ > h) xs_ = x - h;
    else if (xs_ - x > h) xs_ = x + h;
    float raw = raw_at_x(xs_);
    const float sigma = (std::fabs(duty) > 0 ? nom_->sigma_pwm_counts
                                             : nom_->sigma0_counts) * v_.noise_scale;
    raw += gauss_(rng_) * sigma;
    if (std::fabs(duty) > 0 && uni_(rng_) < SensorSim::SPIKE_PROB_PWM) {
      const float mag = 60.0f + uni_(rng_) * 180.0f;
      raw += (uni_(rng_) < 0.5f ? -mag : mag);
    }
    if (frozen_) raw = frozen_val_;
    else frozen_val_ = raw;
    switch (fault_) {
      case Fault::FLAT: raw = 0.0f; break;
      case Fault::RAIL: raw = FULL_SCALE; break;
      case Fault::STUCK: raw = stuck_val_; break;
      default: break;
    }
    if (raw < v_.clip_lo) raw = v_.clip_lo;
    if (raw > v_.clip_hi) raw = v_.clip_hi;
    return std::round(raw * 4.0f) / 4.0f;
  }

 private:
  const UnitCalib *nom_{nullptr};
  PlantVariant v_;
  std::mt19937 rng_;
  std::normal_distribution<float> gauss_{0.0f, 1.0f};
  std::uniform_real_distribution<float> uni_{0.0f, 1.0f};
  float xs_{50.0f};
  Fault fault_{Fault::NONE};
  float stuck_val_{0.0f};
  bool frozen_{false};
  float frozen_val_{0.0f};
};

}  // namespace slider_sil
