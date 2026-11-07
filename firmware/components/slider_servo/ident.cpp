// L6 Ident implementation. Source of truth: docs/kueche-mief-spec.md §3
// (rows cited inline), deviations D1..D9 documented in
// docs/findings/ident-sil-2026-11-07.md. Pure C++, no heap, no logging.

#include "ident.h"

#include <algorithm>
#include <cmath>

namespace slider_servo {

namespace {
inline float clipf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
inline float sgn(float x) { return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f); }
inline bool in_run(IdentState s) {
  return s >= IdentState::I1_PREFLIGHT && s <= IdentState::I9_COMMIT;
}
inline bool in_motion_phase(IdentState s) {  // rows 27–30: "beliebig I2…I8"
  return s >= IdentState::I2_POLARITY && s <= IdentState::I8_IMPULSES;
}
// percentile with linear interpolation (numpy default) over a small array;
// sorts `v` in place
float percentile(float *v, int n, float q) {
  if (n <= 0) return 0.0f;
  std::sort(v, v + n);
  const float pos = q * static_cast<float>(n - 1);
  const int i = static_cast<int>(pos);
  if (i >= n - 1) return v[n - 1];
  const float t = pos - static_cast<float>(i);
  return v[i] + t * (v[i + 1] - v[i]);
}

// sub-state ids (per phase; uint8_t sub_)
enum : uint8_t {
  S_INIT = 0, S_REPO = 1, S_SETTLE = 2, S_RUN = 3, S_REST = 4, S_FIRE = 5,
  S_MEDIAN = 6, S_EVAL = 7,
};
}  // namespace

const char *ident_state_name(IdentState s) {
  switch (s) {
    case IdentState::I0_OFF: return "off";
    case IdentState::I1_PREFLIGHT: return "preflight";
    case IdentState::I2_POLARITY: return "polarity";
    case IdentState::I3_ENDPOINTS: return "endpoints";
    case IdentState::I4_QUIET: return "quiet";
    case IdentState::I5_BRACKET: return "bracket";
    case IdentState::I6_STALL: return "stall";
    case IdentState::I7_FLIGHTS: return "flights";
    case IdentState::I8_IMPULSES: return "impulses";
    case IdentState::I9_COMMIT: return "commit";
    case IdentState::FAIL: return "fail";
    case IdentState::ABORT: return "abort";
  }
  return "?";
}

const char *ident_reason_name(IdentReason r) {
  switch (r) {
    case IdentReason::NONE: return "none";
    case IdentReason::ADC_FLAT: return "adc_flat";
    case IdentReason::ADC_RAIL: return "adc_rail";
    case IdentReason::ADC_NOISE: return "adc_noise";
    case IdentReason::THERMAL: return "thermal";
    case IdentReason::SUPPLY: return "supply";
    case IdentReason::NO_MOTION: return "no_motion";
    case IdentReason::SPAN: return "span";
    case IdentReason::NO_BREAKAWAY: return "no_breakaway";
    case IdentReason::PLANT_IMPLAUSIBLE: return "plant_implausible";
    case IdentReason::TABLE_IMPLAUSIBLE: return "table_implausible";
    case IdentReason::WATCHDOG: return "watchdog";
    case IdentReason::CMD: return "cmd";
    case IdentReason::HAND: return "hand";
    case IdentReason::INTERLOCK: return "interlock";
    case IdentReason::FAULT: return "fault";
    case IdentReason::REJECTED: return "rejected";
  }
  return "?";
}

uint16_t ident_endpoint_id(const UnitCalib &c) {
  // FNV-1a over the endpoints on a 16-count grid (a re-cal moving either
  // endpoint by more than ~0.2 % changes the id -> dependents invalid, P6)
  const int32_t a = static_cast<int32_t>(std::lround(c.raw_at_0 / 16.0f));
  const int32_t b = static_cast<int32_t>(std::lround(c.raw_at_100 / 16.0f));
  uint32_t h = 2166136261u;
  const int32_t v[2] = {a, b};
  for (int k = 0; k < 2; k++)
    for (int i = 0; i < 4; i++) {
      h ^= static_cast<uint32_t>((v[k] >> (8 * i)) & 0xff);
      h *= 16777619u;
    }
  return static_cast<uint16_t>((h >> 16) ^ (h & 0xffff));
}

bool ident_stamp_valid(const IdentResult &r, uint16_t fw_major, uint8_t instance) {
  return r.valid && r.calib.identified && r.stamp.fw_major == fw_major &&
         r.stamp.instance == instance &&
         r.stamp.endpoint_id == ident_endpoint_id(r.calib);
}

// ------------------------------------------------------------------- setup --

void Ident::init(const IdentConfig *icfg, const ServoConfig *cfg, SeededBug bug) {
  icfg_ = icfg;
  cfg_ = cfg;
  bug_ = bug;
  state_ = IdentState::I0_OFF;
  outcome_ = IdentOutcome::NONE;
  reason_ = IdentReason::NONE;
  just_finished_ = false;
  committed_ = IdentResult{};
  commit_seq_ = 0;
}

bool Ident::start(bool ok_to_start, const UnitCalib &prior,
                  const IdentStamp &prev_stamp, uint32_t date_counter) {
  if (active()) {
    // CMD Ident while I1..I9 is not in §3 -> no-op (fuzz-checked). The seeded
    // KF bug adds the undocumented restart I5 -> I1.
    if (!(SIL_BUG(bug_, KF_ROGUE_IDENT) && state_ == IdentState::I5_BRACKET))
      return false;
  } else if (!ok_to_start) {  // row 2: reject with the reason in the status
    outcome_ = IdentOutcome::REJECTED;
    reason_ = IdentReason::REJECTED;
    return false;
  }
  const IdentConfig &ic = *icfg_;
  work_ = IdentResult{};
  work_.calib = prior;
  prev_stamp_ = prev_stamp;
  date_counter_ = date_counter;
  raw0_ = ic.raw_hint_at_0;
  raw100_ = ic.raw_hint_at_100;
  scale_measured_ = false;
  dir_sign_ = ic.dir_sign_hint >= 0 ? 1 : -1;
  raw8_n_ = raw8_head_ = 0;
  raw32_n_ = raw32_head_ = 0;
  hist_n_ = hist_head_ = 0;
  ring_n_ = ring_head_ = 0;
  err_ema_ = 0.0f;
  zero_ticks_ = 0;
  last_out_ = 0.0f;
  pulse_left_ = settle_left_ = 0;
  repo_duty_ = ic.repo_duty;
  repo_gain_[0] = repo_gain_[1] = ic.repo_gain0;
  repo_prev_ = false;
  fs_min_so_far_ = 2.0f;
  first_point_ = true;
  low_moving_ = 0.0f;
  train_dir_ = -1;
  work_.detail.rail_duty_used = 0.0f;
  train_prog_sum_ = 0.0f;
  train_prog_n_ = 0;
  total_ticks_ = 0;
  outcome_ = IdentOutcome::NONE;
  reason_ = IdentReason::NONE;
  state_ = IdentState::I0_OFF;
  enter(IdentState::I1_PREFLIGHT);  // row 1
  return true;
}

void Ident::abort() {
  if (in_run(state_)) do_abort(IdentReason::CMD);  // row 26
}

void Ident::enter(IdentState s) {
  if (in_run(state_))
    work_.detail.phase_ticks[static_cast<int>(state_)] = phase_ticks_;
  state_ = s;
  phase_ticks_ = 0;
  sub_ = S_INIT;
  timer_ = 0;
  pulse_left_ = settle_left_ = 0;
  if (in_run(s)) {
    const float exp_s = icfg_->phase_expected_s[static_cast<int>(s)];
    phase_limit_ = static_cast<uint32_t>((2.0f * exp_s + 1.0f) * FS_HZ);  // row 29
  }
}

void Ident::fail(IdentReason r) {
  if (in_run(state_))
    work_.detail.phase_ticks[static_cast<int>(state_)] = phase_ticks_;
  state_ = IdentState::FAIL;
  reason_ = r;
  outcome_ = IdentOutcome::FAILED;
  pulse_left_ = settle_left_ = 0;
}

void Ident::do_abort(IdentReason r) {
  if (in_run(state_))
    work_.detail.phase_ticks[static_cast<int>(state_)] = phase_ticks_;
  state_ = IdentState::ABORT;
  reason_ = r;
  outcome_ = IdentOutcome::ABORTED;
  pulse_left_ = settle_left_ = 0;
}

// --------------------------------------------------------- conditioning --

float Ident::to_pct(float raw) const {
  return (raw - raw0_) / (raw100_ - raw0_) * 100.0f;
}

float Ident::med8() const {
  float v[8];
  const int n = raw8_n_;
  for (int i = 0; i < n; i++) v[i] = raw8_[i];
  for (int i = 1; i < n; i++) {  // insertion sort, n <= 8
    const float x = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > x) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = x;
  }
  if (n == 0) return 0.0f;
  return (n % 2) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

float Ident::hist_ago(int ticks) const {
  if (hist_n_ == 0) return raw_med_;
  if (ticks > hist_n_ - 1) ticks = hist_n_ - 1;
  int idx = hist_head_ - 1 - ticks;
  while (idx < 0) idx += IDENT_HIST;
  return hist_[idx];
}

float Ident::mean32() const {
  float s = 0.0f;
  for (int i = 0; i < raw32_n_; i++) s += raw32_[i];
  return raw32_n_ > 0 ? s / static_cast<float>(raw32_n_) : raw_med_;
}

void Ident::buf_push(float raw) {
  if (buf_n_ >= IDENT_BUF) return;
  long q = std::lround(raw * 4.0f);
  if (q > 32767) q = 32767;
  if (q < -32768) q = -32768;
  buf_[buf_n_++] = static_cast<int16_t>(q);
}

float Ident::buf_median() {
  if (buf_n_ == 0) return 0.0f;
  int16_t *mid = buf_ + buf_n_ / 2;
  std::nth_element(buf_, mid, buf_ + buf_n_);
  return static_cast<float>(*mid) * 0.25f;
}

float Ident::buf_mad_sigma(float med) {
  if (buf_n_ == 0) return 0.0f;
  const int32_t mq = static_cast<int32_t>(std::lround(med * 4.0f));
  for (int i = 0; i < buf_n_; i++) {
    int32_t d = static_cast<int32_t>(buf_[i]) - mq;
    if (d < 0) d = -d;
    if (d > 32767) d = 32767;
    buf_[i] = static_cast<int16_t>(d);
  }
  int16_t *mid = buf_ + buf_n_ / 2;
  std::nth_element(buf_, mid, buf_ + buf_n_);
  return static_cast<float>(*mid) * 0.25f * 1.4826f;
}

// ------------------------------------------------------------ pulse engine --

void Ident::pulse(float motor_duty, int width_ms, int settle_ms) {
  pulse_duty_ = motor_duty;
  pulse_left_ = width_ms > 0 ? width_ms : 1;
  settle_left_ = settle_ms;
}

float Ident::pulse_out() {
  if (pulse_left_ > 0) {
    pulse_left_--;
    return pulse_duty_;
  }
  if (settle_left_ > 0) settle_left_--;
  return 0.0f;
}

void Ident::repo_begin(float target) {
  repo_target_ = target;
  repo_prev_ = false;
}

bool Ident::repo_step(float *duty) {
  const IdentConfig &ic = *icfg_;
  *duty = 0.0f;
  if (pulse_busy()) {
    *duty = pulse_out();
    return false;
  }
  const float p = p_med_;
  if (repo_prev_) {  // learn from the pulse that just settled
    const float travel = std::fabs(p - repo_p_before_);
    if (travel < ic.train_min_progress_pct) {
      repo_duty_ = std::min(1.0f, repo_duty_ + ic.repo_escalate);
    } else {
      const int di = repo_dir_prev_ > 0 ? 0 : 1;
      const float g = travel / static_cast<float>(repo_w_prev_);
      repo_gain_[di] = 0.5f * repo_gain_[di] + 0.5f * g;
    }
    repo_prev_ = false;
  }
  const float err = repo_target_ - p;
  if (std::fabs(err) <= ic.repo_tol_pct) return true;
  const int dir = err > 0.0f ? 1 : -1;
  const int di = dir > 0 ? 0 : 1;
  const float w = clipf(ic.repo_aim * std::fabs(err) / repo_gain_[di],
                        ic.repo_w_min_ms, ic.repo_w_max_ms);
  const int wi = static_cast<int>(w + 0.5f);
  repo_p_before_ = p;
  repo_w_prev_ = wi;
  repo_dir_prev_ = dir;
  repo_prev_ = true;
  pulse(static_cast<float>(dir * dir_sign_) * repo_duty_, wi, ic.repo_settle_ms);
  *duty = pulse_out();
  return false;
}

// -------------------------------------------------------------------- tick --

float Ident::tick(const IdentInputs &in) {
  just_finished_ = false;
  if (state_ == IdentState::I0_OFF) return 0.0f;
  if (state_ == IdentState::FAIL || state_ == IdentState::ABORT) {
    // row 31: torque 0, previous stamp stays (committed_ untouched), arbiter free
    state_ = IdentState::I0_OFF;
    just_finished_ = true;
    last_out_ = 0.0f;
    return 0.0f;
  }
  if (state_ == IdentState::I9_COMMIT) {  // row 25
    commit();
    work_.detail.phase_ticks[9] = 1;
    state_ = IdentState::I0_OFF;
    just_finished_ = true;
    last_out_ = 0.0f;
    return 0.0f;
  }
  const IdentConfig &ic = *icfg_;
  total_ticks_++;
  phase_ticks_++;

  // row 32: NaN or a persistent ADC error rate -> ABORT(fault)
  const bool finite = std::isfinite(in.raw);
  const float err_now = (in.adc_err || !finite) ? 1.0f : 0.0f;
  err_ema_ += (err_now - err_ema_) * 0.01f;  // ~100-tick window
  if (!finite || err_ema_ > ic.adc_err_rate_max) {
    do_abort(IdentReason::FAULT);
    last_out_ = 0.0f;
    return 0.0f;
  }

  // conditioning: median-of-8 (principle 1), 32-mean, per-tick history
  raw8_[raw8_head_] = in.raw;
  raw8_head_ = (raw8_head_ + 1) % 8;
  if (raw8_n_ < 8) raw8_n_++;
  raw32_[raw32_head_] = in.raw;
  raw32_head_ = (raw32_head_ + 1) % 32;
  if (raw32_n_ < 32) raw32_n_++;
  raw_med_ = med8();
  p_med_ = to_pct(raw_med_);
  hist_[hist_head_] = raw_med_;
  hist_head_ = (hist_head_ + 1) % IDENT_HIST;
  if (hist_n_ < IDENT_HIST) hist_n_++;
  duty_ring_[ring_head_] = std::fabs(last_out_);  // row 15 32er-Ring
  pos_ring_[ring_head_] = raw_med_;
  ring_head_ = (ring_head_ + 1) % 32;
  if (ring_n_ < 32) ring_n_++;

  if (in_motion_phase(state_)) {
    if (in.l1_cut) {  // row 28: L1 is supreme
      do_abort(IdentReason::INTERLOCK);
      last_out_ = 0.0f;
      return 0.0f;
    }
    const float vm = work_.detail.vm;
    if (vm > 0.0f && in.supply_v >= 0.0f &&
        std::fabs(in.supply_v - vm) > ic.supply_dev * vm) {  // row 30
      do_abort(IdentReason::SUPPLY);
      last_out_ = 0.0f;
      return 0.0f;
    }
    if (phase_ticks_ > phase_limit_) {  // row 29
      fail(IdentReason::WATCHDOG);
      last_out_ = 0.0f;
      return 0.0f;
    }
    // row 27: motion at duty 0 (coast excluded by the guard) = a hand
    if (!SIL_BUG(bug_, K5_NO_HAND) && zero_ticks_ > ic.hand_guard_ms) {
      int w = zero_ticks_ - ic.hand_guard_ms;
      if (w > ic.hand_win_ms) w = ic.hand_win_ms;
      if (w >= 10) {
        const float moved = std::fabs(to_pct(raw_med_) - to_pct(hist_ago(w)));
        if (moved > ic.hand_pct) {
          do_abort(IdentReason::HAND);
          last_out_ = 0.0f;
          return 0.0f;
        }
      }
    }
  }

  float out = 0.0f;
  switch (state_) {
    case IdentState::I1_PREFLIGHT: out = tick_preflight(in); break;
    case IdentState::I2_POLARITY: out = tick_polarity(); break;
    case IdentState::I3_ENDPOINTS: out = tick_endpoints(); break;
    case IdentState::I4_QUIET: out = tick_quiet(); break;
    case IdentState::I5_BRACKET: out = tick_bracket(); break;
    case IdentState::I6_STALL: out = tick_stall(); break;
    case IdentState::I7_FLIGHTS: out = tick_flights(); break;
    case IdentState::I8_IMPULSES: out = tick_impulses(); break;
    default: break;
  }
  if (state_ == IdentState::FAIL || state_ == IdentState::ABORT) out = 0.0f;
  zero_ticks_ = out != 0.0f ? 0 : zero_ticks_ + 1;
  last_out_ = out;
  return out;
}

// ---------------------------------------------------------- I1 PREFLIGHT --

float Ident::tick_preflight(const IdentInputs &in) {
  const IdentConfig &ic = *icfg_;
  if (sub_ == S_INIT) {
    buf_reset();
    supply_sum_ = 0.0f;
    supply_n_ = 0;
    sub_ = S_RUN;
  }
  // row 3 (changed 2026-11-07): L1 not tripped and EMA below the resume level
  if (in.thermal_tripped || in.thermal_ema >= ic.thermal_preflight_max) {
    fail(IdentReason::THERMAL);
    return 0.0f;
  }
  buf_push(in.raw);
  if (in.supply_v >= 0.0f) {
    supply_sum_ += in.supply_v;
    supply_n_++;
  }
  if (++timer_ < ic.preflight_ms) return 0.0f;

  const float med = buf_median();
  const float sigma = buf_mad_sigma(med);
  const bool window = ic.supply_lo < ic.supply_hi;
  const float vm = supply_n_ > 0 ? supply_sum_ / static_cast<float>(supply_n_) : -1.0f;
  if (med <= ic.adc_flat_max) {
    fail(IdentReason::ADC_FLAT);
  } else if (med >= ic.adc_full_scale - ic.adc_rail_margin) {
    fail(IdentReason::ADC_RAIL);
  } else if (sigma < ic.sigma_min_counts || sigma > ic.sigma_max_counts) {
    fail(IdentReason::ADC_NOISE);  // D2
  } else if (window && (vm < ic.supply_lo || vm > ic.supply_hi)) {
    fail(IdentReason::SUPPLY);
  } else {
    work_.detail.vm = vm;  // row 3: VM-Wert stempeln
    enter(IdentState::I2_POLARITY);
  }
  return 0.0f;
}

// ---------------------------------------------------------- I2 POLARITY --

float Ident::tick_polarity() {
  const IdentConfig &ic = *icfg_;
  if (sub_ == S_INIT) {
    const float p = p_med_;
    if (p >= ic.mid_lo && p <= ic.mid_hi) {
      probe_sign_ = 1.0f;  // row 5: Duty-Vorzeichen +
    } else {               // row 6: first probe toward the middle (hint scale)
      const float want = p < ic.mid_lo ? 1.0f : -1.0f;
      probe_sign_ = want * static_cast<float>(ic.dir_sign_hint >= 0 ? 1 : -1);
    }
    probe_pass_ = 0;
    probe_idx_ = 0;
    sub_ = S_FIRE;
  }
  if (sub_ == S_FIRE) {
    probe_ref_ = raw_med_;
    pulse(probe_sign_ * ic.probe_duty,
          static_cast<int>(ic.probe_widths_ms[probe_idx_] + 0.5f), ic.probe_rest_ms);
    sub_ = S_REST;
    return pulse_out();
  }
  // S_REST
  if (pulse_busy()) return pulse_out();
  const float d = to_pct(raw_med_) - to_pct(probe_ref_);
  if (std::fabs(d) >= ic.probe_motion_pct) {  // row 7
    dir_sign_ = static_cast<int8_t>(sgn(d) * sgn(probe_sign_));
    if (SIL_BUG(bug_, K3_DIR_FIXED)) dir_sign_ = 1;
    enter(IdentState::I3_ENDPOINTS);
    return 0.0f;
  }
  if (++probe_idx_ >= IDENT_PROBE_WIDTHS) {
    probe_idx_ = 0;
    probe_sign_ = -probe_sign_;  // D1: the repetition probes the other way
    if (++probe_pass_ >= ic.probe_passes) {  // row 8
      fail(IdentReason::NO_MOTION);
      return 0.0f;
    }
  }
  sub_ = S_FIRE;
  return 0.0f;
}

// --------------------------------------------------------- I3 ENDPOINTS --

float Ident::tick_endpoints() {
  const IdentConfig &ic = *icfg_;
  if (sub_ == S_INIT) {  // a train toward rail A (-p), later B (+p)
    train_duty_ = ic.train_duty;
    train_stall_ = 0;
    train_count_ = 0;
    train_prev_ = raw_med_;
    sub_ = S_RUN;
  }
  if (sub_ == S_RUN) {  // row 9
    if (pulse_busy()) return pulse_out();
    if (train_count_ > 0) {
      const float prog = std::fabs(to_pct(raw_med_) - to_pct(train_prev_));
      if (prog < ic.train_min_progress_pct) {
        train_stall_++;
      } else {
        train_stall_ = 0;
        if (train_duty_ == ic.train_duty) {
          train_prog_sum_ += prog;
          train_prog_n_++;
        }
      }
    }
    if (train_stall_ >= ic.train_stall_pulses) {
      if (train_duty_ < ic.rail_confirm_duty) {  // D3: confirm at full duty
        train_duty_ = ic.rail_confirm_duty;
        train_stall_ = 0;
      } else {  // row 10: rail reached
        if (work_.detail.rail_duty_used < train_duty_)
          work_.detail.rail_duty_used = train_duty_;
        sub_ = S_REST;
        timer_ = 0;
        return 0.0f;
      }
    }
    train_prev_ = raw_med_;
    train_count_++;
    pulse(static_cast<float>(train_dir_ * dir_sign_) * train_duty_, ic.train_width_ms,
          ic.train_period_ms - ic.train_width_ms);
    return pulse_out();
  }
  if (sub_ == S_REST) {  // row 10: 0.5 s rest
    if (++timer_ >= ic.rail_rest_ms) {
      buf_reset();
      timer_ = 0;
      sub_ = S_MEDIAN;
    }
    return 0.0f;
  }
  if (sub_ == S_MEDIAN) {  // row 10: median of 200 ticks
    buf_push(raw8_[(raw8_head_ + 7) % 8]);  // this tick's raw sample
    if (++timer_ < ic.rail_median_ticks) return 0.0f;
    const float med = buf_median();
    const float sig = buf_mad_sigma(med);
    if (train_dir_ < 0) {
      raw_a_ = med;
      sig_a_ = sig;
      train_dir_ = 1;  // now the B leg
      sub_ = S_INIT;
      return 0.0f;
    }
    raw_b_ = med;
    sig_b_ = sig;
    sub_ = S_EVAL;
  }
  // S_EVAL: rows 11/12
  const float span = std::fabs(raw_b_ - raw_a_);
  const float expect = sgn(ic.raw_hint_at_100 - ic.raw_hint_at_0);
  const bool same_side = sgn(raw_b_ - raw_a_) != expect;
  const bool sig_ok = sig_a_ >= ic.sigma_min_counts && sig_a_ <= ic.sigma_max_counts &&
                      sig_b_ >= ic.sigma_min_counts && sig_b_ <= ic.sigma_max_counts;
  train_dir_ = -1;
  if (span < ic.min_span_counts || same_side || !sig_ok) {
    fail(IdentReason::SPAN);
    return 0.0f;
  }
  // reposition gain prior from the train (hint-% -> rail-%)
  const float hint_span = std::fabs(ic.raw_hint_at_100 - ic.raw_hint_at_0);
  raw0_ = raw_a_;
  raw100_ = raw_b_;
  scale_measured_ = true;
  if (train_prog_n_ > 0) {
    const float prog = train_prog_sum_ / static_cast<float>(train_prog_n_) *
                       hint_span / span;
    repo_gain_[0] = repo_gain_[1] = prog / static_cast<float>(ic.train_width_ms);
  }
  p_med_ = to_pct(raw_med_);
  UnitCalib &c = work_.calib;
  c.raw_at_0 = raw_a_;
  c.raw_at_100 = raw_b_;
  c.counts_per_percent = span / 100.0f;
  c.span_lo = ic.span_margin_pct;
  c.span_hi = 100.0f - ic.span_margin_pct;
  c.dir_sign = dir_sign_;
  enter(IdentState::I4_QUIET);
  return 0.0f;
}

// ------------------------------------------------------------- I4 QUIET --

float Ident::tick_quiet() {
  const IdentConfig &ic = *icfg_;
  if (sub_ == S_INIT) {
    buf_reset();
    dec_ = 0;
    sub_ = S_RUN;
  }
  if (dec_++ % ic.decimate == 0) buf_push(raw8_[(raw8_head_ + 7) % 8]);
  if (++timer_ >= ic.quiet_ms) {  // row 13: σ0 := MAD * 1.48
    const float med = buf_median();
    work_.detail.sigma0_counts = buf_mad_sigma(med);
    enter(IdentState::I5_BRACKET);
    bp_pos_ = bp_dir_ = bp_rep_ = 0;
  }
  return 0.0f;
}

// ----------------------------------------------------------- I5 BRACKET --
// Rows 14–17. Per point (position, direction, rep): reposition -> rest ->
// coarse holds upward from the (adaptive) start in 0.05 steps until the
// binary >= 5 % classifier fires -> duty 0 -> onset from the 32-ring -> fine
// holds 0.03 downward (each preceded by a reposition) until one stays quiet.
// F_s(point) = the lowest duty that moved (upper bound, within +0.03).

float Ident::tick_bracket() {
  const IdentConfig &ic = *icfg_;
  const float s = bp_dir_ == 0 ? 1.0f : -1.0f;
  // fine mode <=> low_moving_ > 0 (a coarse hit exists for this point)
  switch (sub_) {
    case S_INIT:
      repo_begin(ic.positions_pct[bp_pos_]);
      sub_ = S_REPO;
      [[fallthrough]];
    case S_REPO: {
      float d;
      if (!repo_step(&d)) return d;
      timer_ = 0;
      sub_ = S_SETTLE;
      return 0.0f;
    }
    case S_SETTLE:
      if (++timer_ < ic.pre_hold_settle_ms) return 0.0f;
      hold_ref_ = raw_med_;
      if (low_moving_ > 0.0f) {  // fine step, downward-confirmed
        hold_duty_ = low_moving_ - ic.bracket_fine;
      } else if (SIL_BUG(bug_, K2_BRACKET_RIG)) {
        hold_duty_ = 0.60f;  // the rig constant (K2 negative control)
      } else if (first_point_) {
        hold_duty_ = ic.bracket_start;
      } else {  // D5 adaptive start on the 0.05 grid, never below the start:
        // rep 0 from the lowest F_s so far, rep 1 from this point's rep 0
        const float ref = bp_rep_ > 0 ? work_.detail.fs_rep[bp_dir_][bp_pos_][0]
                                      : fs_min_so_far_;
        const float want = ref - ic.bracket_adapt_margin;
        const float k = std::floor((want - ic.bracket_start) / ic.bracket_step + 1e-4f);
        hold_duty_ = k > 0.0f ? ic.bracket_start + k * ic.bracket_step : ic.bracket_start;
      }
      timer_ = 0;
      sub_ = S_RUN;
      return 0.0f;
    case S_RUN: {
      const bool fine = low_moving_ > 0.0f;
      const float moved = std::fabs(to_pct(raw_med_) - to_pct(hold_ref_));
      if (moved >= ic.bracket_motion_pct) {  // row 15: duty 0 at once
        float hit = hold_duty_;
        if (!fine) {
          // onset := duty at the last quiet tick (32-ring, newest first)
          for (int k = 0; k < ring_n_; k++) {
            int idx = ring_head_ - 1 - k;
            while (idx < 0) idx += 32;
            const float dp = std::fabs(to_pct(pos_ring_[idx]) - to_pct(hold_ref_));
            if (dp < ic.quiet_band_pct && duty_ring_[idx] > 0.0f) {
              hit = duty_ring_[idx];
              break;
            }
          }
          // sustain bit: >= 3 % in the 50 ms before the trip (D4)
          const float sus = std::fabs(to_pct(raw_med_) - to_pct(hist_ago(ic.sustain_ms)));
          if (sus >= ic.sustain_pct) work_.detail.sustain_bits[bp_dir_] |= (1u << bp_pos_);
        }
        low_moving_ = hit;
        if (low_moving_ - ic.bracket_fine >= ic.bracket_start - 1e-4f) {
          repo_begin(ic.positions_pct[bp_pos_]);
          sub_ = S_REPO;  // confirm one fine step lower
        } else {
          sub_ = S_EVAL;  // floor reached: point done
        }
        return 0.0f;
      }
      if (++timer_ >= ic.hold_ms) {
        timer_ = 0;
        if (fine) {  // quiet at low_moving_ - 0.03 -> confirmed
          sub_ = S_EVAL;
          return 0.0f;
        }
        hold_duty_ += ic.bracket_step;
        if (hold_duty_ > ic.bracket_max + 1e-4f) {  // row 16
          fail(IdentReason::NO_BREAKAWAY);
          return 0.0f;
        }
      }
      return s * static_cast<float>(dir_sign_) * hold_duty_;
    }
    case S_EVAL:
    default: {
      work_.detail.fs_rep[bp_dir_][bp_pos_][bp_rep_] = low_moving_;
      if (low_moving_ < fs_min_so_far_) fs_min_so_far_ = low_moving_;
      first_point_ = false;
      low_moving_ = 0.0f;
      const int reps = SIL_BUG(bug_, K9_SINGLE_PROBE) ? 1 : IDENT_REPS;
      if (++bp_rep_ >= reps) {
        bp_rep_ = 0;
        if (++bp_dir_ >= 2) {
          bp_dir_ = 0;
          if (++bp_pos_ >= IDENT_POSITIONS) {
            finish_bracket();
            return 0.0f;
          }
        }
      }
      sub_ = S_INIT;
      return 0.0f;
    }
  }
}

void Ident::derive_r7() {
  const IdentConfig &ic = *icfg_;
  UnitCalib &c = work_.calib;
  const float p95 = work_.detail.fs_p95;
  c.impulse_duty = clipf(p95 + ic.impulse_margin, ic.impulse_lo, ic.impulse_hi);  // R7a
  c.u_max = clipf(p95 + ic.umax_margin, c.impulse_duty, ic.umax_hi);              // R7b
  if (SIL_BUG(bug_, K7_RIG_DUTY)) {
    c.impulse_duty = 0.8f;
    c.u_max = 0.9f;
  }
}

void Ident::finish_bracket() {
  const IdentConfig &ic = *icfg_;
  IdentDetail &d = work_.detail;
  const bool single = SIL_BUG(bug_, K9_SINGLE_PROBE);
  float all[2 * IDENT_POSITIONS * IDENT_REPS];
  int n = 0;
  for (int dir = 0; dir < 2; dir++)
    for (int p = 0; p < IDENT_POSITIONS; p++) {
      float sum = 0.0f;
      const int reps = single ? 1 : IDENT_REPS;
      for (int r = 0; r < reps; r++) {
        sum += d.fs_rep[dir][p][r];
        all[n++] = d.fs_rep[dir][p][r];
      }
      d.fs[dir][p] = sum / static_cast<float>(reps);
    }
  d.fs_med = percentile(all, n, 0.5f);
  d.fs_min = all[0];
  d.fs_p95 = percentile(all, n, 0.95f);
  if (d.fs_p95 < ic.fs_p95_lo || d.fs_p95 > ic.fs_p95_hi) {  // row 17 (D6)
    fail(IdentReason::PLANT_IMPLAUSIBLE);
    return;
  }
  // 7 measured points -> 21-node maps: linear between points, flat beyond
  UnitCalib &c = work_.calib;
  for (int i = 0; i < STATIC_NODES; i++) {
    const float x = 5.0f * static_cast<float>(i);
    for (int dir = 0; dir < 2; dir++) {
      const float *xs = ic.positions_pct;
      const float *ys = d.fs[dir];
      float y;
      if (x <= xs[0]) {
        y = ys[0];
      } else if (x >= xs[IDENT_POSITIONS - 1]) {
        y = ys[IDENT_POSITIONS - 1];
      } else {
        int k = 1;
        while (k < IDENT_POSITIONS - 1 && xs[k] < x) k++;
        const float t = (x - xs[k - 1]) / (xs[k] - xs[k - 1]);
        y = ys[k - 1] + t * (ys[k] - ys[k - 1]);
      }
      (dir == 0 ? c.fs_fwd : c.fs_rev)[i] = y;
    }
  }
  derive_r7();
  if (SIL_BUG(bug_, K4_EARLY_COMMIT)) {  // NVS before I9 (K4 negative control)
    work_.valid = true;
    committed_ = work_;
    commit_seq_++;
  }
  enter(IdentState::I6_STALL);
}

// ------------------------------------------------------------- I6 STALL --

float Ident::tick_stall() {
  const IdentConfig &ic = *icfg_;
  IdentDetail &d = work_.detail;
  if (sub_ == S_INIT) {
    stall_duty_ = SIL_BUG(bug_, K2_BRACKET_RIG)
                      ? 0.45f
                      : std::max(ic.stall_floor, d.fs_min - ic.stall_below_fs);  // row 18
    d.stall_duty = stall_duty_;
    probe_sign_ = p_med_ < 50.0f ? 1.0f : -1.0f;  // toward the middle
    stall_ref_ = raw_med_;
    buf_reset();
    dec_ = 0;
    spikes_ = 0;
    stall_ticks_ = 0;
    sub_ = S_RUN;
  }
  const float raw_now = raw8_[(raw8_head_ + 7) % 8];
  if (dec_++ % ic.decimate == 0) buf_push(raw_now);
  if (std::fabs(raw_now - raw_med_) > ic.spike_sigma * d.sigma0_counts) spikes_++;
  stall_ticks_++;
  const float moved = std::fabs(to_pct(raw_med_) - to_pct(stall_ref_));
  if (moved > d.stall_motion_pct) d.stall_motion_pct = moved;
  if (moved >= ic.bracket_motion_pct) {  // D6b: a "stall" that moves is no stall
    fail(IdentReason::PLANT_IMPLAUSIBLE);
    return 0.0f;
  }
  if (stall_ticks_ >= ic.stall_ms) {
    const float med = buf_median();
    d.sigma_pwm_counts = buf_mad_sigma(med);
    d.spike_rate = static_cast<float>(spikes_) / static_cast<float>(stall_ticks_);
    enter(IdentState::I7_FLIGHTS);
    fl_dir_ = fl_duty_ = fl_rep_ = 0;
    return 0.0f;
  }
  return probe_sign_ * static_cast<float>(dir_sign_) * stall_duty_;
}

// ----------------------------------------------------------- I7 FLIGHTS --

float Ident::tick_flights() {
  const IdentConfig &ic = *icfg_;
  IdentDetail &d = work_.detail;
  const float s = fl_dir_ == 0 ? 1.0f : -1.0f;
  if (sub_ == S_INIT) {
    const float d2 = std::min(d.fs_p95 + ic.flight_d2, ic.flight_max);  // row 19, D8
    d.flight_duty[1] = d2;
    d.flight_duty[0] = std::min(d.fs_p95 + ic.flight_d1, d2 - ic.flight_min_gap);
    repo_begin(fl_dir_ == 0 ? ic.flight_from : ic.flight_to);
    sub_ = S_REPO;
  }
  switch (sub_) {
    case S_REPO: {
      float duty;
      if (!repo_step(&duty)) return duty;
      timer_ = 0;
      sub_ = S_SETTLE;
      return 0.0f;
    }
    case S_SETTLE:
      if (++timer_ < ic.pre_hold_settle_ms) return 0.0f;
      timer_ = 0;
      sub_ = S_RUN;
      return 0.0f;
    case S_RUN: {
      timer_++;
      const bool arrived = s > 0.0f ? p_med_ >= ic.flight_to : p_med_ <= ic.flight_from;
      if (arrived || timer_ >= ic.flight_timeout_ms) {
        const int k = std::min(ic.flight_slope_ms, timer_ - 1);
        const float v = k > 0 ? s * (to_pct(raw_med_) - to_pct(hist_ago(k))) /
                                    (static_cast<float>(k) * DT)
                              : 0.0f;
        d.flight_v[fl_dir_][fl_duty_][fl_rep_] = v;
        timer_ = 0;
        sub_ = S_REST;
        return 0.0f;
      }
      return s * static_cast<float>(dir_sign_) * d.flight_duty[fl_duty_];
    }
    case S_REST:
    default: {
      if (++timer_ < ic.pre_hold_settle_ms) return 0.0f;
      const int reps = SIL_BUG(bug_, K9_SINGLE_PROBE) ? 1 : ic.flight_reps;
      if (++fl_rep_ >= reps) {
        fl_rep_ = 0;
        if (++fl_duty_ >= 2) {
          fl_duty_ = 0;
          if (++fl_dir_ >= 2) {
            if (finish_flights()) {
              enter(IdentState::I8_IMPULSES);
              im_dir_ = im_rep_ = im_w_ = 0;
              im_pulses_ = 0;
            }
            return 0.0f;
          }
        }
      }
      sub_ = S_INIT;
      return 0.0f;
    }
  }
}

bool Ident::finish_flights() {
  const IdentConfig &ic = *icfg_;
  IdentDetail &d = work_.detail;
  UnitCalib &c = work_.calib;
  const int reps = SIL_BUG(bug_, K9_SINGLE_PROBE) ? 1 : ic.flight_reps;
  float fk[2], b[2];
  for (int dir = 0; dir < 2; dir++) {
    float v[2];
    for (int k = 0; k < 2; k++) {
      float sum = 0.0f;
      for (int r = 0; r < reps; r++) sum += d.flight_v[dir][k][r];
      v[k] = sum / static_cast<float>(reps);
    }
    const float dv = v[1] - v[0];
    b[dir] = dv > 0.0f ? (d.flight_duty[1] - d.flight_duty[0]) / dv : -1.0f;
    fk[dir] = d.flight_duty[0] - b[dir] * v[0];
    // row 21 guard against this direction's breakaway (D7b: p95 per direction)
    float fsd[IDENT_POSITIONS * IDENT_REPS];
    int n = 0;
    for (int p = 0; p < IDENT_POSITIONS; p++)
      for (int r = 0; r < (SIL_BUG(bug_, K9_SINGLE_PROBE) ? 1 : IDENT_REPS); r++)
        fsd[n++] = d.fs_rep[dir][p][r];
    const float fs95 = percentile(fsd, n, 0.95f);
    if (b[dir] <= 0.0f || fk[dir] >= fs95) {
      fail(IdentReason::PLANT_IMPLAUSIBLE);
      return false;
    }
  }
  c.fk_mid_fwd = fk[0];
  c.fk_mid_rev = fk[1];
  c.b_fwd = b[0];
  c.b_rev = b[1];
  c.b = 0.5f * (b[0] + b[1]);
  for (int i = 0; i < KINETIC_NODES; i++) {  // flat maps (auto-ident #6 PARTIAL)
    c.fk_fwd[i] = fk[0];
    c.fk_rev[i] = fk[1];
  }
  // R7c: flight speed one bracket step above the cliff, from the measured law
  const float fk_mean = 0.5f * (fk[0] + fk[1]);
  const float v_hit = (d.fs_med + ic.bracket_step - fk_mean) / c.b;
  c.v_min_cmd = clipf(ic.vmin_margin * v_hit, ic.vmin_lo, cfg_->v_cruise);
  return true;
}

// ---------------------------------------------------------- I8 IMPULSES --

float Ident::tick_impulses() {
  const IdentConfig &ic = *icfg_;
  IdentDetail &d = work_.detail;
  const float s = im_dir_ == 0 ? 1.0f : -1.0f;
  const float duty = work_.calib.impulse_duty;  // R7a (row 22)
  switch (sub_) {
    case S_INIT:  // D9b: approach the start from behind (backlash taken up
                  // in the pulse direction, the T7 idea applied to the table)
      repo_begin((im_dir_ == 0 ? ic.imp_start_fwd : ic.imp_start_rev) -
                 s * ic.imp_preload_pct);
      sub_ = S_REPO;
      [[fallthrough]];
    case S_REPO: {
      float dd;
      if (!repo_step(&dd)) return dd;
      repo_begin(im_dir_ == 0 ? ic.imp_start_fwd : ic.imp_start_rev);
      sub_ = S_MEDIAN;
      return 0.0f;
    }
    case S_MEDIAN: {
      float dd;
      if (!repo_step(&dd)) return dd;
      timer_ = 0;
      sub_ = S_SETTLE;
      return 0.0f;
    }
    case S_SETTLE:
      if (++timer_ < ic.pre_hold_settle_ms) return 0.0f;
      timer_ = 1;  // "just repositioned" for the room check
      sub_ = S_FIRE;
      [[fallthrough]];
    case S_FIRE: {
      const float w = ic.imp_widths_ms[im_w_];
      if (im_w_ > 0 && timer_ == 0) {  // room check (D9) unless just re-centred
        const float pred = im_last_travel_ * (w / ic.imp_widths_ms[im_w_ - 1]) *
                               ic.imp_predict + 1.0f;
        const float after = p_med_ + s * pred;
        if (after < ic.imp_guard_lo || after > ic.imp_guard_hi) {
          sub_ = S_INIT;
          return 0.0f;
        }
      }
      im_ref_ = mean32();
      pulse(s * static_cast<float>(dir_sign_) * duty, static_cast<int>(w + 0.5f),
            ic.imp_settle_ms);
      im_pulses_++;
      sub_ = S_RUN;
      return pulse_out();
    }
    case S_RUN:
    default: {
      if (pulse_busy()) return pulse_out();
      const float travel = s * (to_pct(mean32()) - to_pct(im_ref_));
      d.imp_travel[im_dir_][im_w_][im_rep_] = travel;
      im_last_travel_ = travel > 0.0f ? travel : 0.0f;
      timer_ = 0;
      sub_ = S_FIRE;
      if (++im_w_ >= IDENT_IMP_WIDTHS) {
        im_w_ = 0;
        sub_ = S_INIT;  // group done: back to the start position
        const int reps = SIL_BUG(bug_, K9_SINGLE_PROBE) ? 1 : ic.imp_reps;
        if (++im_rep_ >= reps) {
          im_rep_ = 0;
          if (++im_dir_ >= 2) {
            if (finish_impulses()) enter(IdentState::I9_COMMIT);
            return 0.0f;
          }
        }
      }
      return 0.0f;
    }
  }
}

bool Ident::finish_impulses() {
  const IdentConfig &ic = *icfg_;
  IdentDetail &d = work_.detail;
  UnitCalib &c = work_.calib;
  const int reps = SIL_BUG(bug_, K9_SINGLE_PROBE) ? 1 : ic.imp_reps;
  for (int dir = 0; dir < 2; dir++)
    for (int w = 0; w < IDENT_IMP_WIDTHS; w++) {
      float v[IDENT_REPS];
      for (int r = 0; r < reps; r++) v[r] = d.imp_travel[dir][w][r];
      d.imp_table[dir][w] = percentile(v, reps, 0.5f);  // median per width
    }
  for (int dir = 0; dir < 2; dir++) {  // row 23/24: monotone in width, > 0
    if (d.imp_table[dir][0] <= 0.0f) {
      fail(IdentReason::TABLE_IMPLAUSIBLE);
      return false;
    }
    for (int w = 1; w < IDENT_IMP_WIDTHS; w++)
      if (d.imp_table[dir][w] <= d.imp_table[dir][w - 1]) {
        fail(IdentReason::TABLE_IMPLAUSIBLE);
        return false;
      }
  }
  // UnitCalib tables: rev = the 6 measured points; fwd keeps its 8-point grid
  // {2,3,5,8,10,20,30,50}: 2 ms = 2/3 of the 3 ms travel (through the origin,
  // conservative), 8 ms = linear between 5 and 10 (what interp does anyway)
  const float *t = d.imp_table[0];
  const float fw[FWD_PULSE_POINTS] = {2, 3, 5, 8, 10, 20, 30, 50};
  const float ft[FWD_PULSE_POINTS] = {t[0] * 2.0f / 3.0f, t[0], t[1],
                                      t[1] + (t[2] - t[1]) * 0.6f, t[2], t[3], t[4], t[5]};
  for (int i = 0; i < FWD_PULSE_POINTS; i++) {
    c.fwd_widths[i] = fw[i];
    c.fwd_travel[i] = ft[i];
  }
  for (int i = 0; i < REV_PULSE_POINTS; i++) {
    c.rev_widths[i] = ic.imp_widths_ms[i];
    c.rev_travel[i] = d.imp_table[1][i];
  }
  return true;
}

// ------------------------------------------------------------- I9 COMMIT --

void Ident::commit() {
  const IdentConfig &ic = *icfg_;
  IdentDetail &d = work_.detail;
  UnitCalib &c = work_.calib;
  c.sigma0_counts = d.sigma0_counts;
  c.sigma_pwm_counts = d.sigma_pwm_counts > d.sigma0_counts ? d.sigma_pwm_counts
                                                            : d.sigma0_counts;
  c.spike_rate = d.spike_rate;
  c.dir_sign = dir_sign_;
  c.derived = true;
  c.identified = true;
  d.total_ticks = total_ticks_;
  IdentStamp &st = work_.stamp;
  st.fw_major = ic.fw_major;
  st.instance = ic.instance;
  st.vm_bucket = d.vm >= 0.0f
                     ? static_cast<uint8_t>(std::min(254.0f, std::round(d.vm * 10.0f)))
                     : 255;
  st.endpoint_id = ident_endpoint_id(c);
  st.date_counter = date_counter_;
  st.commit_counter = prev_stamp_.commit_counter + 1;
  work_.valid = true;
  // atomic publication: one copy, then the sequence bump the wrapper polls
  committed_ = work_;
  commit_seq_++;
  outcome_ = IdentOutcome::OK;
  reason_ = IdentReason::NONE;
}

// --------------------------------------------------------------- progress --

float Ident::phase_progress() const {
  if (!in_run(state_)) return 0.0f;
  const IdentConfig &ic = *icfg_;
  switch (state_) {
    case IdentState::I5_BRACKET:
      return static_cast<float>((bp_pos_ * 2 + bp_dir_) * IDENT_REPS + bp_rep_) /
             static_cast<float>(IDENT_POSITIONS * 2 * IDENT_REPS);
    case IdentState::I7_FLIGHTS:
      return static_cast<float>((fl_dir_ * 2 + fl_duty_) * ic.flight_reps + fl_rep_) /
             static_cast<float>(4 * ic.flight_reps);
    case IdentState::I8_IMPULSES:
      return static_cast<float>(im_pulses_) /
             static_cast<float>(2 * ic.imp_reps * IDENT_IMP_WIDTHS);
    default: {
      const float exp_ticks = ic.phase_expected_s[static_cast<int>(state_)] * FS_HZ;
      if (exp_ticks <= 0.0f) return 1.0f;
      const float f = static_cast<float>(phase_ticks_) / exp_ticks;
      return f > 1.0f ? 1.0f : f;
    }
  }
}

}  // namespace slider_servo
