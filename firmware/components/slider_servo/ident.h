#pragma once

// L6 Ident — on-device self-identification (kueche-mief-spec.md §3, rows
// 1–32; methods from auto-ident-spec.md, corrected by kueche-mief-spec §5).
//
// Pure C++, zero IDF includes, no function-static state, no heap: every
// buffer is a fixed member array (sized at compile time, "allocated" with the
// object). No logging in the tick. The Ident works on RAW ADC counts and its
// OWN commanded duty only — it never sees a plant model (the SIL truth plant
// is invisible by construction) and it emits MOTOR duty (the sign the
// H-bridge gets), because the polarity is what it is learning (R6).
//
// Ownership: while active (I1..I9) the Ident owns the motor exclusively; the
// Servo puts the controller into State::IDENT and passes the Ident's duty
// through L1, which stays supreme (thermal / abort clamp -> row 28).
//
// Result publication (K4): the identified UnitCalib + stamp becomes visible
// ONLY in I9 (committed(), commit_seq() increments once). Everything before
// I9 lives in a private work copy; FAIL/ABORT leave the committed copy
// bit-identical. The wrapper persists committed() to NVS from its main task
// when commit_seq() changes — never from the tick.

#include <cstdint>

#include "servo_params.h"
#include "sil_bugs.h"

namespace slider_servo {

enum class IdentState : uint8_t {
  I0_OFF = 0,
  I1_PREFLIGHT = 1,
  I2_POLARITY = 2,
  I3_ENDPOINTS = 3,
  I4_QUIET = 4,
  I5_BRACKET = 5,
  I6_STALL = 6,
  I7_FLIGHTS = 7,
  I8_IMPULSES = 8,
  I9_COMMIT = 9,
  FAIL = 10,
  ABORT = 11,
};

// FAIL reasons (rows 4, 8, 12, 16, 21, 24, 29 + D2 adc_noise, D6) and ABORT
// reasons (rows 26, 27, 28, 30, 32), plus the row-2 rejection.
enum class IdentReason : uint8_t {
  NONE = 0,
  ADC_FLAT = 1,
  ADC_RAIL = 2,
  ADC_NOISE = 3,          // D2: row 3 checks σ but row 4 names no reason
  THERMAL = 4,
  SUPPLY = 5,             // row 4 (FAIL) and row 30 (ABORT) share the name
  NO_MOTION = 6,
  SPAN = 7,
  NO_BREAKAWAY = 8,
  PLANT_IMPLAUSIBLE = 9,
  TABLE_IMPLAUSIBLE = 10,
  WATCHDOG = 11,
  CMD = 12,               // row 26 CMD Abbruch
  HAND = 13,              // row 27
  INTERLOCK = 14,         // row 28
  FAULT = 15,             // row 32
  REJECTED = 16,          // row 2 (not a run: instance busy / RELEASED / fault)
};

enum class IdentOutcome : uint8_t { NONE = 0, OK = 1, FAILED = 2, ABORTED = 3, REJECTED = 4 };

const char *ident_state_name(IdentState s);
const char *ident_reason_name(IdentReason r);

// Validity stamp (§5). Invalid when fw major, instance or endpoint id mismatch.
struct IdentStamp {
  uint16_t fw_major = 0;
  uint8_t instance = 0;
  uint8_t vm_bucket = 255;      // supply / 0.1 V, 255 = no supply ADC
  uint16_t endpoint_id = 0;     // hash of raw_at_0 / raw_at_100 (16-count grid)
  uint16_t reserved = 0;
  uint32_t date_counter = 0;    // wrapper-provided (e.g. days since epoch)
  uint32_t commit_counter = 0;  // previous + 1
};

// Raw measurements behind the UnitCalib (report / plantreport cross-check).
struct IdentDetail {
  float fs_rep[2][IDENT_POSITIONS][IDENT_REPS] = {};  // [dir fwd/rev][pos][rep]
  float fs[2][IDENT_POSITIONS] = {};                  // averaged per point
  uint16_t sustain_bits[2] = {0, 0};                  // bit = position, rep 0|1 or'ed
  float fs_p95 = 0.0f;
  float fs_min = 0.0f;
  float fs_med = 0.0f;
  float flight_duty[2] = {0.0f, 0.0f};                // d1, d2
  float flight_v[2][2][IDENT_REPS] = {};              // [dir][duty][rep] %/s
  float imp_travel[2][IDENT_IMP_WIDTHS][IDENT_REPS] = {};
  float imp_table[2][IDENT_IMP_WIDTHS] = {};          // median per width
  float sigma0_counts = 0.0f;
  float sigma_pwm_counts = 0.0f;
  float spike_rate = 0.0f;
  float stall_duty = 0.0f;
  float stall_motion_pct = 0.0f;
  float vm = -1.0f;                                   // stamped supply [V], <0 none
  float rail_duty_used = 0.0f;                        // I3 duty that reached the rails
  uint8_t clipped[2] = {0, 0};                        // D22: rail A/B ADC-clipped
  float clip_entry_raw[2] = {0.0f, 0.0f};             // last live reading before the clip
  uint32_t phase_ticks[IDENT_PHASES] = {};
  uint32_t total_ticks = 0;
};

struct IdentResult {
  bool valid = false;
  UnitCalib calib;
  IdentStamp stamp;
  IdentDetail detail;
};

uint16_t ident_endpoint_id(const UnitCalib &c);
// §5 stamp rule: a stored result is usable iff valid, fw major + instance
// match and the endpoint id matches the stored endpoints.
bool ident_stamp_valid(const IdentResult &r, uint16_t fw_major, uint8_t instance);

// Per-tick inputs (the Servo fills them; the SIL harness through the Servo).
struct IdentInputs {
  float raw = 0.0f;        // ADC counts (mean of the oversampled reads)
  float supply_v = -1.0f;  // <0 = no supply ADC
  bool adc_err = false;    // any oneshot read failed this tick
  float thermal_ema = 0.0f;
  bool thermal_tripped = false;
  bool l1_cut = false;     // L1 abort/thermal clamp active THIS tick (row 28)
};

inline constexpr int IDENT_BUF = 512;  // median/MAD scratch (int16 quarter counts)
inline constexpr int IDENT_HIST = 128; // per-tick median-position history

class Ident {
 public:
  void init(const IdentConfig *icfg, const ServoConfig *cfg,
            SeededBug bug = SeededBug::NONE);

  // Row 1/2. ok_to_start = controller IDLE && no fault && arbiter free.
  // prior = the calib the core currently runs on (m, defaults, previous stamp).
  bool start(bool ok_to_start, const UnitCalib &prior, const IdentStamp &prev_stamp,
             uint32_t date_counter);
  void abort();  // row 26 (no-op unless I1..I9)
  // One 1 kHz tick. Returns MOTOR duty (pre-L1).
  float tick(const IdentInputs &in);

  bool active() const {
    return state_ != IdentState::I0_OFF;
  }
  IdentState state() const { return state_; }
  IdentOutcome outcome() const { return outcome_; }
  IdentReason reason() const { return reason_; }
  float phase_progress() const;
  // true for exactly the tick on which the run ended (I9/FAIL/ABORT -> I0)
  bool just_finished() const { return just_finished_; }
  const IdentResult &committed() const { return committed_; }
  uint32_t commit_seq() const { return commit_seq_; }
  // last run's working copy (diagnostics only; NOT a valid result)
  const IdentResult &work() const { return work_; }
  void set_committed(const IdentResult &r) { committed_ = r; }
  // position estimate in % with the CURRENT scale (hint until I3, then rails)
  float pos_pct() const { return p_med_; }

 private:
  enum class Sub : uint8_t;

  void enter(IdentState s);
  void fail(IdentReason r);
  void do_abort(IdentReason r);
  float to_pct(float raw) const;
  float med8() const;
  float hist_ago(int ticks) const;  // p_med `ticks` ago (clamped to history)
  void buf_reset() { buf_n_ = 0; }
  void buf_push(float raw);
  float buf_median();               // destroys order
  float buf_mad_sigma(float med);   // MAD * 1.4826, destroys order
  float mean32() const;             // mean of last 32 raw -> %

  // pulse engine (motor duty); returns true while busy
  void pulse(float motor_duty, int width_ms, int settle_ms);
  bool pulse_busy() const { return pulse_left_ > 0 || settle_left_ > 0; }
  float pulse_out();

  // reposition by impulses to position target (row 14/22)
  void repo_begin(float target);
  bool repo_step(float *duty);  // true when done

  float tick_preflight(const IdentInputs &in);
  float tick_polarity();
  float tick_endpoints();
  float tick_quiet();
  float tick_bracket();
  float tick_stall();
  float tick_flights();
  float tick_impulses();
  void commit();
  void derive_r7();
  void finish_bracket();
  bool finish_flights();
  bool finish_impulses();

  const IdentConfig *icfg_{nullptr};
  const ServoConfig *cfg_{nullptr};
  SeededBug bug_{SeededBug::NONE};

  IdentState state_{IdentState::I0_OFF};
  IdentOutcome outcome_{IdentOutcome::NONE};
  IdentReason reason_{IdentReason::NONE};
  bool just_finished_{false};

  IdentResult work_;
  IdentResult committed_;
  uint32_t commit_seq_{0};
  IdentStamp prev_stamp_;
  uint32_t date_counter_{0};

  // scale
  float raw0_{8000.0f}, raw100_{100.0f};
  bool scale_measured_{false};
  int8_t dir_sign_{1};

  // signal conditioning
  float raw8_[8] = {};
  int raw8_n_{0}, raw8_head_{0};
  float raw32_[32] = {};
  int raw32_n_{0}, raw32_head_{0};
  float hist_[IDENT_HIST] = {};
  int hist_n_{0}, hist_head_{0};
  float p_med_{50.0f};
  float raw_med_{0.0f};
  float duty_ring_[32] = {};   // row 15 32er-Ring: (duty, pos)
  float pos_ring_[32] = {};
  int ring_head_{0}, ring_n_{0};
  float err_ema_{0.0f};
  int zero_ticks_{0};          // ticks since last nonzero output
  float last_out_{0.0f};

  int16_t buf_[IDENT_BUF] = {};
  int buf_n_{0};
  int dec_{0};

  // phase bookkeeping
  uint32_t phase_ticks_{0};
  uint32_t phase_limit_{0};
  uint32_t total_ticks_{0};
  uint8_t sub_{0};
  int timer_{0};

  // pulse engine
  int pulse_left_{0};
  int settle_left_{0};
  float pulse_duty_{0.0f};

  // reposition
  float repo_target_{50.0f};
  float repo_duty_{0.8f};
  float repo_gain_[2] = {0.3f, 0.3f};  // %/ms [fwd, rev]
  bool repo_prev_{false};
  float repo_p_before_{0.0f};
  int repo_w_prev_{0};
  int repo_dir_prev_{0};

  // I1
  float supply_sum_{0.0f};
  int supply_n_{0};
  // I2
  int probe_pass_{0}, probe_idx_{0};
  float probe_sign_{1.0f};
  float probe_ref_{0.0f};
  // I3
  float train_duty_{0.8f};
  float train_prev_{0.0f};
  int train_stall_{0};
  int train_count_{0};
  int train_dir_{-1};        // -1 = toward A (0 %), +1 = toward B
  float raw_a_{0.0f}, raw_b_{0.0f}, sig_a_{0.0f}, sig_b_{0.0f};
  bool leg_live_seen_{false};    // D22: this train saw unclipped samples
  bool leg_clip_seen_{false};
  float leg_last_live_raw_{0.0f};
  float leg_clip_entry_raw_{0.0f};
  bool is_clipped(float raw) const;
  float train_prog_sum_{0.0f};
  int train_prog_n_{0};
  // I5
  int bp_pos_{0}, bp_dir_{0}, bp_rep_{0};
  float hold_duty_{0.0f};
  float hold_ref_{0.0f};
  float low_moving_{0.0f};
  float fs_min_so_far_{2.0f};
  bool first_point_{true};
  // I6
  float stall_duty_{0.0f};
  float stall_ref_{0.0f};
  int spikes_{0};
  int stall_ticks_{0};
  // I7
  int fl_dir_{0}, fl_duty_{0}, fl_rep_{0};
  // I8
  int im_dir_{0}, im_rep_{0}, im_w_{0};
  float im_ref_{0.0f};
  float im_last_travel_{0.0f};
  int im_pulses_{0};
};

}  // namespace slider_servo
