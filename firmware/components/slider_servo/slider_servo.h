#pragma once

// slider_servo — the ESPHome servo component (motorslider project).
//
// This is the WRAPPER (HAL + HA surface) around the pure-C++ control core
// (servo.h & co.). The core owns the §2 trigger graph, the EKF observer (L2),
// the endgame, CRAWL and L1 safety; it has zero IDF includes and is exercised
// by the SIL harness (sil/). The wrapper feeds the core a position sample in
// % travel every 1 kHz tick and applies the returned duty to the H-bridge.
// Spec: docs/controller-spec.md, docs/kueche-mief-spec.md (R1–R5, R8).
//
// Stage 0B (kueche-mief): N instances (YAML list, MULTI_CONF) share ONE hub:
//   - the hub owns ADC1 (one adc_oneshot unit for every instance and every
//     extra ADC1 channel — R2), the GPTimer, the prio-24 tick task, the single
//     httpd and the motor ARBITER (R1: at most one instance with duty != 0);
//   - each instance owns its core, its two LEDC channels, its NVS slot
//     (key 'SVRC' + index), its recorder ring and /servo/<id>/dump.csv.
//
// House rules inherited from slider_lab (bench-proven):
//   - deterministic 1 kHz tick in a dedicated task at prio 24 (above WiFi),
//     woken by a GPTimer ISR that does nothing but notify;
//   - NOTHING blocks in the tick: no logging, no alloc, no flash access, no
//     mutex — cross-task data is atomics only (see the locking model below);
//   - the recorder taps the pipeline read-only (control is recorder-blind);
//   - CSV rows are formatted with INTEGER math only — high-rate double
//     formatting crashes newlib _dtoa_r on this soft-float S2 (lab lesson).

#ifdef USE_ESP_IDF

#include <atomic>
#include <cstdint>
#include <string>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/preferences.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <driver/ledc.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_attr.h>
#include <esp_http_server.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// The pure-C++ control core lives flat in this component dir (ESPHome only
// bundles a component's top-level sources — a core/ subdir would not compile).
#include "servo.h"

namespace esphome {
namespace slider_servo_hal {

// The pure-C++ control core lives in the global ::slider_servo namespace; alias
// it as `core`. The wrapper namespace is deliberately slider_servo_hal, not
// slider_servo, so it cannot collide with ::slider_servo under main.cpp's
// `using namespace esphome;`.
namespace core = ::slider_servo;

class SliderServoHub;

// One record per full-path tick. Scaled integers only, so the CSV dump never
// touches float formatting (soft-float _dtoa_r crash on the S2). 24 bytes ->
// 45000 records = ~1.08 MB, 45 s of trace in PSRAM.
struct Record {
  uint32_t t_ms;       // ms since hub start (a jump = a missed / cheap-path tick, honest)
  int16_t z_c;         // measured position, centi-% (z * 100)
  int16_t p_c;         // observer p_hat, centi-%
  int16_t v_d;         // observer v_hat, deci-%/s (v_hat * 10)
  int16_t u_cmd_m;     // controller output pre-L1, milli-duty (* 1000)
  int16_t u_app_m;     // applied duty after L1 + arbiter, milli-duty
  int16_t vdes_d;      // trajectory v_des, deci-%/s
  int16_t ff_m;        // feedforward term, milli-duty
  uint16_t tick_us;    // this instance's tick execution time [µs]
  uint8_t state;       // core::State
  uint8_t settle;      // core::SettleFlag
  uint8_t release;     // core::ReleaseReason
  uint8_t clamped;     // L1 clamped this tick (0/1)
};
static_assert(sizeof(Record) == 24, "Record layout must stay 24 bytes");

// NVS blob: the committed IdentResult (calib + §5 stamp + detail). Key stays
// 'SVRC' + index; magic/version are new, so an old StoredCalib blob (other
// size, other magic) fails to load and the unit boots "nicht identifiziert".
struct StoredIdent {
  uint32_t magic;
  uint32_t version;
  core::IdentResult result;
};

// What the status text shows besides the controller state (main-task events).
enum class StatusEvent : uint8_t {
  NONE = 0,
  BOOT_REJECT = 1,        // command before setup completed (R5) — dropped, said so
  QUEUE_FULL = 2,         // arbiter request ring full — command refused, said so
  IDENT_UNAVAILABLE = 3,  // (unused since the core merge; kept for the enum values)
  ABORTED = 4,            // Abbruch pressed
  IDENT_RUNNING = 5,      // set_target refused by the core: ident owns the motor
  IDENT_REJECTED = 6,     // start_ident refused (row 2: not IDLE / fault / busy)
  IDENT_FINISHED = 7,     // ident ended (OK / FAIL / ABORT) — shown until the next command
};

// ============================================================================
// SliderServo — one slider (one core, one H-bridge, one wiper channel).
// ============================================================================
class SliderServo : public Component {
  friend class SliderServoHub;

 public:
  static constexpr uint32_t kCalibMagic = 0x53565243;  // 'SVRC'; NVS key = magic + index
  static constexpr uint32_t kIdentMagic = 0x53564931;  // 'SVI1' blob magic (IdentResult)
  static constexpr uint32_t kIdentVersion = 2;

  void setup() override;
  void loop() override;  // main task: NVS commit of a new ident, supply voltage
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // ---- codegen setters ----
  void set_hub(SliderServoHub *hub) { this->hub_ = hub; }
  void set_index(uint8_t idx) { this->nvs_index_ = idx; }
  void set_name_id(const char *name) { this->name_id_ = name; }
  void set_adc_channel(uint8_t ch) { this->adc_channel_ = ch; }
  void set_pins(uint8_t ain1, uint8_t ain2) {
    this->ain1_pin_ = ain1;
    this->ain2_pin_ = ain2;
  }
  void set_sync_pin(uint8_t pin) { this->sync_pin_ = pin; }
  void set_idle_full(bool full) { this->idle_full_ = full; }
  void set_idle_sample_divider(uint8_t div) { this->idle_div_ = div > 0 ? div : 1; }
  // R6: only the I2 start value (IdentConfig::dir_sign_hint) and the sign of
  // the NOMINAL calib until an ident has learned dir_sign.
  void set_invert_direction(bool inv) { this->invert_direction_ = inv; }
  // bench/rig: treat the nominal (rig-identified) map as identified -> no R8
  // throttling without an NVS ident. Default false (kueche-mief-spec R8).
  void set_assume_identified(bool v) { this->assume_identified_ = v; }
  void set_fw_major(uint16_t v) { this->icfg_.fw_major = v; }
#ifdef USE_SENSOR
  // optional supply voltage for ident preflight/row 30 (any sensor, volts)
  void set_supply_sensor(sensor::Sensor *s) { this->supply_sensor_ = s; }
#endif
  void set_raw_endpoints(float raw0, float raw100) {
    this->raw0_ = raw0;
    this->raw100_ = raw100;
  }
  void set_recorder_capacity(uint32_t n) { this->req_capacity_ = n; }
  // §5 tunables exposed to YAML (recompile-to-retune during HIL bring-up; the
  // core stays untouched). Defaults come from ServoConfig, applied in setup().
  void set_tol(float v) { this->cfg_.tol = v; }
  void set_v_cruise(float v) { this->cfg_.v_cruise = v; }
  void set_v_min_cmd(float v) { this->cfg_.v_min_cmd = v; }
  void set_impulse_settle_ms(int v) { this->cfg_.impulse_settle_ms = v; }
  void set_impulse_settle_short_ms(int v) { this->cfg_.impulse_settle_short_ms = v; }
  void set_impulse_aim(float v) { this->cfg_.impulse_aim = v; }
  void set_impulse_max(int v) { this->cfg_.impulse_max = v; }
  void set_crawl_impulse_duty(float v) { this->cfg_.crawl_impulse_duty = v; }
  void set_crawl_max_width_ms(float v) { this->cfg_.crawl_max_width_ms = v; }
  void set_crawl_step(float v) { this->cfg_.crawl_step = v; }
  void set_crawl_w_gain(float v) { this->cfg_.crawl_w_gain = v; }
  void set_crawl_settle_ms(int v) { this->cfg_.crawl_settle_ms = v; }
  void set_crawl_land_zone(float v) { this->cfg_.crawl_land_zone = v; }
  // GLIDE (glide-mode-spec.md): continuous keep-rolling slow mode. glide>0 routes
  // a slow request to State::GLIDE instead of CRAWL. Default off = CRAWL.
  void set_glide(float v) { this->cfg_.glide = v; }
  void set_glide_floor(float v) { this->cfg_.glide_floor = v; }
  void set_glide_step(float v) { this->cfg_.glide_step = v; }
  void set_glide_settle_ms(int v) { this->cfg_.glide_settle_ms = v; }
  void set_v_roll(float v) { this->cfg_.v_roll = v; }
  void set_glide_prog_win(int v) { this->cfg_.glide_prog_win = v; }
  void set_glide_runaway_v(float v) { this->cfg_.glide_runaway_v = v; }
  void set_glide_max_gap_ms(int v) { this->cfg_.glide_max_gap_ms = v; }
  void set_coast_zone(float v) { this->cfg_.coast_zone = v; }
  // L1 thermal (EMA over |applied duty| with trip/resume hysteresis). Only
  // effective at codegen time: Safety::init() derives alpha from tau.
  void set_thermal_tau_s(float v) { this->cfg_.thermal_tau_s = v; }
  void set_thermal_avg_max(float v) { this->cfg_.thermal_avg_max = v; }
  void set_thermal_resume(float v) { this->cfg_.thermal_resume = v; }
  void set_u_max(float v) { this->cfg_.u_max = v; }

  // ---- HA surface (called from YAML lambdas, main task) ----
  // A move command: latched into atomics + one arbiter request; the tick task
  // delivers it when this instance holds the motor grant (R1). Before setup
  // has completed every command is refused (R5: boot never moves).
  bool command_target(float target_pct);
  void command_speed(float v_req) { this->cmd_speed_.store(v_req); }
  // Release (row 23): hand the slider back; a later Target still runs (rig semantics).
  void command_release() { this->release_req_.store(true); }
  // L1 abort switch (rig): torque 0 within one tick while on.
  void set_abort(bool v) { this->abort_req_.store(v); }

  // R4 Ident / Abbruch buttons. Ident is queued through the arbiter like a
  // move (it needs the motor exclusively) and keeps the grant until it ends
  // (OK / FAIL / ABORT). Abbruch = torque 0 now: cancels a queued request and
  // any command latched so far, aborts an ident (row 26), releases the core.
  bool start_ident();
  void abort_ident();

  // ---- getters (main task; atomics written by the tick) ----
  float get_position_percent() const { return this->pos_pct_.load(); }
  int get_wiper_raw() const { return this->raw_pub_.load(); }
  float get_p_hat() const { return this->p_hat_.load(); }
  float get_v_hat() const { return this->v_hat_.load(); }
  float get_target() const { return this->target_echo_.load(); }
  float get_thermal_avg() const { return this->thermal_avg_.load(); }
  int get_pulses() const { return this->pulses_.load(); }
  int get_kicks() const { return this->kicks_.load(); }
  int get_overshoots() const { return this->overshoots_.load(); }
  uint32_t get_adc_error_count() const { return this->adc_err_count_.load(); }
  // hub-wide tick health (one shared tick serves all instances)
  uint16_t get_last_tick_us() const;
  uint32_t get_missed_ticks() const;
  uint16_t get_tick_p99_us() const;
  uint32_t get_adc_error_total() const;   // all wipers + extra channels (hub)
  uint32_t get_adc_contention() const;    // ticks that held a sample (shared ADC busy)
  uint32_t get_queue_overflows() const;   // arbiter requests refused (ring full)
  // per-instance path diagnostics (K10 evidence, measured on hardware only)
  uint16_t get_idle_path_p99_us() const;
  uint16_t get_idle_path_max_us() const { return this->cheap_max_us_.load(); }
  uint16_t get_full_path_max_us() const { return this->full_max_us_.load(); }
  uint32_t get_idle_path_ticks() const { return this->cheap_ticks_.load(); }
  uint32_t get_full_path_ticks() const { return this->full_ticks_.load(); }
  uint32_t get_arbiter_blocked() const { return this->arbiter_blocked_.load(); }
  bool is_motor_owner() const { return this->owner_pub_.load(); }
  bool is_waiting() const { return this->waiting_.load(); }
  bool is_settled() const { return this->settled_.load(); }
  bool is_recorder_full() const { return this->recorder_full_.load(); }
  // the core runs on an identified calib (NVS blob adopted with a valid §5
  // stamp, or an ident committed this boot) — "Ident gültig".
  bool is_calib_valid() const { return this->identified_pub_.load(); }
  bool is_ident_active() const { return this->ident_running_.load(); }
  // R4 "Ident Bericht": values of the identified calib or the FAIL/ABORT reason.
  std::string get_ident_report() const;
  bool get_sync_level() const {
    return this->sync_pin_ != kNoPin && gpio_get_level((gpio_num_t) this->sync_pin_) != 0;
  }
  std::string get_state_name() const;
  std::string get_settle_name() const;
  std::string get_release_name() const;
  // R4 status text: idle / moving / crawl / released(<reason>) / ident(<phase>)
  // / wartet / ident nicht verfügbar / ..., suffixed ", nicht identifiziert"
  // while no valid NVS calib exists (R8, K8).
  std::string get_status_text() const;
  // reset reason latched at boot (esp_reset_reason) — surfaces WHY the last
  // reboot happened (poweron / SW / panic / task-wdt / brownout ...).
  std::string get_reset_reason() const { return this->reset_reason_; }
  const char *get_name_id() const { return this->name_id_; }
  uint8_t get_index() const { return this->nvs_index_; }

 protected:
  static constexpr uint8_t kNoPin = 0xFF;

  // ---- called by the hub ----
  bool hw_setup_(adc_oneshot_unit_handle_t adc, uint8_t slot, ledc_channel_t ch_a, ledc_channel_t ch_b,
                 int64_t t0_us);
  // One tick for this instance. owner: holds the motor grant; adc_free: the
  // shared-ADC flag was clear at tick entry. Returns the duty applied.
  float tick_(bool owner, bool adc_free);
  // arbiter hooks (tick task only)
  bool owner_done_() const;     // may hand the grant back (IDLE, nothing pending)
  void deliver_();              // grant -> execute the latched command
  void handle_release_();       // consume release/abort requests

  void sample_(bool adc_free);
  void publish_ident_();
  bool read_result_(core::IdentResult *out) const;  // seqlock copy of pub_result_
  void apply_duty_(float duty);
  void set_ledc_(uint8_t idx, uint32_t duty);
  void write_sync_(bool driving);
  void snapshot_(float z, float duty, uint16_t tick_us);

  // re-seed a core frozen on the cheap path (keeps ident result + thermal EMA)
  void core_resync_(float raw);

  // ---- HTTP dump (httpd task) ----
  static esp_err_t dump_handler_(httpd_req_t *req);
  esp_err_t handle_dump_(httpd_req_t *req);

  // config
  SliderServoHub *hub_{nullptr};
  uint8_t slot_{0};          // registration order (hub arrays, LEDC pair)
  uint8_t nvs_index_{0};     // YAML `index` (NVS key 'SVRC' + index)
  const char *name_id_{"servo"};
  uint8_t adc_channel_{8};
  uint8_t ain1_pin_{11};
  uint8_t ain2_pin_{10};
  uint8_t sync_pin_{kNoPin};
  bool idle_full_{true};     // false = IDLE non-owner takes the cheap path
  uint8_t idle_div_{1};      // cheap path samples every idle_div_ ticks (>=100 Hz)
  bool invert_direction_{false};
  bool assume_identified_{false};
  float raw0_{8000.0f};
  float raw100_{100.0f};
  uint32_t req_capacity_{45000};

  // core control (calib_/cfg_ outlive servo_; pointers handed to init())
  core::UnitCalib calib_{};
  core::ServoConfig cfg_{};
  core::IdentConfig icfg_{};
  core::Servo servo_;
  ESPPreferenceObject pref_;
#ifdef USE_SENSOR
  sensor::Sensor *supply_sensor_{nullptr};
#endif
  std::atomic<float> supply_v_{-1.0f};  // main -> tick; < 0 = no supply ADC

  // Ident result publication. The core resets its committed result and
  // commit_seq on every init() (resync), so the wrapper keeps its own copy:
  // pub_result_ is written by the TICK only (boot: before the tick starts),
  // guarded by the seqlock result_gen_ (odd = write in progress); readers
  // (main task NVS save, report, httpd header) copy and retry on change.
  core::IdentResult *pub_result_{nullptr};  // heap (PSRAM preferred), allocated in setup
  std::atomic<uint32_t> result_gen_{0};
  std::atomic<uint32_t> pub_seq_{0};   // wrapper commit counter (monotonic)
  uint32_t saved_seq_{0};              // main task: last seq persisted to NVS
  uint32_t last_core_seq_{0};          // tick: core commit_seq at last check
  StoredIdent *io_buf_{nullptr};       // main task scratch (NVS load/save, report), heap
  std::atomic<uint32_t> ident_date_{0};  // main -> tick: date counter for the stamp

  // handles
  adc_oneshot_unit_handle_t adc_{nullptr};
  ledc_channel_t ledc_ch_[2]{LEDC_CHANNEL_0, LEDC_CHANNEL_1};
  uint32_t sync_lo_{0};  // GPIO_OUT_REG mask (pins 0..31); 0 = no sync pin
  uint32_t sync_hi_{0};  // GPIO_OUT1_REG mask (pins 32..); 0 = no sync pin
  std::string dump_uri_;  // "/servo/<id>/dump.csv" (built in setup, never in the tick)

  // tick-task-only state (no other task touches these)
  float z_{50.0f};
  float raw_f_{0.0f};       // mean of the last sampled raw counts (tick_raw input)
  bool adc_err_tick_{false};
  int raw_mean_{0};
  uint8_t idle_countdown_{0};
  bool core_stale_{false};    // cheap path ran -> core observer is behind
  uint32_t last_seq_{0};

  // recorder (free-running ring; written by tick, read by httpd while paused)
  Record *buf_{nullptr};
  uint32_t capacity_{0};
  bool buf_in_psram_{false};
  std::atomic<uint32_t> rec_head_{0};
  std::atomic<uint32_t> rec_count_{0};
  std::atomic<bool> recorder_full_{false};
  std::atomic<bool> dump_active_{false};
  int64_t rec_t0_us_{0};

  // command handoff (main task -> tick task)
  std::atomic<float> cmd_target_{0.0f};
  std::atomic<float> cmd_speed_{0.0f};
  std::atomic<uint32_t> cmd_seq_{0};
  std::atomic<uint32_t> discard_seq_{0};  // Abbruch: commands up to this seq are void
  std::atomic<bool> ident_req_{false};
  std::atomic<bool> release_req_{false};
  std::atomic<bool> cancel_req_{false};   // Abbruch (release + cancel + ident abort)
  std::atomic<bool> abort_req_{false};

  // HA-visible snapshot (tick writes, sensors read)
  std::atomic<float> pos_pct_{0.0f};
  std::atomic<int> raw_pub_{0};
  std::atomic<float> p_hat_{0.0f};
  std::atomic<float> v_hat_{0.0f};
  std::atomic<float> target_echo_{0.0f};
  std::atomic<float> thermal_avg_{0.0f};
  std::atomic<int> pulses_{0};
  std::atomic<int> kicks_{0};
  std::atomic<int> overshoots_{0};
  std::atomic<uint8_t> state_pub_{0};
  std::atomic<uint8_t> settle_pub_{0};
  std::atomic<uint8_t> release_pub_{0};
  std::atomic<uint8_t> event_{0};         // StatusEvent
  std::atomic<bool> settled_{false};
  std::atomic<bool> owner_pub_{false};
  std::atomic<bool> identified_pub_{false};
  std::atomic<bool> unid_flag_pub_{true};
  std::atomic<bool> ident_running_{false};
  std::atomic<uint8_t> ident_state_pub_{0};
  std::atomic<uint8_t> ident_outcome_pub_{0};
  std::atomic<uint8_t> ident_reason_pub_{0};
  std::atomic<float> ident_progress_pub_{0.0f};
  std::atomic<bool> waiting_{false};
  std::atomic<uint32_t> adc_err_count_{0};
  std::atomic<uint32_t> arbiter_blocked_{0};  // non-owner core asked for duty -> forced 0

  // per-path timing (K10 evidence). Cheap path: 10 µs buckets 0..400 µs.
  static constexpr int kCheapHistBuckets = 41;
  static constexpr int kCheapHistStepUs = 10;
  uint32_t cheap_hist_[kCheapHistBuckets]{};
  std::atomic<uint16_t> cheap_max_us_{0};
  std::atomic<uint16_t> full_max_us_{0};
  std::atomic<uint32_t> cheap_ticks_{0};
  std::atomic<uint32_t> full_ticks_{0};

  // LEDC duty cache (skip redundant register writes in the tick)
  uint32_t ledc_last_[2]{0xFFFFFFFF, 0xFFFFFFFF};
  const char *reset_reason_{"?"};  // latched from esp_reset_reason() in setup()
};

// ============================================================================
// SliderServoHub — shared resources of all instances (created by codegen,
// not a Component: every instance's setup() calls setup_once()).
//
// LOCKING MODEL (R1/R2; S2 is single-core, tick task prio 24 > everything
// that touches these structures):
//   * Tick never waits. It holds no mutex and never calls anything that can
//     block; everything it shares is a std::atomic or a lock-free SPSC ring.
//   * Commands: main-task producers serialise among THEMSELVES with
//     producer_mtx_ (only producers take it), then push an instance slot into
//     the SPSC ring req_q_. The tick is the only consumer and drains it
//     without locking. Ring full -> command refused with an honest status.
//   * Shared ADC1: wipers are read by the tick; extra channels by the main
//     task via try_read_shared(), which (a) only starts a conversion if the
//     next tick is >= kSharedReadGuardUs away and (b) brackets the read with
//     the atomic adc_busy_. If the tick still preempts inside that bracket it
//     sees adc_busy_ at entry and SKIPS its wiper reads (sample-and-hold of the
//     last value, counted in adc_contention_) instead of waiting — IDF's
//     adc_oneshot_read uses a try-lock and would fail anyway. The extra read
//     then simply retries on the next main-loop pass.
// ============================================================================
class SliderServoHub {
 public:
  static constexpr int kMaxInstances = 4;
  static constexpr int kMaxExtraChannels = 8;

  // ---- codegen ----
  void register_servo(SliderServo *s);
  // returns the slot the sensor uses with try_read_shared()
  uint8_t add_extra_channel(uint8_t channel, uint8_t atten);
  void set_http_port(uint16_t port) { this->http_port_ = port; }

  // ---- setup (first instance setup() does everything, later calls no-op) ----
  bool setup_once();
  bool is_started() const { return this->started_; }
  void dump_config();

  // ---- main task ----
  // Shared ADC1 read for an extra channel slot (see locking model). Returns
  // false if this is not a safe window or the read failed; caller retries.
  bool try_read_shared(uint8_t slot, int *raw);
  // raw counts -> volts with the IDF line/curve-fitting calibration of that
  // attenuation (linear fallback if the eFuse has no calibration). Main task.
  float raw_to_volts(uint8_t atten, int raw) const;
  bool enqueue_request(uint8_t slot);  // producer side, under producer_mtx_
  int wait_position(uint8_t slot) const { return this->wait_pos_[slot].load(); }

  // diagnostics (main task)
  uint16_t get_tick_p99_us() const;
  uint16_t get_last_tick_us() const { return this->last_tick_us_.load(); }
  uint32_t get_missed_ticks() const { return this->missed_ticks_.load(); }
  uint32_t get_adc_contention() const { return this->adc_contention_.load(); }
  uint32_t get_queue_overflows() const { return this->queue_overflows_.load(); }
  uint32_t get_shared_read_errors() const { return this->shared_read_errors_.load(); }
  uint32_t get_adc_error_total() const;
  int get_owner_slot() const { return this->owner_pub_.load(); }

 protected:
  static bool timer_isr_(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg);
  static void tick_task_trampoline_(void *arg);
  void tick_();
  void arbiter_step_();
  bool fifo_contains_(int8_t slot) const;
  void fifo_remove_(int8_t slot);
  void publish_fifo_();
  bool setup_ledc_timer_();
  void setup_calibration_();

  // instances
  SliderServo *servos_[kMaxInstances]{};
  uint8_t n_servos_{0};

  // ADC1 (one unit for everything — R2)
  adc_oneshot_unit_handle_t adc_{nullptr};
  struct Extra {
    uint8_t channel;
    uint8_t atten;
  };
  Extra extras_[kMaxExtraChannels]{};
  uint8_t n_extras_{0};
  static constexpr int kAttenLevels = 4;  // adc_atten_t 0, 2.5, 6, 12 dB
  adc_cali_handle_t cali_[kAttenLevels]{};
  std::atomic<bool> adc_busy_{false};
  std::atomic<uint32_t> tick_start_us_{0};
  std::atomic<uint32_t> adc_contention_{0};
  std::atomic<uint32_t> shared_read_errors_{0};

  // tick
  gptimer_handle_t gptimer_{nullptr};
  TaskHandle_t tick_task_{nullptr};
  std::atomic<uint16_t> last_tick_us_{0};
  std::atomic<uint32_t> missed_ticks_{0};
  // whole-tick histogram for the A9 p99 diagnostic (25 µs buckets, 0..2000 µs).
  // Written by the tick, read by the sensor getter — a diagnostic, torn reads
  // are harmless.
  static constexpr int kHistBuckets = 81;  // 0..2000 in 25 µs steps + overflow
  static constexpr int kHistStepUs = 25;
  uint32_t tick_hist_[kHistBuckets]{};

  // arbiter: SPSC request ring (main -> tick) + tick-only FIFO of waiters
  static constexpr uint8_t kReqQueueCap = 8;  // power of two; >= 2 * kMaxInstances
  static_assert((kReqQueueCap & (kReqQueueCap - 1)) == 0, "ring capacity must be a power of two");
  uint8_t req_q_[kReqQueueCap]{};
  std::atomic<uint8_t> req_head_{0};  // producer-owned
  std::atomic<uint8_t> req_tail_{0};  // consumer-owned (tick)
  SemaphoreHandle_t producer_mtx_{nullptr};
  std::atomic<uint32_t> queue_overflows_{0};
  int8_t owner_{-1};                  // tick-only: slot holding the motor grant
  int8_t fifo_[kMaxInstances]{};      // tick-only: waiting slots, arrival order
  uint8_t fifo_n_{0};
  std::atomic<int> owner_pub_{-1};
  std::atomic<int> wait_pos_[kMaxInstances]{};  // 0 = not waiting, k = k-th in line

  // http (one httpd for all instances)
  uint16_t http_port_{80};
  httpd_handle_t httpd_{nullptr};

  int64_t t0_us_{0};
  bool setup_done_{false};
  bool started_{false};
};

}  // namespace slider_servo_hal
}  // namespace esphome

#endif  // USE_ESP_IDF
