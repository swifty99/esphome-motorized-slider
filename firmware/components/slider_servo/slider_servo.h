#pragma once

// slider_servo — the ESPHome servo component (motorslider project).
//
// This is the WRAPPER (HAL + HA surface) around the pure-C++ control core in
// core/*. The core owns the §2 trigger graph, the EKF observer (L2), the
// endgame, CRAWL and L1 safety; it has zero IDF includes and is exercised by
// the SIL harness (sil/). This file feeds the core a position sample in %
// travel every 1 kHz tick and applies the returned duty to the L9110 — nothing
// more. Spec: docs/controller-spec.md, docs/handover-servo-2026-07-18.md.
//
// House rules inherited from slider_lab (bench-proven):
//   - deterministic 1 kHz tick in a dedicated task at prio 24 (above WiFi),
//     woken by a GPTimer ISR that does nothing but notify;
//   - NOTHING blocks in the tick: no logging, no alloc, no flash access;
//   - the recorder taps the pipeline read-only (control is recorder-blind);
//   - CSV rows are formatted with INTEGER math only — high-rate double
//     formatting crashes newlib _dtoa_r on this soft-float S2 (lab lesson).

#ifdef USE_ESP_IDF

#include <atomic>
#include <cstdint>
#include <string>

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <driver/ledc.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_attr.h>
#include <esp_http_server.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
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

// One record per 1 kHz tick. Scaled integers only, so the CSV dump never
// touches float formatting (soft-float _dtoa_r crash on the S2). 24 bytes ->
// 45000 records = ~1.08 MB, 45 s of trace in PSRAM.
struct Record {
  uint32_t t_ms;       // ms since recorder start (a jump = a missed tick, honest)
  int16_t z_c;         // measured position, centi-% (z * 100)
  int16_t p_c;         // observer p_hat, centi-%
  int16_t v_d;         // observer v_hat, deci-%/s (v_hat * 10)
  int16_t u_cmd_m;     // controller output pre-L1, milli-duty (* 1000)
  int16_t u_app_m;     // applied duty after L1, milli-duty
  int16_t vdes_d;      // trajectory v_des, deci-%/s
  int16_t ff_m;        // feedforward term, milli-duty
  uint16_t tick_us;    // tick execution time [µs]
  uint8_t state;       // core::State
  uint8_t settle;      // core::SettleFlag
  uint8_t release;     // core::ReleaseReason
  uint8_t clamped;     // L1 clamped this tick (0/1)
};
static_assert(sizeof(Record) == 24, "Record layout must stay 24 bytes");

// NVS blob (auto-ident writes this later; bring-up runs on the nominal defaults
// compiled into UnitCalib). Versioned so a layout change ignores stale data.
struct StoredCalib {
  uint32_t magic;
  uint32_t version;
  core::UnitCalib calib;
};

class SliderServo : public Component {
 public:
  static constexpr uint32_t kCalibMagic = 0x53565243;  // 'SVRC'
  static constexpr uint32_t kCalibVersion = 1;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // ---- codegen setters ----
  void set_adc_channel(uint8_t ch) { this->adc_channel_ = ch; }
  void set_pins(uint8_t ain1, uint8_t ain2, uint8_t sync) {
    this->ain1_pin_ = ain1;
    this->ain2_pin_ = ain2;
    this->sync_pin_ = sync;
  }
  void set_http_port(uint16_t port) { this->http_port_ = port; }
  void set_invert_direction(bool inv) { this->invert_direction_ = inv; }
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
  void set_thermal_avg_max(float v) { this->cfg_.thermal_avg_max = v; }
  void set_u_max(float v) { this->cfg_.u_max = v; }

  // ---- HA surface (called from YAML lambdas, main task) ----
  // A move command: latched into atomics, consumed by the tick task (the core
  // controller is single-threaded and must only be driven from the tick).
  void command_target(float target_pct) {
    this->cmd_target_.store(target_pct);
    this->cmd_seq_.fetch_add(1);
  }
  void command_speed(float v_req) { this->cmd_speed_.store(v_req); }
  void command_release() { this->release_req_.store(true); }
  void set_abort(bool v) { this->abort_req_.store(v); }

  float get_position_percent() const { return this->pos_pct_.load(); }
  float get_p_hat() const { return this->p_hat_.load(); }
  float get_v_hat() const { return this->v_hat_.load(); }
  float get_target() const { return this->target_echo_.load(); }
  float get_thermal_avg() const { return this->thermal_avg_.load(); }
  int get_pulses() const { return this->pulses_.load(); }
  int get_kicks() const { return this->kicks_.load(); }
  int get_overshoots() const { return this->overshoots_.load(); }
  uint32_t get_adc_error_count() const { return this->adc_err_count_.load(); }
  uint16_t get_last_tick_us() const { return this->last_tick_us_.load(); }
  uint32_t get_missed_ticks() const { return this->missed_ticks_.load(); }
  uint16_t get_tick_p99_us() const;
  bool is_settled() const { return this->settled_.load(); }
  bool is_recorder_full() const { return this->recorder_full_.load(); }
  bool get_sync_level() const { return gpio_get_level((gpio_num_t) this->sync_pin_) != 0; }
  std::string get_state_name() const;
  std::string get_settle_name() const;
  std::string get_release_name() const;
  // reset reason latched at boot (esp_reset_reason) — surfaces WHY the last
  // reboot happened (poweron / SW / panic / task-wdt / brownout ...).
  std::string get_reset_reason() const { return this->reset_reason_; }

  // auto-ident (Block 6) writes the identified calib here; inert for now.
  void save_calib();

 protected:
  // ---- tick path (dedicated task, prio 24, woken by GPTimer ISR at 1 kHz) ----
  static bool timer_isr_(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg);
  static void tick_task_trampoline_(void *arg);
  void tick_();
  float raw_to_percent_(float raw) const { return (this->raw0_ - raw) / (this->raw0_ - this->raw100_) * 100.0f; }
  void apply_duty_(float duty);
  void set_ledc_(uint8_t idx, uint32_t duty);
  void snapshot_(float z, float duty, uint16_t tick_us);

  // ---- HTTP dump (httpd task) ----
  static esp_err_t dump_handler_(httpd_req_t *req);
  esp_err_t handle_dump_(httpd_req_t *req);

  // config
  uint8_t adc_channel_{8};
  uint8_t ain1_pin_{11};
  uint8_t ain2_pin_{10};
  uint8_t sync_pin_{14};
  uint16_t http_port_{80};
  bool invert_direction_{false};
  float raw0_{8000.0f};
  float raw100_{100.0f};
  uint32_t req_capacity_{45000};

  // core control (calib_/cfg_ outlive servo_; pointers handed to init())
  core::UnitCalib calib_{};
  core::ServoConfig cfg_{};
  core::Servo servo_;
  ESPPreferenceObject pref_;

  // handles
  adc_oneshot_unit_handle_t adc_{nullptr};
  gptimer_handle_t gptimer_{nullptr};
  TaskHandle_t tick_task_{nullptr};
  httpd_handle_t httpd_{nullptr};

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
  uint32_t last_seq_{0};
  std::atomic<bool> release_req_{false};
  std::atomic<bool> abort_req_{false};

  // HA-visible snapshot (tick writes, sensors read)
  std::atomic<float> pos_pct_{0.0f};
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
  std::atomic<bool> settled_{false};
  std::atomic<uint32_t> adc_err_count_{0};
  std::atomic<uint16_t> last_tick_us_{0};
  std::atomic<uint32_t> missed_ticks_{0};

  // tick-time histogram for the p99 diagnostic (25 µs buckets, 0..2000 µs).
  // Written by the tick, read by the sensor getter — a diagnostic, torn reads
  // are harmless.
  static constexpr int kHistBuckets = 81;  // 0..2000 in 25 µs steps + overflow
  static constexpr int kHistStepUs = 25;
  uint32_t tick_hist_[kHistBuckets]{};

  // LEDC duty cache (skip redundant register writes in the tick)
  uint32_t ledc_last_[2]{0xFFFFFFFF, 0xFFFFFFFF};
  bool ready_{false};  // setup completed; tick may drive
  const char *reset_reason_{"?"};  // latched from esp_reset_reason() in setup()
};

}  // namespace slider_servo_hal
}  // namespace esphome

#endif  // USE_ESP_IDF
