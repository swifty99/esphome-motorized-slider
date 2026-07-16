#pragma once

// slider_lab — Phase-1 identification instrument for the motorslider project.
//
// Plays scripted open-loop torque profiles, records raw data deterministically
// at 1 kHz, and dumps CSV over HTTP. It is NOT the servo controller and must
// not grow into one (docs/lab-rig-spec.md).
//
// Clean-room implementation against the spec. House rules:
//   - NO function-static state anywhere; everything lives in instance members.
//   - Safety interlocks (R4) are enforced in the tick, never in the player.
//   - The recorder stores RAW data; all filtering happens offline (R2).

#ifdef USE_ESP_IDF

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/json/json_util.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_http_server.h>
#include <esp_timer.h>

namespace esphome {
namespace slider_lab {

enum class DecayMode : uint8_t { FAST = 0, SLOW = 1 };

// Segment types per R5. Note: the spec's dither(f, A, bias) segment is
// represented as HOLD(bias) with the dither_f/dither_a fields set — dither is
// superimposable on ANY segment (dither_grid needs breakaway + dither).
enum class SegType : uint8_t { HOLD, RAMP, PULSE, BREAKAWAY, SETTLE, GOTO };

struct Segment {
  SegType type{SegType::SETTLE};
  uint32_t duration_ms{1};  // total duration; max duration for BREAKAWAY/GOTO
  float a{0.0f};  // HOLD: duty | RAMP: d0 | PULSE: duty | BREAKAWAY: signed step | GOTO: target %
  float b{0.0f};  // RAMP: d1 | PULSE: width_ms | BREAKAWAY: interval_ms | GOTO: duty
  float c{0.0f};  // BREAKAWAY: move threshold % | GOTO: tolerance %
  float dither_f{0.0f};  // superimposed square-wave dither frequency, 0 = off
  float dither_a{0.0f};  // dither amplitude
  bool hold_last{false};  // HOLD: hold the final base torque of the previous segment
};

struct Program {
  std::string name;
  std::string params_json{"{}"};
  std::vector<Segment> segments;
  DecayMode mode{DecayMode::FAST};
  bool endstop_safe{false};  // R4: end-zone interlock bypass (endpoint work only)
  bool raw_samples{false};   // dump the 4 individual ADC samples (M1/M2)
  uint32_t budget_ms{30000}; // R4 wall-clock budget, default 30 s
};

// One record per 1 kHz tick. Field order keeps everything naturally aligned
// (24 bytes/record -> 75k records = 1.8 MB in the 2 MB PSRAM).
struct Record {
  uint32_t t_us;       // µs since experiment start (dumped as fractional t_ms)
  float adc_mean;      // raw mean of the 4 oneshot samples, unfiltered (R2)
  int16_t duty_milli;  // applied torque ×1000, after interlocks
  uint16_t seg;        // segment index
  uint16_t tick_us;    // tick execution time in µs (R1)
  uint16_t s[4];       // the 4 individual raw samples
  uint8_t flags;       // FLAG_* bitfield
};
static_assert(sizeof(Record) == 24, "Record layout must stay 24 bytes");

class SliderLab : public Component {
 public:
  // flags bitfield (CSV contract — analysis side is built against this)
  static constexpr uint8_t FLAG_SEG_START = 1;
  static constexpr uint8_t FLAG_ENDZONE = 2;
  static constexpr uint8_t FLAG_THERMAL = 4;
  static constexpr uint8_t FLAG_ABORT = 8;
  static constexpr uint8_t FLAG_GOTO = 16;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // codegen setters
  void set_adc_channel(uint8_t ch) { this->adc_channel_ = ch; }
  void set_pins(uint8_t ain1, uint8_t ain2, uint8_t sync) {
    this->ain1_pin_ = ain1;
    this->ain2_pin_ = ain2;
    this->sync_pin_ = sync;
  }
  void set_http_port(uint16_t port) { this->http_port_ = port; }
  void set_invert_direction(bool inv) { this->invert_direction_ = inv; }

  // ---- HA surface (called from YAML lambdas, main task) ----
  bool run_experiment(const std::string &name, const std::string &params_json);
  void set_abort(bool v) { this->abort_req_.store(v); }
  void set_raw_mode(bool v) { this->raw_mode_.store(v); }
  float get_position_percent() const { return this->pos_percent_.load(); }
  std::string get_player_state() const;
  bool is_recorder_full() const { return this->recorder_full_.load(); }
  uint16_t get_last_tick_us() const { return this->last_tick_us_.load(); }
  // readback of the actually driven scope-sync level (pin is INPUT_OUTPUT)
  bool get_sync_level() const { return gpio_get_level((gpio_num_t) this->sync_pin_) != 0; }

 protected:
  enum class State : uint8_t { IDLE = 0, RUNNING = 1, TAIL = 2 };

  // deferred-event bits (tick -> loop() logging; no logging in the tick path)
  static constexpr uint32_t EV_DONE = 1;
  static constexpr uint32_t EV_ABORT_SWITCH = 2;
  static constexpr uint32_t EV_BUDGET = 4;
  static constexpr uint32_t EV_THERMAL = 8;
  static constexpr uint32_t EV_ENDZONE = 16;
  static constexpr uint32_t EV_GOTO_TIMEOUT = 32;
  static constexpr uint32_t EV_FULL = 64;

  // ---- tick path (esp_timer task, 1 kHz) ----
  static void tick_trampoline_(void *arg);
  void tick_();
  float player_eval_(float pos, uint8_t &flags, bool &active_drive);
  void start_run_(Program *p, int64_t now_us);
  void finish_run_(bool aborted);
  int16_t apply_torque_(float torque, DecayMode mode, bool active_drive);
  void set_ledc_(uint8_t idx, uint32_t duty);
  float raw_to_percent_(float raw) const;
  float thermal_avg_() const;

  // ---- main task ----
  bool build_program_(const std::string &name, JsonObject p, Program &out);

  // ---- HTTP dump (httpd task) ----
  static esp_err_t dump_handler_(httpd_req_t *req);
  esp_err_t handle_dump_(httpd_req_t *req);
  bool player_busy_() const {
    return this->state_.load() != State::IDLE || this->pending_.load() != nullptr;
  }

  // config
  uint8_t adc_channel_{8};
  uint8_t ain1_pin_{11};
  uint8_t ain2_pin_{10};
  uint8_t sync_pin_{14};
  uint16_t http_port_{80};
  bool invert_direction_{false};

  // handles
  adc_oneshot_unit_handle_t adc_{nullptr};
  esp_timer_handle_t timer_{nullptr};
  httpd_handle_t httpd_{nullptr};

  // recorder (buffer written by tick only; read by httpd only while idle)
  Record *buf_{nullptr};
  uint32_t capacity_{0};
  bool buf_in_psram_{false};
  std::atomic<uint32_t> count_{0};
  std::atomic<bool> recorder_full_{false};
  bool recording_{false};  // tick-owned

  // player runtime state — tick-owned unless noted
  std::atomic<State> state_{State::IDLE};
  std::atomic<Program *> pending_{nullptr};   // main -> tick handoff
  std::atomic<Program *> finished_{nullptr};  // tick -> main reclaim
  Program *active_{nullptr};
  uint32_t seg_idx_{0};
  uint32_t seg_tick_{0};
  uint16_t rec_seg_{0};  // segment index snapshot for the current record
  std::atomic<uint32_t> seg_idx_pub_{0};
  int64_t run_t0_us_{0};
  uint32_t tail_ticks_{0};
  float bk_duty_{0.0f};
  float bk_ref_pos_{0.0f};
  float dither_phase_{0.0f};
  float seg_final_torque_{0.0f};  // latched at segment end (pre-dither), for hold_last
  uint32_t sync_low_ticks_{0};
  bool run_aborted_{false};
  bool run_raw_samples_{false};  // effective raw mode of the current/last run
  bool endzone_flagged_{false};  // one deferred log per run

  // R4 safety
  std::atomic<bool> abort_req_{false};
  std::atomic<bool> raw_mode_{false};
  // 10 s moving average of |duty_milli| in 100 ms buckets
  uint32_t th_bucket_[100]{};
  uint32_t th_total_{0};
  uint32_t th_idx_{0};
  uint32_t th_tick_in_bucket_{0};

  // HA-visible
  std::atomic<float> pos_percent_{0.0f};
  std::atomic<uint16_t> last_tick_us_{0};
  // written by run_experiment() only while idle; read by text sensor (main task)
  // and by the dump handler (only while idle)
  std::string run_name_{"none"};
  std::string run_params_{"{}"};

  std::atomic<bool> dump_active_{false};
  std::atomic<uint32_t> events_{0};

  // LEDC duty cache (skip redundant register writes in the tick)
  uint32_t ledc_last_[2]{0xFFFFFFFF, 0xFFFFFFFF};
};

}  // namespace slider_lab
}  // namespace esphome

#endif  // USE_ESP_IDF
