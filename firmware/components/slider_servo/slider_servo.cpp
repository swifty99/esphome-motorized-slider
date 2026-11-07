#ifdef USE_ESP_IDF

#include "slider_servo.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"

#include <esp_adc/adc_cali_scheme.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <lwip/sockets.h>
#include <soc/gpio_reg.h>
#include <soc/soc_caps.h>

namespace esphome {
namespace slider_servo_hal {

static const char *const TAG = "slider_servo";

namespace {

// ---- hardware constants (identical drive stack to slider_lab, bench-proven) --
constexpr uint32_t kTickUs = 1000;             // 1 kHz loop (spec §1)
constexpr int kAdcSamples = 4;                 // 4 oneshot reads per sampled tick
constexpr int kSeedSamples = 8;                // boot seed of the observer
constexpr uint32_t kPwmFreqHz = 19531;
constexpr ledc_timer_bit_t kPwmRes = LEDC_TIMER_12_BIT;
constexpr uint32_t kPwmFull = 1u << 12;
// One LEDC timer for all servo channels, taken from the TOP of the LEDC range
// (timer 3, channels 7/6, 5/4, ...). ESPHome's own ledc outputs allocate from
// channel 0 upward with timer = channel/2, so they never meet ours as long as
// they use < 8 - 2N channels (enforced in __init__.py FINAL_VALIDATE).
constexpr ledc_timer_t kLedcTimer = static_cast<ledc_timer_t>(SOC_LEDC_TIMER_NUM - 1);
constexpr int kLedcTopChannel = SOC_LEDC_CHANNEL_NUM - 1;
// R6 dump-session guards (same latch fix as the lab dump handler)
constexpr int kDumpSendTimeoutS = 3;
// 60 s (up from 30): under WiFi+tick contention a 12 s ring can stream slowly
// (~15 KB/s worst case ≈ 40 s). 60 s lets it finish rather than truncate the
// trailing idle. The servo dump pauses only LOGGING, never control, so a long
// dump lockout is low-risk.
constexpr int64_t kDumpSessionCapUs = 60LL * 1000000;
constexpr int kHttpdStack = 12288;
constexpr int kHttpdMaxSockets = 3;   // dumps are single-client; leave lwIP sockets to api/ota/udp
constexpr int kTickTaskStack = 4096;
constexpr UBaseType_t kTickTaskPrio = 24;  // above WiFi (23) — esp_timer lost against WiFi (lab lesson)
// Shared-ADC window: the main task starts an extra-channel conversion only if
// the next tick is at least this far away (one oneshot read is a few tens of
// µs on the S2; the margin absorbs a WiFi preemption of the main task).
constexpr uint32_t kSharedReadGuardUs = 250;
// An IDLE non-owner only freezes its core (cheap path) once its L1 thermal EMA
// is below this: the re-seed on the next grant restarts the EMA at 0, so the
// thermal budget error is bounded by this value (with tau = 120 s that is
// ~6 s of full duty). Above it the instance keeps the full path and the EMA
// decays exactly as the core computes it.
constexpr float kThermalCheapMax = 0.05f;
// Linear fallback full-scale [mV] per attenuation for 13-bit S2 counts when the
// eFuse carries no ADC calibration (IDF datasheet ranges, approximate).
constexpr float kFallbackFullScaleMv[4] = {750.0f, 1050.0f, 1300.0f, 2500.0f};
constexpr float kAdcMaxCount = 8191.0f;

int16_t sat16(long v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}
float clampf(float v, float lo, float hi) { return std::min(std::max(v, lo), hi); }

}  // namespace

// ============================================================================
// SliderServo — main-task surface
// ============================================================================

void SliderServo::setup() {
  if (this->hub_ == nullptr || !this->hub_->setup_once())
    this->mark_failed();
}

bool SliderServo::command_target(float target_pct) {
  // R5: nothing before the boot has completed may command a move — this also
  // catches a Number restore or an on_boot lambda that slipped into a YAML.
  if (!App.is_setup_complete() || !this->hub_->is_started()) {
    this->event_.store((uint8_t) StatusEvent::BOOT_REJECT);
    return false;
  }
  this->cmd_target_.store(target_pct);
  this->cmd_seq_.fetch_add(1);
  if (!this->hub_->enqueue_request(this->slot_)) {
    this->event_.store((uint8_t) StatusEvent::QUEUE_FULL);
    return false;
  }
  this->event_.store((uint8_t) StatusEvent::NONE);
  return true;
}

bool SliderServo::start_ident() {
  if (!App.is_setup_complete() || !this->hub_->is_started()) {
    this->event_.store((uint8_t) StatusEvent::BOOT_REJECT);
    return false;
  }
  this->ident_req_.store(true);
  if (!this->hub_->enqueue_request(this->slot_)) {
    this->ident_req_.store(false);
    this->event_.store((uint8_t) StatusEvent::QUEUE_FULL);
    return false;
  }
  this->event_.store((uint8_t) StatusEvent::NONE);
  return true;
}

void SliderServo::abort_ident() {
  // everything latched up to now is void; a later command runs normally
  this->discard_seq_.store(this->cmd_seq_.load());
  this->ident_req_.store(false);
  this->cancel_req_.store(true);
  this->event_.store((uint8_t) StatusEvent::ABORTED);
}

uint16_t SliderServo::get_last_tick_us() const { return this->hub_->get_last_tick_us(); }
uint32_t SliderServo::get_missed_ticks() const { return this->hub_->get_missed_ticks(); }
uint16_t SliderServo::get_tick_p99_us() const { return this->hub_->get_tick_p99_us(); }
uint32_t SliderServo::get_adc_error_total() const { return this->hub_->get_adc_error_total(); }
uint32_t SliderServo::get_adc_contention() const { return this->hub_->get_adc_contention(); }
uint32_t SliderServo::get_queue_overflows() const { return this->hub_->get_queue_overflows(); }

uint16_t SliderServo::get_idle_path_p99_us() const {
  uint64_t total = 0;
  for (int i = 0; i < kCheapHistBuckets; i++)
    total += this->cheap_hist_[i];
  if (total == 0)
    return 0;
  const uint64_t thresh = (total * 99 + 99) / 100;
  uint64_t cum = 0;
  for (int i = 0; i < kCheapHistBuckets; i++) {
    cum += this->cheap_hist_[i];
    if (cum >= thresh)
      return (uint16_t) ((i + 1) * kCheapHistStepUs);
  }
  return (uint16_t) (kCheapHistBuckets * kCheapHistStepUs);
}

void SliderServo::dump_config() {
  ESP_LOGCONFIG(TAG, "slider_servo '%s' (slot %u, NVS index %u -> key 0x%08X):", this->name_id_, this->slot_,
                this->nvs_index_, (unsigned) (kCalibMagic + this->nvs_index_));
  ESP_LOGCONFIG(TAG, "  ADC1 ch%u, 13-bit, ATTEN_DB_12, %d samples/tick", this->adc_channel_, kAdcSamples);
  ESP_LOGCONFIG(TAG, "  H-bridge AIN1=GPIO%u AIN2=GPIO%u on LEDC ch%d/%d, %u Hz 12-bit, FAST=brake-low",
                this->ain1_pin_, this->ain2_pin_, (int) this->ledc_ch_[0], (int) this->ledc_ch_[1],
                (unsigned) kPwmFreqHz);
  if (this->sync_pin_ == kNoPin) {
    ESP_LOGCONFIG(TAG, "  sync pin: none");
  } else {
    ESP_LOGCONFIG(TAG, "  sync pin: GPIO%u", this->sync_pin_);
  }
  ESP_LOGCONFIG(TAG, "  invert_direction: %s, idle path: %s (sample divider %u)",
                this->invert_direction_ ? "yes" : "no", this->idle_full_ ? "full" : "cheap", this->idle_div_);
  ESP_LOGCONFIG(TAG, "  endpoints: raw %.0f -> 0 %%, raw %.0f -> 100 %%", this->raw0_, this->raw100_);
  ESP_LOGCONFIG(TAG, "  calib: %s", this->calib_valid_ ? "NVS" : "nominal defaults (nicht identifiziert)");
  ESP_LOGCONFIG(TAG, "  recorder: %u records (%.1f s) in %s", (unsigned) this->capacity_,
                this->capacity_ / 1000.0f, this->buf_in_psram_ ? "PSRAM" : "internal RAM (fallback!)");
  ESP_LOGCONFIG(TAG, "  cfg: tol %.2f %%, v_cruise %.0f, v_min %.0f, settle %d/%d ms, u_max %.2f",
                this->cfg_.tol, this->cfg_.v_cruise, this->cfg_.v_min_cmd, this->cfg_.impulse_settle_ms,
                this->cfg_.impulse_settle_short_ms, this->cfg_.u_max);
  ESP_LOGCONFIG(TAG, "  thermal: tau %.0f s, trip %.2f, resume %.2f", this->cfg_.thermal_tau_s,
                this->cfg_.thermal_avg_max, this->cfg_.thermal_resume);
  ESP_LOGCONFIG(TAG, "  dump: %s", this->dump_uri_.c_str());
  if (this->slot_ == 0)
    this->hub_->dump_config();
}

// ============================================================================
// SliderServo — hardware setup (called by the hub, setup context)
// ============================================================================

bool SliderServo::hw_setup_(adc_oneshot_unit_handle_t adc, uint8_t slot, ledc_channel_t ch_a, ledc_channel_t ch_b,
                            int64_t t0_us) {
  this->adc_ = adc;
  this->slot_ = slot;
  this->ledc_ch_[0] = ch_a;
  this->ledc_ch_[1] = ch_b;
  this->rec_t0_us_ = t0_us;
  this->idle_countdown_ = 0;

  // latch WHY we last rebooted (diagnostic — a mid-operation reboot was seen
  // once during bring-up, correlated with a pathological slow HTTP dump)
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: this->reset_reason_ = "poweron"; break;
    case ESP_RST_EXT: this->reset_reason_ = "ext"; break;
    case ESP_RST_SW: this->reset_reason_ = "sw/ota"; break;
    case ESP_RST_PANIC: this->reset_reason_ = "panic"; break;
    case ESP_RST_INT_WDT: this->reset_reason_ = "int_wdt"; break;
    case ESP_RST_TASK_WDT: this->reset_reason_ = "task_wdt"; break;
    case ESP_RST_WDT: this->reset_reason_ = "wdt"; break;
    case ESP_RST_BROWNOUT: this->reset_reason_ = "brownout"; break;
    case ESP_RST_DEEPSLEEP: this->reset_reason_ = "deepsleep"; break;
    default: this->reset_reason_ = "other"; break;
  }

  // --- optional scope-sync pin: LOW when idle, readable back (INPUT_OUTPUT).
  //     Without a pin both masks stay 0 and the tick's register writes are
  //     no-ops — same code path either way (no branch in the tick). ---
  if (this->sync_pin_ != kNoPin) {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << this->sync_pin_;
    io.mode = GPIO_MODE_INPUT_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level((gpio_num_t) this->sync_pin_, 0);
    if (this->sync_pin_ < 32)
      this->sync_lo_ = 1u << this->sync_pin_;
    else
      this->sync_hi_ = 1u << (this->sync_pin_ - 32);
  }

  // --- LEDC channels on the shared servo timer; boot state 0 = brake-low, silent ---
  const uint8_t pins[2] = {this->ain1_pin_, this->ain2_pin_};
  for (int i = 0; i < 2; i++) {
    ledc_channel_config_t ccfg = {};
    ccfg.gpio_num = pins[i];
    ccfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ccfg.channel = this->ledc_ch_[i];
    ccfg.intr_type = LEDC_INTR_DISABLE;
    ccfg.timer_sel = kLedcTimer;
    ccfg.duty = 0;
    ccfg.hpoint = 0;
    if (ledc_channel_config(&ccfg) != ESP_OK) {
      ESP_LOGE(TAG, "'%s': LEDC channel %d config failed", this->name_id_, (int) this->ledc_ch_[i]);
      return false;
    }
    this->ledc_last_[i] = 0;
  }

  // --- wiper channel on the hub's ADC1 unit, 13-bit, ATTEN_DB_12 ---
  adc_oneshot_chan_cfg_t chcfg = {};
  chcfg.atten = ADC_ATTEN_DB_12;
  chcfg.bitwidth = ADC_BITWIDTH_13;
  if (adc_oneshot_config_channel(this->adc_, (adc_channel_t) this->adc_channel_, &chcfg) != ESP_OK) {
    ESP_LOGE(TAG, "'%s': ADC channel %u config failed", this->name_id_, this->adc_channel_);
    return false;
  }

  // --- recorder ring: PSRAM first (descending), internal RAM fallback ---
  uint32_t want = this->req_capacity_ > 0 ? this->req_capacity_ : 45000;
  const uint32_t psram_sizes[] = {want, 45000, 30000, 15000, 8000};
  for (uint32_t n : psram_sizes) {
    if (n > want)
      continue;
    this->buf_ = (Record *) heap_caps_malloc((size_t) n * sizeof(Record), MALLOC_CAP_SPIRAM);
    if (this->buf_ != nullptr) {
      this->capacity_ = n;
      this->buf_in_psram_ = true;
      break;
    }
  }
  if (this->buf_ == nullptr) {
    ESP_LOGW(TAG, "'%s': PSRAM alloc failed - falling back to internal RAM (short trace!)", this->name_id_);
    const uint32_t int_sizes[] = {4000, 2000, 1000};
    for (uint32_t n : int_sizes) {
      this->buf_ = (Record *) heap_caps_malloc((size_t) n * sizeof(Record), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      if (this->buf_ != nullptr) {
        this->capacity_ = n;
        this->buf_in_psram_ = false;
        break;
      }
    }
  }
  if (this->buf_ == nullptr) {
    ESP_LOGE(TAG, "'%s': recorder ring allocation failed entirely", this->name_id_);
    return false;
  }

  // --- NVS UnitCalib (key 'SVRC' + index; index 0 == the rig's existing key).
  //     Valid blob -> per-unit calib; otherwise nominal defaults (R8). ---
  this->pref_ = global_preferences->make_preference<StoredCalib>(kCalibMagic + this->nvs_index_);
  StoredCalib sc;
  if (this->pref_.load(&sc) && sc.magic == kCalibMagic && sc.version == kCalibVersion) {
    this->calib_ = sc.calib;
    this->calib_valid_ = true;
    ESP_LOGI(TAG, "'%s': loaded per-unit calib from NVS", this->name_id_);
  } else {
    ESP_LOGI(TAG, "'%s': no valid NVS calib - nominal defaults (nicht identifiziert)", this->name_id_);
  }

  // --- seed the observer at the current position so it starts converged ---
  float p0 = 50.0f;
  {
    int32_t sum = 0;
    int got = 0;
    for (int i = 0; i < kSeedSamples; i++) {
      int raw = 0;
      if (adc_oneshot_read(this->adc_, (adc_channel_t) this->adc_channel_, &raw) == ESP_OK) {
        sum += raw;
        got++;
      }
    }
    if (got > 0) {
      this->raw_mean_ = (int) (sum / got);
      p0 = clampf(this->raw_to_percent_((float) sum / (float) got), 0.0f, 100.0f);
    }
  }
  this->z_ = p0;
  this->raw_pub_.store(this->raw_mean_);

  // recorder ring disabled inside the core (capacity 0): the wrapper owns the
  // PSRAM ring and taps the core's read-only accessors (T13: control is
  // bit-identical whether or not anything records). R5: the core boots IDLE
  // with zero torque and hold := measured position.
  this->servo_.init(&this->calib_, &this->cfg_, p0, 0, core::SeededBug::NONE);
  this->target_echo_.store(p0);
  this->pos_pct_.store(p0);
  this->p_hat_.store(p0);
  this->last_seq_ = this->cmd_seq_.load();
  ESP_LOGI(TAG, "'%s': core seeded at %.1f %% (IDLE, zero torque until commanded)", this->name_id_, p0);

  this->dump_uri_ = std::string("/servo/") + this->name_id_ + "/dump.csv";
  return true;
}

// ============================================================================
// SliderServo — per-instance tick (tick task). NOTHING here may block.
// ============================================================================

void SliderServo::sample_(bool adc_free) {
  // 4 oneshot reads, mean, count failures (never react to the error in-tick:
  // 2026-07-17 the ADC read flat 0 for minutes; only OTA reboot cleared it —
  // the counter names the failure instead of silence). If the main task holds
  // the shared ADC (adc_free == false) the tick does NOT wait: z stays at the
  // last sample (sample-and-hold, counted by the hub as contention).
  if (!adc_free)
    return;
  int32_t sum = 0;
  for (int i = 0; i < kAdcSamples; i++) {
    int raw = 0;
    if (adc_oneshot_read(this->adc_, (adc_channel_t) this->adc_channel_, &raw) != ESP_OK)
      this->adc_err_count_.fetch_add(1, std::memory_order_relaxed);
    sum += raw;
  }
  this->raw_mean_ = (int) (sum / kAdcSamples);
  this->z_ = this->raw_to_percent_((float) sum / (float) kAdcSamples);
  this->raw_pub_.store(this->raw_mean_, std::memory_order_relaxed);
}

float SliderServo::tick_(bool owner, bool adc_free) {
  const int64_t t_entry = esp_timer_get_time();

  // Path choice: the owner always runs the full core. A non-owner runs it too
  // unless the cheap path is enabled and its thermal EMA is low (see
  // kThermalCheapMax). Cheap path = sampling (optionally decimated, >= 100 Hz)
  // + published state, no observer/controller work, torque 0.
  const bool full = owner || this->idle_full_ || !this->cheap_ok_;

  if (full) {
    this->sample_(adc_free);
    const float z = this->z_;
    this->servo_.set_abort(this->abort_req_.load(std::memory_order_relaxed));

    // one control evaluation: observer -> §2 graph -> L1. Returns applied duty.
    float duty = this->servo_.tick(z);
    // R1 backstop: only the grant holder may drive. A non-owner core asking
    // for torque (it cannot from IDLE; this guards a future core change) is
    // forced to 0 and counted — never two bridges driven in one tick.
    if (!owner && duty != 0.0f) {
      this->arbiter_blocked_.fetch_add(1, std::memory_order_relaxed);
      duty = 0.0f;
    }
    this->apply_duty_(duty);

    const core::State st = this->servo_.controller().state();
    const bool driving = st == core::State::MOVE || st == core::State::BRAKE || st == core::State::PULSE ||
                         st == core::State::CRAWL || st == core::State::GLIDE;
    this->write_sync_(driving);

    const uint16_t tick_us = (uint16_t) (esp_timer_get_time() - t_entry);
    this->snapshot_(z, duty, tick_us);
    this->cheap_ok_ = this->servo_.safety().thermal_avg() < kThermalCheapMax;
    if (tick_us > this->full_max_us_.load(std::memory_order_relaxed))
      this->full_max_us_.store(tick_us, std::memory_order_relaxed);
    this->full_ticks_.fetch_add(1, std::memory_order_relaxed);
    return duty;
  }

  // ---- cheap path (IDLE non-owner) ----
  this->core_stale_ = true;
  if (this->idle_countdown_ == 0) {
    this->sample_(adc_free);
    this->idle_countdown_ = this->idle_div_;
    this->pos_pct_.store(this->z_, std::memory_order_relaxed);
  }
  this->idle_countdown_--;
  this->apply_duty_(0.0f);  // cached: no register write while already 0
  this->write_sync_(false);
  const uint16_t tick_us = (uint16_t) (esp_timer_get_time() - t_entry);
  const int bucket = std::min((int) (tick_us / kCheapHistStepUs), kCheapHistBuckets - 1);
  this->cheap_hist_[bucket]++;
  if (tick_us > this->cheap_max_us_.load(std::memory_order_relaxed))
    this->cheap_max_us_.store(tick_us, std::memory_order_relaxed);
  this->cheap_ticks_.fetch_add(1, std::memory_order_relaxed);
  return 0.0f;
}

bool SliderServo::owner_done_() const {
  return this->servo_.controller().state() == core::State::IDLE && !this->core_ident_active_();
}

void SliderServo::deliver_() {
  // the grant was just given (or the owner is retargeted): bring a frozen core
  // up to date first, then execute what is latched. Latest target wins
  // (set-point semantics); commands voided by Abbruch are skipped.
  if (this->core_stale_) {
    this->core_resync_(this->z_);
    this->core_stale_ = false;
  }
  if (this->ident_req_.exchange(false)) {
    if (this->core_start_ident_())
      return;  // ident owns the motor; the core refuses set_target meanwhile (spec §3)
    // refused: say so, and still execute a move latched with it (its ring
    // entry was coalesced into this grant — it must not get lost, K6)
    this->event_.store((uint8_t) StatusEvent::IDENT_UNAVAILABLE, std::memory_order_relaxed);
  }
  const uint32_t seq = this->cmd_seq_.load();
  const uint32_t void_upto = this->discard_seq_.load();
  if (seq != this->last_seq_ && seq != void_upto) {
    this->last_seq_ = seq;
    this->servo_.set_target(this->cmd_target_.load(), this->cmd_speed_.load());
  } else {
    this->last_seq_ = seq;
  }
}

void SliderServo::handle_release_() {
  if (this->cancel_req_.exchange(false)) {
    this->core_abort_ident_();
    this->last_seq_ = this->discard_seq_.load();
    // a frozen core is IDLE with zero torque already — nothing to release
    if (!this->core_stale_)
      this->servo_.release();
  }
  if (this->release_req_.exchange(false) && !this->core_stale_)
    this->servo_.release();
}

void SliderServo::write_sync_(bool driving) {
  // branch-free: masks are 0 without a sync pin, so both writes are no-ops
  const uint32_t on = 0u - (uint32_t) driving;
  REG_WRITE(GPIO_OUT_W1TS_REG, this->sync_lo_ & on);
  REG_WRITE(GPIO_OUT_W1TC_REG, this->sync_lo_ & ~on);
#if SOC_GPIO_PIN_COUNT > 32
  REG_WRITE(GPIO_OUT1_W1TS_REG, this->sync_hi_ & on);
  REG_WRITE(GPIO_OUT1_W1TC_REG, this->sync_hi_ & ~on);
#endif
}

void SliderServo::snapshot_(float z, float duty, uint16_t tick_us) {
  const core::Observer &obs = this->servo_.observer();
  const core::Controller &ctl = this->servo_.controller();
  const core::ControllerLog &log = ctl.log();

  // HA-visible atomics
  this->pos_pct_.store(z);
  this->p_hat_.store(obs.p_hat());
  this->v_hat_.store(obs.v_hat());
  // show where we actually hold when settled (else the last commanded target);
  // avoids the idle display reading the controller's default target of 50.
  this->target_echo_.store(ctl.settled() ? ctl.hold() : ctl.target());
  this->thermal_avg_.store(this->servo_.safety().thermal_avg());
  this->pulses_.store(log.pulses);
  this->kicks_.store(log.kicks);
  this->overshoots_.store(log.overshoots);
  this->state_pub_.store((uint8_t) ctl.state());
  this->settle_pub_.store((uint8_t) ctl.settle_flag());
  this->release_pub_.store((uint8_t) ctl.release_reason());
  this->settled_.store(ctl.settled());

  // recorder ring — paused (not stalled) during a dump so the httpd task reads
  // a stable buffer. Pausing the LOG never pauses CONTROL.
  if (this->dump_active_.load() || this->buf_ == nullptr)
    return;
  const uint32_t head = this->rec_head_.load();
  Record &r = this->buf_[head];
  r.t_ms = (uint32_t) ((esp_timer_get_time() - this->rec_t0_us_) / 1000);
  r.z_c = sat16(lrintf(z * 100.0f));
  r.p_c = sat16(lrintf(obs.p_hat() * 100.0f));
  r.v_d = sat16(lrintf(obs.v_hat() * 10.0f));
  r.u_cmd_m = sat16(lrintf(this->servo_.u_cmd_last() * 1000.0f));
  r.u_app_m = sat16(lrintf(duty * 1000.0f));
  r.vdes_d = sat16(lrintf(ctl.last_v_des() * 10.0f));
  r.ff_m = sat16(lrintf(ctl.last_ff() * 1000.0f));
  r.tick_us = tick_us;
  r.state = (uint8_t) ctl.state();
  r.settle = (uint8_t) ctl.settle_flag();
  r.release = (uint8_t) ctl.release_reason();
  r.clamped = this->servo_.safety().clamped() ? 1 : 0;

  const uint32_t nc = this->rec_count_.load();
  this->rec_head_.store((head + 1) % this->capacity_);
  if (nc < this->capacity_)
    this->rec_count_.store(nc + 1);
  else if (!this->recorder_full_.load())
    this->recorder_full_.store(true);
}

// ============================================================================
// duty -> H-bridge via LEDC. FAST decay recipe (brake-low), verbatim from
// slider_lab: fwd AIN1=PWM(d)/AIN2=0, rev mirrored, both 0 when off. The L9110
// cannot coast, so "off" is a brake — this is exactly what the plant was
// identified with (UnitCalib.b includes the brake damping).
// ============================================================================

void SliderServo::apply_duty_(float duty) {
  const float t = clampf(duty, -1.0f, 1.0f);
  const float phys = this->invert_direction_ ? -t : t;
  auto duty_of = [](float x) -> uint32_t { return (uint32_t) lrintf(clampf(x, 0.0f, 1.0f) * (float) kPwmFull); };

  uint32_t a1 = 0, a2 = 0;
  if (phys > 0.0f)
    a1 = duty_of(phys);
  else if (phys < 0.0f)
    a2 = duty_of(-phys);
  this->set_ledc_(0, a1);
  this->set_ledc_(1, a2);
}

void SliderServo::set_ledc_(uint8_t idx, uint32_t duty) {
  if (this->ledc_last_[idx] == duty)
    return;
  this->ledc_last_[idx] = duty;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, this->ledc_ch_[idx], duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, this->ledc_ch_[idx]);
}

// ============================================================================
// ===== CORE-ADAPTER (Ident + resync) ========================================
// The core agent delivers Servo::start_ident()/abort_ident() (L6, spec §3) in
// parallel. Until the merge these are documented stubs: Ident reports
// "ident nicht verfügbar" and never touches the motor. MERGE JOB (~10 lines):
//   core_start_ident_()  -> return this->servo_.start_ident();
//   core_abort_ident_()  -> this->servo_.abort_ident();
//   core_ident_active_() -> return this->servo_.ident_active();   (keeps the grant)
//   core_ident_phase_()  -> return this->servo_.ident_phase_name(); (status text)
//   core_resync_(z)      -> this->servo_.resync(z);   (wish: keeps L1 EMA + logs)
// All are called from the tick task only (the core is single-threaded).
// ============================================================================

bool SliderServo::core_start_ident_() { return false; }
void SliderServo::core_abort_ident_() {}
bool SliderServo::core_ident_active_() const { return false; }
const char *SliderServo::core_ident_phase_() const { return nullptr; }

void SliderServo::core_resync_(float z) {
  // Re-seed a core that was frozen on the cheap path at the current position.
  // Stage-0 stand-in: init() (no heap: core recorder capacity 0). It resets the
  // diagnostic logs (pulses/kicks/overshoots) and the L1 thermal EMA — the EMA
  // error is bounded by kThermalCheapMax because the cheap path is only taken
  // below it. Core wish: Servo::resync(z) that keeps both.
  this->servo_.init(&this->calib_, &this->cfg_, clampf(z, 0.0f, 100.0f), 0, core::SeededBug::NONE);
}

// ============================================================================
// SliderServo — status / names (main task)
// ============================================================================

std::string SliderServo::get_state_name() const {
  switch ((core::State) this->state_pub_.load()) {
    case core::State::IDLE: return "idle";
    case core::State::MOVE: return "move";
    case core::State::BRAKE: return "brake";
    case core::State::PULSE: return "pulse";
    case core::State::WAIT: return "wait";
    case core::State::CRAWL: return "crawl";
    case core::State::RELEASED: return "released";
    case core::State::GLIDE: return "glide";
  }
  return "?";
}

std::string SliderServo::get_settle_name() const {
  switch ((core::SettleFlag) this->settle_pub_.load()) {
    case core::SettleFlag::NONE: return "none";
    case core::SettleFlag::REACHED: return "reached";
    case core::SettleFlag::GAVE_UP: return "gave_up";
    case core::SettleFlag::ADOPTED: return "adopted";
  }
  return "?";
}

std::string SliderServo::get_release_name() const {
  switch ((core::ReleaseReason) this->release_pub_.load()) {
    case core::ReleaseReason::NONE: return "none";
    case core::ReleaseReason::GRAB: return "grab";
    case core::ReleaseReason::INTERLOCK: return "interlock";
    case core::ReleaseReason::WATCHDOG: return "watchdog";
    case core::ReleaseReason::FAULT: return "fault";
    case core::ReleaseReason::COMMAND: return "command";
  }
  return "?";
}

std::string SliderServo::get_status_text() const {
  std::string s;
  const auto ev = (StatusEvent) this->event_.load();
  const char *phase = this->core_ident_phase_();
  if (phase != nullptr) {
    s = std::string("ident(") + phase + ")";
  } else if (ev == StatusEvent::BOOT_REJECT) {
    s = "befehl vor boot-ende verworfen";
  } else if (ev == StatusEvent::QUEUE_FULL) {
    s = "queue voll, befehl abgewiesen";
  } else if (ev == StatusEvent::IDENT_UNAVAILABLE) {
    s = "ident nicht verfügbar";
  } else if (this->waiting_.load()) {
    s = "wartet (" + std::to_string(this->hub_->wait_position(this->slot_)) + ". in queue)";
  } else {
    switch ((core::State) this->state_pub_.load()) {
      case core::State::IDLE: s = ev == StatusEvent::ABORTED ? "idle (abgebrochen)" : "idle"; break;
      case core::State::MOVE:
      case core::State::BRAKE:
      case core::State::PULSE:
      case core::State::WAIT: s = "moving"; break;
      case core::State::CRAWL:
      case core::State::GLIDE: s = "crawl"; break;
      case core::State::RELEASED: {
        const auto rr = (core::ReleaseReason) this->release_pub_.load();
        const char *why = rr == core::ReleaseReason::GRAB        ? "grab"
                          : rr == core::ReleaseReason::INTERLOCK ? "interlock"
                          : rr == core::ReleaseReason::WATCHDOG  ? "watchdog"
                          : rr == core::ReleaseReason::FAULT     ? "fault"
                                                                 : "cmd";
        s = std::string("released(") + why + ")";
        break;
      }
    }
  }
  if (!this->calib_valid_)
    s += ", nicht identifiziert";
  return s;
}

void SliderServo::save_calib() {
  StoredCalib sc{kCalibMagic, kCalibVersion, this->calib_};
  if (this->pref_.save(&sc)) {
    global_preferences->sync();
    this->calib_valid_ = true;
    ESP_LOGI(TAG, "'%s': saved per-unit calib to NVS", this->name_id_);
  } else {
    ESP_LOGW(TAG, "'%s': NVS calib save failed", this->name_id_);
  }
}

// ============================================================================
// HTTP CSV dump — chunked stream from the PSRAM ring. Recorder is paused (not
// stalled) while streaming so the buffer is stable; control keeps running.
// Rows are INTEGER-formatted (soft-float _dtoa_r crash lesson from the lab).
// ============================================================================

esp_err_t SliderServo::dump_handler_(httpd_req_t *req) {
  return static_cast<SliderServo *>(req->user_ctx)->handle_dump_(req);
}

esp_err_t SliderServo::handle_dump_(httpd_req_t *req) {
  this->dump_active_.store(true);
  // let any in-flight tick finish its record write before we read the ring
  vTaskDelay(pdMS_TO_TICKS(3));

  const int sockfd = httpd_req_to_sockfd(req);
  if (sockfd >= 0) {
    int nodelay = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    struct timeval tv = {};
    tv.tv_sec = kDumpSendTimeoutS;
    tv.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  }

  const uint32_t count = this->rec_count_.load();
  const uint32_t head = this->rec_head_.load();
  const uint32_t start = (this->capacity_ + head - count) % (this->capacity_ ? this->capacity_ : 1);

  httpd_resp_set_type(req, "text/csv");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"servo_dump.csv\"");

  const size_t bufsz = 8192;
  char *buf = (char *) malloc(bufsz);
  if (buf == nullptr) {
    this->dump_active_.store(false);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    return ESP_OK;
  }

  int len = snprintf(buf, bufsz,
                     "# slider_servo dump\n"
                     "# instance: %s index=%u slot=%u calib=%s\n"
                     "# fw: esphome %s, built %s %s\n"
                     "# tick_us_nominal: 1000\n"
                     "# adc: bits=13 atten=DB_12 samples=4  endpoints: raw%0.f=0%% raw%0.f=100%%\n"
                     "# records: %u capacity: %u storage: %s  missed_ticks: %u  adc_errors: %u\n"
                     "# state: 0=idle 1=move 2=brake 3=pulse 4=wait 5=crawl 6=released 7=glide\n"
                     "# settle: 0=none 1=reached 2=gave_up 3=adopted\n"
                     "# release: 0=none 1=grab 2=interlock 3=watchdog 4=fault 5=command\n"
                     "# columns: z/p_hat in %%, v_hat/v_des in %%/s, u_cmd/u_applied/ff in duty\n",
                     this->name_id_, this->nvs_index_, this->slot_, this->calib_valid_ ? "nvs" : "nominal",
                     ESPHOME_VERSION, __DATE__, __TIME__, this->raw0_, this->raw100_, (unsigned) count,
                     (unsigned) this->capacity_, this->buf_in_psram_ ? "psram" : "internal",
                     (unsigned) this->hub_->get_missed_ticks(), (unsigned) this->adc_err_count_.load());
  esp_err_t err = httpd_resp_send_chunk(req, buf, len);

  if (err == ESP_OK) {
    len = snprintf(buf, bufsz,
                   "t_ms,z,p_hat,v_hat,u_cmd,u_applied,v_des,ff,tick_us,state,settle,release,clamped\n");
    err = httpd_resp_send_chunk(req, buf, len);
  }

  const int64_t deadline_us = esp_timer_get_time() + kDumpSessionCapUs;
  size_t used = 0;
  for (uint32_t i = 0; i < count && err == ESP_OK; i++) {
    if (esp_timer_get_time() > deadline_us) {
      err = ESP_ERR_TIMEOUT;
      break;
    }
    const Record &r = this->buf_[(start + i) % this->capacity_];
#define SGN(x) ((x) < 0 ? "-" : "")
#define ABSU(x) ((unsigned) ((x) < 0 ? -(int) (x) : (int) (x)))
    int m = snprintf(buf + used, bufsz - used,
                     "%u,%s%u.%02u,%s%u.%02u,%s%u.%01u,%s%u.%03u,%s%u.%03u,%s%u.%01u,%s%u.%03u,%u,%u,%u,%u,%u\n",
                     (unsigned) r.t_ms, SGN(r.z_c), ABSU(r.z_c) / 100, ABSU(r.z_c) % 100, SGN(r.p_c),
                     ABSU(r.p_c) / 100, ABSU(r.p_c) % 100, SGN(r.v_d), ABSU(r.v_d) / 10, ABSU(r.v_d) % 10,
                     SGN(r.u_cmd_m), ABSU(r.u_cmd_m) / 1000, ABSU(r.u_cmd_m) % 1000, SGN(r.u_app_m),
                     ABSU(r.u_app_m) / 1000, ABSU(r.u_app_m) % 1000, SGN(r.vdes_d), ABSU(r.vdes_d) / 10,
                     ABSU(r.vdes_d) % 10, SGN(r.ff_m), ABSU(r.ff_m) / 1000, ABSU(r.ff_m) % 1000,
                     (unsigned) r.tick_us, (unsigned) r.state, (unsigned) r.settle, (unsigned) r.release,
                     (unsigned) r.clamped);
#undef SGN
#undef ABSU
    if (m < 0)
      break;
    used += (size_t) m;
    if (used > bufsz - 128) {
      err = httpd_resp_send_chunk(req, buf, used);
      used = 0;
      // Yield after every chunk: this httpd task (low prio) is CPU-bound
      // building rows while the 1 kHz tick (prio 24) + WiFi (23) run — without
      // an explicit yield the IDLE task can starve and trip the task watchdog
      // on a long/slow dump. ~1 ms per 8 KB chunk is negligible (~70 ms total).
      vTaskDelay(1);
    }
  }
  if (err == ESP_OK && used > 0)
    err = httpd_resp_send_chunk(req, buf, used);
  if (err == ESP_OK)
    err = httpd_resp_send_chunk(req, nullptr, 0);

  free(buf);
  this->dump_active_.store(false);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "'%s': dump aborted after %u records (%s)", this->name_id_, (unsigned) count,
             err == ESP_ERR_TIMEOUT ? "60 s session cap" : "client stalled/gone");
    return ESP_FAIL;
  }
  return ESP_OK;
}

// ============================================================================
// SliderServoHub — codegen + setup
// ============================================================================

void SliderServoHub::register_servo(SliderServo *s) {
  if (this->n_servos_ >= kMaxInstances)
    return;  // codegen enforces MULTI_CONF <= kMaxInstances
  this->servos_[this->n_servos_++] = s;
}

uint8_t SliderServoHub::add_extra_channel(uint8_t channel, uint8_t atten) {
  if (this->n_extras_ >= kMaxExtraChannels)
    return 0xFF;  // codegen enforces the limit
  this->extras_[this->n_extras_] = Extra{channel, atten};
  return this->n_extras_++;
}

bool SliderServoHub::setup_ledc_timer_() {
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode = LEDC_LOW_SPEED_MODE;
  tcfg.duty_resolution = kPwmRes;
  tcfg.timer_num = kLedcTimer;
  tcfg.freq_hz = kPwmFreqHz;
  tcfg.clk_cfg = LEDC_AUTO_CLK;
  return ledc_timer_config(&tcfg) == ESP_OK;
}

void SliderServoHub::setup_calibration_() {
  // one calibration scheme per attenuation in use (main-task conversion only)
  bool need[kAttenLevels] = {false, false, false, true};  // wipers: 12 dB
  for (uint8_t i = 0; i < this->n_extras_; i++)
    if (this->extras_[i].atten < kAttenLevels)
      need[this->extras_[i].atten] = true;
  for (int a = 0; a < kAttenLevels; a++) {
    if (!need[a])
      continue;
    esp_err_t err = ESP_ERR_NOT_SUPPORTED;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cc = {};
    cc.unit_id = ADC_UNIT_1;
    cc.atten = (adc_atten_t) a;
    cc.bitwidth = ADC_BITWIDTH_13;
    err = adc_cali_create_scheme_curve_fitting(&cc, &this->cali_[a]);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t lc = {};
    lc.unit_id = ADC_UNIT_1;
    lc.atten = (adc_atten_t) a;
    lc.bitwidth = ADC_BITWIDTH_13;
    err = adc_cali_create_scheme_line_fitting(&lc, &this->cali_[a]);
#endif
    if (err != ESP_OK) {
      this->cali_[a] = nullptr;
      ESP_LOGW(TAG, "ADC1 calibration for atten %d unavailable - linear fallback", a);
    }
  }
}

bool SliderServoHub::setup_once() {
  if (this->setup_done_)
    return this->started_;
  this->setup_done_ = true;
  this->t0_us_ = esp_timer_get_time();
  for (int i = 0; i < kMaxInstances; i++)
    this->wait_pos_[i].store(0);

  this->producer_mtx_ = xSemaphoreCreateMutex();
  if (this->producer_mtx_ == nullptr) {
    ESP_LOGE(TAG, "arbiter mutex creation failed");
    return false;
  }

  // --- ADC1: ONE oneshot unit for all instances + extra channels (R2) ---
  adc_oneshot_unit_init_cfg_t ucfg = {};
  ucfg.unit_id = ADC_UNIT_1;
  ucfg.ulp_mode = ADC_ULP_MODE_DISABLE;
  if (adc_oneshot_new_unit(&ucfg, &this->adc_) != ESP_OK) {
    ESP_LOGE(TAG, "ADC1 oneshot unit init failed (is another component using ADC1?)");
    return false;
  }
  for (uint8_t i = 0; i < this->n_extras_; i++) {
    adc_oneshot_chan_cfg_t chcfg = {};
    chcfg.atten = (adc_atten_t) this->extras_[i].atten;
    chcfg.bitwidth = ADC_BITWIDTH_13;
    if (adc_oneshot_config_channel(this->adc_, (adc_channel_t) this->extras_[i].channel, &chcfg) != ESP_OK) {
      ESP_LOGE(TAG, "ADC1 extra channel %u config failed", this->extras_[i].channel);
      return false;
    }
  }
  this->setup_calibration_();

  // --- LEDC: one timer for every servo channel (all run 19531 Hz / 12 bit) ---
  if (!this->setup_ledc_timer_()) {
    ESP_LOGE(TAG, "LEDC timer config failed");
    return false;
  }

  // --- per-instance hardware (LEDC pair k = channels top-2k / top-2k-1) ---
  for (uint8_t k = 0; k < this->n_servos_; k++) {
    const auto ch_a = static_cast<ledc_channel_t>(kLedcTopChannel - 2 * k);
    const auto ch_b = static_cast<ledc_channel_t>(kLedcTopChannel - 2 * k - 1);
    if (!this->servos_[k]->hw_setup_(this->adc_, k, ch_a, ch_b, this->t0_us_))
      return false;
  }

  // --- ONE httpd: /servo/<id>/dump.csv per instance, /servo/dump.csv = index 0 ---
  httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
  hcfg.server_port = this->http_port_;
  hcfg.ctrl_port = this->http_port_ + 1;  // must differ from any other httpd's ctrl port
  hcfg.stack_size = kHttpdStack;
  hcfg.max_uri_handlers = this->n_servos_ + 1;
  hcfg.max_open_sockets = kHttpdMaxSockets;
  hcfg.lru_purge_enable = true;
  if (httpd_start(&this->httpd_, &hcfg) == ESP_OK) {
    SliderServo *alias = nullptr;
    for (uint8_t k = 0; k < this->n_servos_; k++) {
      SliderServo *s = this->servos_[k];
      httpd_uri_t uri = {};
      uri.uri = s->dump_uri_.c_str();
      uri.method = HTTP_GET;
      uri.handler = SliderServo::dump_handler_;
      uri.user_ctx = s;
      httpd_register_uri_handler(this->httpd_, &uri);
      if (s->nvs_index_ == 0)
        alias = s;
    }
    if (alias == nullptr)
      alias = this->servos_[0];
    httpd_uri_t uri = {};
    uri.uri = "/servo/dump.csv";  // legacy path: analysis/servo-hil tools
    uri.method = HTTP_GET;
    uri.handler = SliderServo::dump_handler_;
    uri.user_ctx = alias;
    httpd_register_uri_handler(this->httpd_, &uri);
  } else {
    ESP_LOGE(TAG, "esp_http_server failed to start on port %u", this->http_port_);
    // not fatal: the tick + interlocks keep running, dump unavailable
  }

  // --- deterministic 1 kHz tick: GPTimer ISR -> notify -> prio-24 task ---
  if (xTaskCreate(SliderServoHub::tick_task_trampoline_, "slider_servo_tick", kTickTaskStack, this, kTickTaskPrio,
                  &this->tick_task_) != pdPASS) {
    ESP_LOGE(TAG, "tick task creation failed");
    return false;
  }
  gptimer_config_t gcfg = {};
  gcfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  gcfg.direction = GPTIMER_COUNT_UP;
  gcfg.resolution_hz = 1000000;
  gptimer_event_callbacks_t cbs = {};
  cbs.on_alarm = SliderServoHub::timer_isr_;
  gptimer_alarm_config_t acfg = {};
  acfg.alarm_count = kTickUs;
  acfg.reload_count = 0;
  acfg.flags.auto_reload_on_alarm = true;
  if (gptimer_new_timer(&gcfg, &this->gptimer_) != ESP_OK ||
      gptimer_register_event_callbacks(this->gptimer_, &cbs, this) != ESP_OK ||
      gptimer_set_alarm_action(this->gptimer_, &acfg) != ESP_OK || gptimer_enable(this->gptimer_) != ESP_OK ||
      gptimer_start(this->gptimer_) != ESP_OK) {
    ESP_LOGE(TAG, "GPTimer start failed");
    return false;
  }
  this->started_ = true;
  return true;
}

void SliderServoHub::dump_config() {
  ESP_LOGCONFIG(TAG, "slider_servo hub: %u instance(s), ADC1 owner (%u extra channel(s)), tick 1 kHz prio %u",
                this->n_servos_, this->n_extras_, (unsigned) kTickTaskPrio);
  for (uint8_t i = 0; i < this->n_extras_; i++)
    ESP_LOGCONFIG(TAG, "  extra ADC1 ch%u atten %u, calibration: %s", this->extras_[i].channel,
                  this->extras_[i].atten, this->cali_[this->extras_[i].atten % kAttenLevels] ? "eFuse" : "linear");
  ESP_LOGCONFIG(TAG, "  LEDC timer %d (shared), httpd port %u, request queue %u", (int) kLedcTimer,
                this->http_port_, (unsigned) kReqQueueCap);
}

// ============================================================================
// SliderServoHub — main-task services
// ============================================================================

bool SliderServoHub::try_read_shared(uint8_t slot, int *raw) {
  if (!this->started_ || slot >= this->n_extras_)
    return false;
  // (a) only right after a tick: the tick task (prio 24) finished, so the
  //     main task runs; demand enough headroom before the next tick.
  const uint32_t since = (uint32_t) esp_timer_get_time() - this->tick_start_us_.load();
  if (since > kTickUs - kSharedReadGuardUs)
    return false;
  // (b) bracket the conversion; a tick landing inside sees the flag and
  //     skips its wiper reads instead of waiting (locking model, header).
  this->adc_busy_.store(true);
  const esp_err_t err = adc_oneshot_read(this->adc_, (adc_channel_t) this->extras_[slot].channel, raw);
  this->adc_busy_.store(false);
  if (err != ESP_OK) {
    this->shared_read_errors_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

float SliderServoHub::raw_to_volts(uint8_t atten, int raw) const {
  const uint8_t a = atten < kAttenLevels ? atten : kAttenLevels - 1;
  int mv = 0;
  if (this->cali_[a] != nullptr && adc_cali_raw_to_voltage(this->cali_[a], raw, &mv) == ESP_OK)
    return (float) mv / 1000.0f;
  return (float) raw / kAdcMaxCount * kFallbackFullScaleMv[a] / 1000.0f;
}

bool SliderServoHub::enqueue_request(uint8_t slot) {
  if (this->producer_mtx_ == nullptr || slot >= this->n_servos_)
    return false;
  // producers serialise among themselves only — the tick never takes this
  xSemaphoreTake(this->producer_mtx_, portMAX_DELAY);
  const uint8_t head = this->req_head_.load(std::memory_order_relaxed);
  const uint8_t tail = this->req_tail_.load(std::memory_order_acquire);
  bool ok = (uint8_t) (head - tail) < kReqQueueCap;
  if (ok) {
    this->req_q_[head & (kReqQueueCap - 1)] = slot;
    this->req_head_.store((uint8_t) (head + 1), std::memory_order_release);
  } else {
    this->queue_overflows_.fetch_add(1, std::memory_order_relaxed);
  }
  xSemaphoreGive(this->producer_mtx_);
  return ok;
}

uint32_t SliderServoHub::get_adc_error_total() const {
  uint32_t n = this->shared_read_errors_.load();
  for (uint8_t k = 0; k < this->n_servos_; k++)
    n += this->servos_[k]->get_adc_error_count();
  return n;
}

uint16_t SliderServoHub::get_tick_p99_us() const {
  uint64_t total = 0;
  for (int i = 0; i < kHistBuckets; i++)
    total += this->tick_hist_[i];
  if (total == 0)
    return 0;
  const uint64_t thresh = (total * 99 + 99) / 100;  // ceil(0.99 * total)
  uint64_t cum = 0;
  for (int i = 0; i < kHistBuckets; i++) {
    cum += this->tick_hist_[i];
    if (cum >= thresh)
      return (uint16_t) ((i + 1) * kHistStepUs);  // upper edge of the bucket
  }
  return (uint16_t) (kHistBuckets * kHistStepUs);
}

// ============================================================================
// SliderServoHub — 1 kHz tick (dedicated task, prio 24). NOTHING may block.
// ============================================================================

bool IRAM_ATTR SliderServoHub::timer_isr_(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
                                          void *arg) {
  auto *self = static_cast<SliderServoHub *>(arg);
  BaseType_t hp_woken = pdFALSE;
  vTaskNotifyGiveFromISR(self->tick_task_, &hp_woken);
  return hp_woken == pdTRUE;
}

void SliderServoHub::tick_task_trampoline_(void *arg) {
  auto *self = static_cast<SliderServoHub *>(arg);
  while (true) {
    // clear-on-exit: coalesce missed notifications into one tick (a t gap is
    // honest; a catch-up burst would fake the timeline). The count is the A9
    // missed-tick metric.
    uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (n > 1)
      self->missed_ticks_.fetch_add(n - 1, std::memory_order_relaxed);
    self->tick_();
  }
}

bool SliderServoHub::fifo_contains_(int8_t slot) const {
  for (uint8_t i = 0; i < this->fifo_n_; i++)
    if (this->fifo_[i] == slot)
      return true;
  return false;
}

void SliderServoHub::fifo_remove_(int8_t slot) {
  uint8_t w = 0;
  for (uint8_t r = 0; r < this->fifo_n_; r++)
    if (this->fifo_[r] != slot)
      this->fifo_[w++] = this->fifo_[r];
  this->fifo_n_ = w;
}

void SliderServoHub::publish_fifo_() {
  for (uint8_t k = 0; k < this->n_servos_; k++) {
    int pos = 0;
    for (uint8_t i = 0; i < this->fifo_n_; i++)
      if (this->fifo_[i] == (int8_t) k)
        pos = i + 1;
    this->wait_pos_[k].store(pos, std::memory_order_relaxed);
    this->servos_[k]->waiting_.store(pos != 0, std::memory_order_relaxed);
    this->servos_[k]->owner_pub_.store(this->owner_ == (int8_t) k, std::memory_order_relaxed);
  }
  this->owner_pub_.store(this->owner_, std::memory_order_relaxed);
}

// R1 arbiter, once per tick before any instance runs. Invariant: at most one
// slot (owner_) may apply duty != 0. Requests are granted in arrival order;
// a request for the current owner retargets it directly only while nobody
// else is waiting (otherwise it queues behind them — no starvation). The
// latched target is read at delivery (latest set-point wins per slider).
void SliderServoHub::arbiter_step_() {
  bool changed = false;
  // 1) Abbruch / release first (same order as the single-instance wrapper:
  //    release before a same-tick command, so the command is stored pending)
  for (uint8_t k = 0; k < this->n_servos_; k++) {
    SliderServo *s = this->servos_[k];
    if (s->cancel_req_.load(std::memory_order_relaxed) && this->fifo_contains_((int8_t) k)) {
      this->fifo_remove_((int8_t) k);
      changed = true;
    }
    s->handle_release_();
  }
  // 2) drain the request ring (lock-free consumer)
  uint8_t tail = this->req_tail_.load(std::memory_order_relaxed);
  const uint8_t head = this->req_head_.load(std::memory_order_acquire);
  while (tail != head) {
    const int8_t slot = (int8_t) this->req_q_[tail & (kReqQueueCap - 1)];
    tail++;
    if (slot == this->owner_ && this->fifo_n_ == 0) {
      this->servos_[slot]->deliver_();
    } else if (!this->fifo_contains_(slot) && slot != this->owner_ && this->fifo_n_ < kMaxInstances) {
      this->fifo_[this->fifo_n_++] = slot;
      changed = true;
    } else if (slot == this->owner_ && !this->fifo_contains_(slot) && this->fifo_n_ < kMaxInstances) {
      this->fifo_[this->fifo_n_++] = slot;  // owner re-request behind the waiters
      changed = true;
    }
    // else: already waiting -> coalesced (the newest latched target is used)
  }
  this->req_tail_.store(tail, std::memory_order_release);
  // 3) the owner hands the grant back once IDLE with nothing pending
  if (this->owner_ >= 0 && this->servos_[this->owner_]->owner_done_()) {
    this->owner_ = -1;
    changed = true;
  }
  // 4) grant the oldest waiter
  if (this->owner_ < 0 && this->fifo_n_ > 0) {
    this->owner_ = this->fifo_[0];
    for (uint8_t i = 1; i < this->fifo_n_; i++)
      this->fifo_[i - 1] = this->fifo_[i];
    this->fifo_n_--;
    this->servos_[this->owner_]->deliver_();
    changed = true;
  }
  if (changed)
    this->publish_fifo_();
}

void SliderServoHub::tick_() {
  const int64_t t_entry = esp_timer_get_time();
  this->tick_start_us_.store((uint32_t) t_entry, std::memory_order_relaxed);
  // shared-ADC flag sampled ONCE: if the main task was preempted inside its
  // extra-channel read, every instance holds its last wiper sample this tick.
  const bool adc_free = !this->adc_busy_.load();
  if (!adc_free)
    this->adc_contention_.fetch_add(1, std::memory_order_relaxed);

  this->arbiter_step_();
  for (uint8_t k = 0; k < this->n_servos_; k++)
    this->servos_[k]->tick_(this->owner_ == (int8_t) k, adc_free);

  const uint16_t tick_us = (uint16_t) (esp_timer_get_time() - t_entry);
  this->last_tick_us_.store(tick_us, std::memory_order_relaxed);
  const int bucket = std::min((int) (tick_us / kHistStepUs), kHistBuckets - 1);
  this->tick_hist_[bucket]++;
}

}  // namespace slider_servo_hal
}  // namespace esphome

#endif  // USE_ESP_IDF
