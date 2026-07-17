#ifdef USE_ESP_IDF

#include "slider_lab.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "esphome/core/log.h"
#include "esphome/core/version.h"

#include <esp_heap_caps.h>
#include <lwip/sockets.h>

namespace esphome {
namespace slider_lab {

static const char *const TAG = "slider_lab";

namespace {

// ---- hardware constants (fixed per spec, docs/lab-rig-spec.md R2/R3/R8) ----
constexpr uint32_t kTickUs = 1000;             // R1: 1 kHz
constexpr int kAdcSamples = 4;                 // R2: 4 oneshot samples per tick
constexpr uint32_t kPwmFreqHz = 19531;         // R3
constexpr ledc_timer_bit_t kPwmRes = LEDC_TIMER_12_BIT;  // >=10 bit required
constexpr uint32_t kPwmFull = 1u << 12;        // LEDC duty for solid 100 %
// provisional legacy endpoints (docs/legacy-analysis.md): raw 8000 -> 0 %,
// raw 100 -> 100 % — wiring is inverted
constexpr float kRaw0 = 8000.0f;
constexpr float kRaw100 = 100.0f;
// R4 interlock constants
constexpr float kEndzoneLo = 3.0f;   // %
constexpr float kEndzoneHi = 97.0f;  // %
constexpr float kThermalLimit = 0.5f;  // 10 s moving average of |duty|
constexpr uint32_t kMaxBudgetMs = 900000;  // absolute sanity cap (15 min)
// recorder
constexpr uint32_t kPostTailMs = 500;  // keep recording this long after program end
// R6 dump-session guards (dump-latch fix): a client that stalls or vanishes
// mid-download must never hold dump_active_ (and with it run_experiment)
// hostage. (1) explicit per-send timeout: a send that makes no progress fails
// after a few seconds instead of relying on httpd/lwIP defaults; (2) hard
// session cap: even a slowly-trickling client cannot hold the dump open
// forever (a full 75k-record dump takes ~9 s at wire speed).
constexpr int kDumpSendTimeoutS = 3;
constexpr int64_t kDumpSessionCapUs = 30LL * 1000000;
// R7 repositioning helper (NOT-A-CONTROLLER, see seg_goto below)
// Bench bracket 2026-07-16: 0.55 only creeps (~0.2 %/4 s), 0.7 flies — the
// legacy breakaway 0.42 is wrong. 0.72 repositions reliably; per-run override
// via the shared experiment param "goto_duty".
constexpr float kGotoDuty = 0.72f;   // default bang-bang duty
constexpr float kGotoTol = 2.0f;     // % — generous by design
constexpr uint32_t kGotoMaxMs = 4000;
// program sanity caps
constexpr size_t kMaxSegments = 1024;

// ---- segment builders (pure helpers, no state) ----
Segment seg_hold(float duty, uint32_t ms) {
  Segment s;
  s.type = SegType::HOLD;
  s.duration_ms = ms;
  s.a = duty;
  return s;
}
Segment seg_hold_last(uint32_t ms) {
  Segment s;
  s.type = SegType::HOLD;
  s.duration_ms = ms;
  s.hold_last = true;
  return s;
}
Segment seg_ramp(float d0, float d1, uint32_t ms) {
  Segment s;
  s.type = SegType::RAMP;
  s.duration_ms = ms;
  s.a = d0;
  s.b = d1;
  return s;
}
Segment seg_pulse(float duty, uint32_t width_ms, uint32_t total_ms) {
  Segment s;
  s.type = SegType::PULSE;
  s.duration_ms = std::max(total_ms, width_ms);
  s.a = duty;
  s.b = (float) width_ms;
  return s;
}
// the spec's dither(f, A, bias) segment: HOLD(bias) + superimposed square dither
Segment seg_dither(float f, float amp, float bias, uint32_t ms) {
  Segment s = seg_hold(bias, ms);
  s.dither_f = f;
  s.dither_a = amp;
  return s;
}
Segment seg_breakaway(float step_signed, uint32_t interval_ms, float thresh_pct, uint32_t max_ms,
                      float dither_f = 0.0f, float dither_a = 0.0f) {
  Segment s;
  s.type = SegType::BREAKAWAY;
  s.duration_ms = max_ms;
  s.a = step_signed;
  s.b = (float) interval_ms;
  s.c = thresh_pct;
  s.dither_f = dither_f;
  s.dither_a = dither_a;
  return s;
}
Segment seg_settle(uint32_t ms) {
  Segment s;
  s.type = SegType::SETTLE;
  s.duration_ms = ms;
  return s;
}
// =============================================================================
// NOT-A-CONTROLLER. NOT-A-CONTROLLER. NOT-A-CONTROLLER.
// R7 repositioning helper: naive bang-bang at fixed duty with a generous
// tolerance and its own timeout. Test infrastructure only — used *between*
// program segments so grid experiments (M4/M7) can run unattended. Its ticks
// are flagged FLAG_GOTO and must be dropped by the analysis. Nothing about its
// behavior is representative of the future servo, and it must never be
// "improved" into one.
// =============================================================================
Segment seg_goto(float target_pct, float duty = kGotoDuty, uint32_t max_ms = kGotoMaxMs) {
  Segment s;
  s.type = SegType::GOTO;
  s.duration_ms = max_ms;
  s.a = target_pct;
  s.b = duty;
  s.c = kGotoTol;
  return s;
}

float clampf(float v, float lo, float hi) { return std::min(std::max(v, lo), hi); }

}  // namespace

// ============================================================================
// setup / teardown
// ============================================================================

void SliderLab::setup() {
  // --- scope-sync pin (R8): LOW when idle ---
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << this->sync_pin_;
  // INPUT_OUTPUT so the driven level can be read back (scope-less sync check)
  io.mode = GPIO_MODE_INPUT_OUTPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io);
  gpio_set_level((gpio_num_t) this->sync_pin_, 0);

  // --- LEDC 19531 Hz, 12-bit (R3); boot state: both outputs 0 = coast (R4) ---
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode = LEDC_LOW_SPEED_MODE;
  tcfg.duty_resolution = kPwmRes;
  tcfg.timer_num = LEDC_TIMER_0;
  tcfg.freq_hz = kPwmFreqHz;
  tcfg.clk_cfg = LEDC_AUTO_CLK;  // 19531 Hz * 4096 needs the 80 MHz source; AUTO picks it
  if (ledc_timer_config(&tcfg) != ESP_OK) {
    ESP_LOGE(TAG, "LEDC timer config failed");
    this->mark_failed();
    return;
  }
  const uint8_t pins[2] = {this->ain1_pin_, this->ain2_pin_};
  for (int i = 0; i < 2; i++) {
    ledc_channel_config_t ccfg = {};
    ccfg.gpio_num = pins[i];
    ccfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ccfg.channel = (i == 0) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;
    ccfg.intr_type = LEDC_INTR_DISABLE;
    ccfg.timer_sel = LEDC_TIMER_0;
    ccfg.duty = 0;
    ccfg.hpoint = 0;
    if (ledc_channel_config(&ccfg) != ESP_OK) {
      ESP_LOGE(TAG, "LEDC channel %d config failed", i);
      this->mark_failed();
      return;
    }
    this->ledc_last_[i] = 0;
  }

  // --- ADC oneshot, 13-bit, ATTEN_DB_12 (R2) ---
  adc_oneshot_unit_init_cfg_t ucfg = {};
  ucfg.unit_id = ADC_UNIT_1;
  ucfg.ulp_mode = ADC_ULP_MODE_DISABLE;
  if (adc_oneshot_new_unit(&ucfg, &this->adc_) != ESP_OK) {
    ESP_LOGE(TAG, "ADC oneshot unit init failed");
    this->mark_failed();
    return;
  }
  adc_oneshot_chan_cfg_t chcfg = {};
  chcfg.atten = ADC_ATTEN_DB_12;
  chcfg.bitwidth = ADC_BITWIDTH_13;
  if (adc_oneshot_config_channel(this->adc_, (adc_channel_t) this->adc_channel_, &chcfg) != ESP_OK) {
    ESP_LOGE(TAG, "ADC channel %u config failed", this->adc_channel_);
    this->mark_failed();
    return;
  }

  // --- recorder buffer (R6): PSRAM first, descending ladder; internal fallback ---
  const uint32_t psram_sizes[] = {75000, 60000, 45000, 30000};
  for (uint32_t n : psram_sizes) {
    this->buf_ = (Record *) heap_caps_malloc(n * sizeof(Record), MALLOC_CAP_SPIRAM);
    if (this->buf_ != nullptr) {
      this->capacity_ = n;
      this->buf_in_psram_ = true;
      break;
    }
  }
  if (this->buf_ == nullptr) {
    ESP_LOGW(TAG, "PSRAM allocation failed - falling back to internal RAM (short capture!)");
    const uint32_t int_sizes[] = {12000, 8000, 5000, 2000};
    for (uint32_t n : int_sizes) {
      this->buf_ = (Record *) heap_caps_malloc(n * sizeof(Record), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      if (this->buf_ != nullptr) {
        this->capacity_ = n;
        this->buf_in_psram_ = false;
        break;
      }
    }
  }
  if (this->buf_ == nullptr) {
    ESP_LOGE(TAG, "recorder buffer allocation failed entirely");
    this->mark_failed();
    return;
  }

  // --- HTTP dump endpoint (R6) ---
  httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
  hcfg.server_port = this->http_port_;
  hcfg.stack_size = 12288;
  hcfg.max_uri_handlers = 2;
  if (httpd_start(&this->httpd_, &hcfg) == ESP_OK) {
    httpd_uri_t uri = {};
    uri.uri = "/lab/dump.csv";
    uri.method = HTTP_GET;
    uri.handler = SliderLab::dump_handler_;
    uri.user_ctx = this;
    httpd_register_uri_handler(this->httpd_, &uri);
  } else {
    ESP_LOGE(TAG, "esp_http_server failed to start on port %u", this->http_port_);
    // not fatal for the tick/interlocks — keep running, dump unavailable
  }

  // --- deterministic 1 kHz tick (R1) ---
  // Hardware GPTimer ISR -> task notification -> dedicated tick task at
  // priority 24, ABOVE the IDF WiFi task (23). The previous esp_timer
  // dispatch ran at task priority 22 and lost ~26 ticks/s to WiFi RX/beacon
  // processing even on a quiet system. The ISR does nothing but notify.
  if (xTaskCreate(SliderLab::tick_task_trampoline_, "slider_lab_tick", 4096, this, 24, &this->tick_task_) != pdPASS) {
    ESP_LOGE(TAG, "tick task creation failed");
    this->mark_failed();
    return;
  }
  gptimer_config_t gcfg = {};
  gcfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  gcfg.direction = GPTIMER_COUNT_UP;
  gcfg.resolution_hz = 1000000;  // 1 µs resolution
  gptimer_event_callbacks_t cbs = {};
  cbs.on_alarm = SliderLab::timer_isr_;
  gptimer_alarm_config_t acfg = {};
  acfg.alarm_count = kTickUs;  // 1000 µs -> 1 kHz
  acfg.reload_count = 0;
  acfg.flags.auto_reload_on_alarm = true;
  if (gptimer_new_timer(&gcfg, &this->gptimer_) != ESP_OK ||
      gptimer_register_event_callbacks(this->gptimer_, &cbs, this) != ESP_OK ||
      gptimer_set_alarm_action(this->gptimer_, &acfg) != ESP_OK || gptimer_enable(this->gptimer_) != ESP_OK ||
      gptimer_start(this->gptimer_) != ESP_OK) {
    ESP_LOGE(TAG, "GPTimer start failed");
    this->mark_failed();
    return;
  }
}

void SliderLab::dump_config() {
  ESP_LOGCONFIG(TAG, "slider_lab (Phase-1 identification instrument):");
  ESP_LOGCONFIG(TAG, "  ADC1 ch%u, 13-bit, ATTEN_DB_12, %d samples/tick", this->adc_channel_, kAdcSamples);
  ESP_LOGCONFIG(TAG, "  DRV8833 AIN1=GPIO%u AIN2=GPIO%u, LEDC %u Hz, 12-bit", this->ain1_pin_, this->ain2_pin_,
                (unsigned) kPwmFreqHz);
  ESP_LOGCONFIG(TAG, "  scope sync: GPIO%u", this->sync_pin_);
  ESP_LOGCONFIG(TAG, "  invert_direction: %s", this->invert_direction_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  recorder: %u records (%.1f s) in %s", (unsigned) this->capacity_, this->capacity_ / 1000.0f,
                this->buf_in_psram_ ? "PSRAM" : "internal RAM (fallback!)");
  ESP_LOGCONFIG(TAG, "  dump: http://<ip>:%u/lab/dump.csv", this->http_port_);
}

// ============================================================================
// 1 kHz tick (dedicated task, prio 24) — R1: sample, interlocks, player,
// apply, record. NOTHING in this path may block: no logging, no allocation,
// no flash access — deferred-event bits carry messages to loop().
// ============================================================================

bool IRAM_ATTR SliderLab::timer_isr_(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg) {
  // ISR: nothing but waking the tick task
  auto *self = static_cast<SliderLab *>(arg);
  BaseType_t hp_woken = pdFALSE;
  vTaskNotifyGiveFromISR(self->tick_task_, &hp_woken);
  return hp_woken == pdTRUE;
}

void SliderLab::tick_task_trampoline_(void *arg) {
  auto *self = static_cast<SliderLab *>(arg);
  while (true) {
    // clear-on-exit: if the task ever runs late, missed notifications collapse
    // into one tick — a gap in the t series is honest, a catch-up burst of
    // duplicate ticks would fake the timeline
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    self->tick_();
  }
}

float SliderLab::raw_to_percent_(float raw) const {
  return (kRaw0 - raw) / (kRaw0 - kRaw100) * 100.0f;
}

float SliderLab::thermal_avg_() const {
  // th_total_ sums |duty_milli| over 10000 ticks -> average duty in [0,1]
  return (float) this->th_total_ * 1e-7f;
}

void SliderLab::tick_() {
  const int64_t t_entry = esp_timer_get_time();

  // 1) sample ADC — 4 oneshot reads, raw, unfiltered (R2)
  uint16_t s[kAdcSamples] = {0};
  int32_t sum = 0;
  for (int i = 0; i < kAdcSamples; i++) {
    int raw = 0;
    adc_oneshot_read(this->adc_, (adc_channel_t) this->adc_channel_, &raw);
    s[i] = (uint16_t) raw;
    sum += raw;
  }
  const float adc_mean = (float) sum / (float) kAdcSamples;
  const float pos = this->raw_to_percent_(adc_mean);
  this->pos_percent_.store(pos);

  // 2) adopt a pending program (handoff from the main task);
  //    queued while a dump is streaming so the dump stays consistent
  if (this->state_.load() == State::IDLE && !this->dump_active_.load()) {
    Program *p = this->pending_.exchange(nullptr);
    if (p != nullptr)
      this->start_run_(p, t_entry);
  }

  // 3) evaluate the experiment player
  uint8_t flags = 0;
  float torque = 0.0f;
  bool active_drive = false;
  if (this->state_.load() == State::RUNNING)
    torque = this->player_eval_(pos, flags, active_drive);

  // 4) safety interlocks (R4) — enforced HERE, never in the player
  if (this->state_.load() == State::RUNNING) {
    if (this->abort_req_.load()) {
      // (d) HA abort switch -> torque 0 within one tick
      flags |= FLAG_ABORT;
      this->events_.fetch_or(EV_ABORT_SWITCH);
      this->finish_run_(true);
      torque = 0.0f;
      active_drive = false;
    } else if ((uint32_t) ((t_entry - this->run_t0_us_) / 1000) > this->active_->budget_ms) {
      // (b) per-program wall-clock budget
      flags |= FLAG_ABORT;
      this->events_.fetch_or(EV_BUDGET);
      this->finish_run_(true);
      torque = 0.0f;
      active_drive = false;
    } else if (this->thermal_avg_() > kThermalLimit) {
      // (c) thermal budget: 10 s moving average of |duty| must stay <= 0.5
      flags |= FLAG_THERMAL | FLAG_ABORT;
      this->events_.fetch_or(EV_THERMAL);
      this->finish_run_(true);
      torque = 0.0f;
      active_drive = false;
    } else if ((pos < kEndzoneLo || pos > kEndzoneHi) && torque != 0.0f) {
      // (a) end-zone marker — PASSIVE (Jans Entscheid 2026-07-17). The former
      // direction-agnostic torque cut never prevented an impact (momentum
      // arrives with torque already 0) but trapped the slider behind the
      // boundary; budget (b) and thermal (c) bound sustained grinding into a
      // stop. Flag and event stay as data markers only.
      flags |= FLAG_ENDZONE;
      if (!this->endzone_flagged_) {
        this->endzone_flagged_ = true;
        this->events_.fetch_or(EV_ENDZONE);
      }
    }
  } else {
    // (e) idle/tail/boot: torque 0
    torque = 0.0f;
    active_drive = false;
  }

  // 5) apply torque -> the two LEDC duties (R3)
  const DecayMode mode = (this->active_ != nullptr) ? this->active_->mode : DecayMode::FAST;
  const int16_t duty_milli = this->apply_torque_(torque, mode, active_drive);

  // thermal bookkeeping: |applied duty|, 100 ms buckets over a 10 s window
  const uint32_t mag = (uint32_t) ((duty_milli < 0) ? -duty_milli : duty_milli);
  this->th_bucket_[this->th_idx_] += mag;
  this->th_total_ += mag;
  if (++this->th_tick_in_bucket_ >= 100) {
    this->th_tick_in_bucket_ = 0;
    this->th_idx_ = (this->th_idx_ + 1) % 100;
    this->th_total_ -= this->th_bucket_[this->th_idx_];
    this->th_bucket_[this->th_idx_] = 0;
  }

  // 6) scope sync (R8): HIGH while a program runs, 1 ms LOW blip at segment
  //    boundaries, LOW when idle (and during the post-run tail)
  int sync_level = 0;
  if (this->state_.load() == State::RUNNING) {
    if (this->sync_low_ticks_ > 0) {
      this->sync_low_ticks_--;
      sync_level = 0;
    } else {
      sync_level = 1;
    }
  }
  gpio_set_level((gpio_num_t) this->sync_pin_, sync_level);

  // 7) write one record (R6) — raw data only, no filtering
  if (this->recording_) {
    const uint32_t n = this->count_.load();
    if (n < this->capacity_) {
      Record &r = this->buf_[n];
      r.t_us = (uint32_t) (t_entry - this->run_t0_us_);
      r.adc_mean = adc_mean;
      r.duty_milli = duty_milli;
      r.seg = this->rec_seg_;
      r.flags = flags;
      for (int i = 0; i < kAdcSamples; i++)
        r.s[i] = s[i];
      // tick execution time up to this point (ADC + player + interlocks + apply)
      r.tick_us = (uint16_t) (esp_timer_get_time() - t_entry);
      this->count_.store(n + 1);
    } else if (!this->recorder_full_.load()) {
      this->recorder_full_.store(true);
      this->events_.fetch_or(EV_FULL);
    }
  }

  // 8) post-run tail: keep recording kPostTailMs after the program ended
  if (this->state_.load() == State::TAIL) {
    if (this->tail_ticks_ > 0)
      this->tail_ticks_--;
    if (this->tail_ticks_ == 0) {
      this->recording_ = false;
      Program *done = this->active_;
      this->active_ = nullptr;
      this->state_.store(State::IDLE);
      this->finished_.store(done);
      this->events_.fetch_or(EV_DONE);
    }
  }

  this->last_tick_us_.store((uint16_t) (esp_timer_get_time() - t_entry));
}

void SliderLab::start_run_(Program *p, int64_t now_us) {
  this->active_ = p;
  this->seg_idx_ = 0;
  this->seg_tick_ = 0;
  this->rec_seg_ = 0;
  this->seg_idx_pub_.store(0);
  this->run_t0_us_ = now_us;
  this->bk_duty_ = 0.0f;
  this->bk_ref_pos_ = 0.0f;
  this->dither_phase_ = 0.0f;
  this->seg_final_torque_ = 0.0f;
  this->sync_low_ticks_ = 0;
  this->run_aborted_ = false;
  this->endzone_flagged_ = false;
  this->count_.store(0);
  this->recorder_full_.store(false);
  this->recording_ = true;
  this->state_.store(State::RUNNING);
}

void SliderLab::finish_run_(bool aborted) {
  this->run_aborted_ = aborted;
  if (this->active_ != nullptr)
    this->rec_seg_ = (uint16_t) this->active_->segments.size();
  this->state_.store(State::TAIL);
  this->tail_ticks_ = kPostTailMs;
  // sync pin falls LOW automatically (state != RUNNING)
}

// ============================================================================
// experiment player (R5) — a tiny interpreter over torque-profile segments.
// It commands torque only; it never enforces safety (that is the tick's job).
// ============================================================================

float SliderLab::player_eval_(float pos, uint8_t &flags, bool &active_drive) {
  Program &pr = *this->active_;
  Segment &sg = pr.segments[this->seg_idx_];
  this->rec_seg_ = (uint16_t) this->seg_idx_;
  this->seg_idx_pub_.store(this->seg_idx_);

  if (this->seg_tick_ == 0) {
    // segment entry
    flags |= FLAG_SEG_START;
    if (this->seg_idx_ > 0)
      this->sync_low_ticks_ = 1;  // R8: 1 ms LOW blip at every segment boundary
    this->bk_ref_pos_ = pos;
    this->bk_duty_ = 0.0f;
    this->dither_phase_ = 0.0f;
  }

  const uint32_t dur = std::max<uint32_t>(sg.duration_ms, 1);
  float base = 0.0f;
  bool done = false;
  active_drive = false;

  switch (sg.type) {
    case SegType::HOLD:
      base = sg.hold_last ? this->seg_final_torque_ : sg.a;
      active_drive = true;
      break;

    case SegType::RAMP: {
      const float f = (float) this->seg_tick_ / (float) dur;
      base = sg.a + (sg.b - sg.a) * f;
      active_drive = true;
      break;
    }

    case SegType::PULSE:
      if (this->seg_tick_ < (uint32_t) sg.b) {
        base = sg.a;
        active_drive = true;
      }
      // off-period: torque 0, hard-off
      break;

    case SegType::BREAKAWAY: {
      // self-terminating stiction probe: +step every interval until the raw
      // position moves more than the threshold (M4/M8)
      const uint32_t interval = std::max<uint32_t>((uint32_t) sg.b, 1);
      if (this->seg_tick_ % interval == 0)
        this->bk_duty_ += sg.a;  // first step right at entry
      base = this->bk_duty_;
      active_drive = true;
      if (std::fabs(pos - this->bk_ref_pos_) > sg.c)
        done = true;
      break;
    }

    case SegType::SETTLE:
      // torque 0 (hard-off), keep recording
      break;

    case SegType::GOTO: {
      // NOT-A-CONTROLLER (R7). Naive bang-bang repositioning between
      // measurement segments. These ticks carry FLAG_GOTO and are not
      // experiment data. Do not extend, tune, or reuse this.
      flags |= FLAG_GOTO;
      const float err = sg.a - pos;
      if (std::fabs(err) <= sg.c) {
        done = true;  // close enough — tolerance is generous by design
      } else {
        base = (err > 0.0f) ? sg.b : -sg.b;
        active_drive = true;
      }
      break;
    }
  }

  // superimposed square-wave dither (phase accumulator; 1 kHz tick)
  float out = base;
  if (sg.dither_f > 0.0f && active_drive) {
    this->dither_phase_ += sg.dither_f * 0.001f;
    if (this->dither_phase_ >= 1.0f)
      this->dither_phase_ -= 1.0f;
    out += (this->dither_phase_ < 0.5f) ? sg.dither_a : -sg.dither_a;
  }

  this->seg_tick_++;
  if (this->seg_tick_ >= dur) {
    if (sg.type == SegType::GOTO && std::fabs(sg.a - pos) > sg.c)
      this->events_.fetch_or(EV_GOTO_TIMEOUT);
    done = true;
  }
  if (done) {
    this->seg_final_torque_ = base;  // pre-dither, for hold_last (M4 sustain check)
    this->seg_idx_++;
    this->seg_tick_ = 0;
    if (this->seg_idx_ >= pr.segments.size())
      this->finish_run_(false);
  }
  return out;
}

// ============================================================================
// torque -> DRV8833 via LEDC (R3)
// ============================================================================

int16_t SliderLab::apply_torque_(float torque, DecayMode mode, bool active_drive) {
  float t = clampf(torque, -1.0f, 1.0f);
  if (!active_drive)
    t = 0.0f;
  const float phys = this->invert_direction_ ? -t : t;

  auto duty_of = [](float x) -> uint32_t { return (uint32_t) lrintf(clampf(x, 0.0f, 1.0f) * (float) kPwmFull); };

  uint32_t a1 = 0, a2 = 0;
  if (!active_drive) {
    // hard-off: both low -> coast, silent (boot / idle / settle / interlock cut)
    a1 = 0;
    a2 = 0;
  } else if (mode == DecayMode::FAST) {
    // FAST decay (coast off-phase): fwd AIN1=PWM(d), AIN2=0; rev mirrored.
    // torque 0 in FAST: both 0 (coast + silent).
    if (phys > 0.0f) {
      a1 = duty_of(phys);
    } else if (phys < 0.0f) {
      a2 = duty_of(-phys);
    }
  } else {
    // SLOW decay (brake off-phase): fwd AIN1=100 %, AIN2=PWM(1-d);
    // rev AIN2=100 %, AIN1=PWM(1+d). d=0 -> both 100 % = brake.
    if (phys >= 0.0f) {
      a1 = kPwmFull;
      a2 = duty_of(1.0f - phys);
    } else {
      a2 = kPwmFull;
      a1 = duty_of(1.0f + phys);
    }
  }
  this->set_ledc_(0, a1);
  this->set_ledc_(1, a2);
  return (int16_t) lrintf(t * 1000.0f);
}

void SliderLab::set_ledc_(uint8_t idx, uint32_t duty) {
  if (this->ledc_last_[idx] == duty)
    return;
  this->ledc_last_[idx] = duty;
  const ledc_channel_t ch = (idx == 0) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

// ============================================================================
// main task: run_experiment + program builders + deferred logging
// ============================================================================

bool SliderLab::run_experiment(const std::string &name, const std::string &params_json) {
  if (this->is_failed() || this->buf_ == nullptr) {
    ESP_LOGW(TAG, "run_experiment('%s'): component failed / no buffer", name.c_str());
    return false;
  }
  if (this->player_busy_()) {
    ESP_LOGW(TAG, "run_experiment('%s'): busy (state=%s)", name.c_str(), this->get_player_state().c_str());
    return false;
  }
  if (this->dump_active_.load()) {
    ESP_LOGW(TAG, "run_experiment('%s'): CSV dump in progress, try again", name.c_str());
    return false;
  }
  if (this->abort_req_.load()) {
    ESP_LOGW(TAG, "run_experiment('%s'): abort switch is ON", name.c_str());
    return false;
  }

  auto *prog = new Program();
  prog->name = name;
  prog->params_json = params_json.empty() ? "{}" : params_json;

  bool ok;
  if (params_json.empty() || params_json == "{}") {
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    ok = this->build_program_(name, root, *prog);
  } else {
    ok = json::parse_json(params_json, [this, &name, prog](JsonObject root) -> bool {
      return this->build_program_(name, root, *prog);
    });
  }

  if (!ok || prog->segments.empty() || prog->segments.size() > kMaxSegments ||
      prog->budget_ms > kMaxBudgetMs) {
    ESP_LOGW(TAG, "run_experiment('%s') rejected (unknown name, bad params, or too large)", name.c_str());
    delete prog;
    return false;
  }

  prog->raw_samples = prog->raw_samples || this->raw_mode_.load();
  if (prog->endstop_safe)
    ESP_LOGI(TAG, "'%s' carries endstop_safe — obsolete no-op, the end-zone cut is a passive marker now", name.c_str());

  // meta for the dump header — safe to write here: player idle, dump refused while busy
  this->run_name_ = prog->name;
  this->run_params_ = prog->params_json;
  this->run_raw_samples_ = prog->raw_samples;

  ESP_LOGI(TAG, "starting '%s': %u segments, budget %.1f s, mode=%s, raw_samples=%d, params=%s", name.c_str(),
           (unsigned) prog->segments.size(), prog->budget_ms / 1000.0f,
           prog->mode == DecayMode::SLOW ? "slow" : "fast", (int) prog->raw_samples, prog->params_json.c_str());

  Program *old = this->pending_.exchange(prog);
  delete old;  // cannot normally happen (busy check) — belt and braces
  return true;
}

bool SliderLab::build_program_(const std::string &name, JsonObject p, Program &out) {
  // shared param helpers (lambdas, no static state)
  auto dir_sign = [&p]() -> float {
    const char *d = p["dir"] | "fwd";
    return (strcmp(d, "bwd") == 0 || strcmp(d, "rev") == 0) ? -1.0f : 1.0f;
  };
  auto dir_list = [&p]() -> std::vector<float> {
    const char *d = p["dir"] | "both";
    if (strcmp(d, "fwd") == 0)
      return {1.0f};
    if (strcmp(d, "bwd") == 0 || strcmp(d, "rev") == 0)
      return {-1.0f};
    return {1.0f, -1.0f};
  };
  auto float_list = [&p](const char *key, std::vector<float> def) -> std::vector<float> {
    if (p[key].is<JsonArray>()) {
      def.clear();
      JsonArray arr = p[key].as<JsonArray>();
      for (JsonVariant v : arr)
        def.push_back(v.as<float>());
    }
    return def;
  };

  out.mode = (strcmp((const char *) (p["mode"] | "fast"), "slow") == 0) ? DecayMode::SLOW : DecayMode::FAST;
  out.raw_samples = p["raw"] | false;
  out.endstop_safe = p["endstop_safe"] | false;
  // R7: shared override for the goto helper's bang-bang duty (repositioning
  // only, NOT-A-CONTROLLER). Too low just means goto times out harmlessly.
  const float goto_duty = clampf(p["goto_duty"] | kGotoDuty, 0.0f, 0.9f);
  auto &sg = out.segments;

  if (name == "noise_floor") {
    // M1: motor unpowered, record raw ADC (individual samples on)
    const uint32_t ms = (uint32_t) ((p["duration_s"] | 10.0f) * 1000.0f);
    out.raw_samples = true;
    sg.push_back(seg_settle(ms));

  } else if (name == "pwm_coupling") {
    // M2: stalled low duty, phase-locked EMI hunt. Duty clamped <= 0.5 so the
    // slider stays below breakaway and does not move (bench 2026-07-16:
    // 0.55 only creeps, 0.7 flies — the legacy 0.42 was wrong; M2 needs
    // near-stall currents just under the real breakaway).
    const float duty = clampf(p["duty"] | 0.2f, 0.0f, 0.5f);
    const uint32_t ms = (uint32_t) ((p["duration_s"] | 10.0f) * 1000.0f);
    out.raw_samples = true;
    sg.push_back(seg_settle(300));
    sg.push_back(seg_hold(dir_sign() * duty, ms));
    sg.push_back(seg_settle(300));

  } else if (name == "duty_ladder") {
    // M3: torque-vs-duty linearity. 0.5 s settles between rungs keep the 10 s
    // |duty| average under the 0.5 thermal budget and the stall periods short.
    const float d0 = p["d0"] | 0.05f;
    const float d1 = p["d1"] | 0.6f;
    const float step = p["step"] | 0.05f;
    const uint32_t step_ms = p["step_ms"] | 2000;
    const uint32_t pause_ms = p["pause_ms"] | 500;
    const float s = dir_sign();
    if (step <= 0.0f || d1 < d0)
      return false;
    for (float d = d0; d <= d1 + 1e-6f; d += step) {
      sg.push_back(seg_hold(s * d, step_ms));
      sg.push_back(seg_settle(pause_ms));
    }

  } else if (name == "breakaway_grid") {
    // M4: stiction map. Default grid (5 positions x both dirs x 1 repeat) fits
    // the recorder; run the full 10x2x3 campaign as parameterized chunks.
    const std::vector<float> positions = float_list("positions", {5.0f, 27.5f, 50.0f, 72.5f, 95.0f});
    const int repeats = p["repeats"] | 1;
    const float step = p["step"] | 0.005f;
    const uint32_t interval = p["interval_ms"] | 20;
    const float thresh = p["move_threshold"] | 0.3f;
    const uint32_t hold_ms = p["hold_ms"] | 300;  // post-breakaway sustain check (Stribeck)
    for (float pos : positions) {
      for (int r = 0; r < repeats; r++) {
        for (float s : dir_list()) {
          sg.push_back(seg_goto(pos, goto_duty));
          sg.push_back(seg_settle(500));
          sg.push_back(seg_breakaway(s * step, interval, thresh, 3000));
          sg.push_back(seg_hold_last(hold_ms));  // does motion sustain?
          sg.push_back(seg_settle(300));
        }
      }
    }

  } else if (name == "step_response") {
    // M5: duty step from standstill at 20 % (fwd) / 80 % (bwd); the end-zone
    // interlock cuts torque at the far end — that cut is part of the data.
    const float duty = p["duty"] | 0.7f;
    const float s = dir_sign();
    const float start = p["start"] | ((s > 0) ? 20.0f : 80.0f);
    const uint32_t hold_ms = p["hold_ms"] | 5000;
    sg.push_back(seg_goto(start, goto_duty));
    sg.push_back(seg_settle(500));
    sg.push_back(seg_hold(s * duty, hold_ms));
    sg.push_back(seg_settle(500));

  } else if (name == "reversal") {
    // M6: backlash width. Creep in approach direction, settle, command the
    // reverse just above breakaway; pot dead time = gap traversal.
    const float duty = p["duty"] | 0.45f;
    const float approach = p["approach_duty"] | 0.5f;
    const float pos = p["position"] | 50.0f;
    const int cycles = p["cycles"] | 3;
    const float s = dir_sign();
    for (int i = 0; i < cycles; i++) {
      sg.push_back(seg_goto(pos, goto_duty));
      sg.push_back(seg_settle(500));
      sg.push_back(seg_hold(s * approach, 600));
      sg.push_back(seg_settle(600));
      sg.push_back(seg_hold(-s * duty, 1200));
      sg.push_back(seg_settle(500));
    }

  } else if (name == "impulse_ladder") {
    // M7: endgame design data. One position per run (default 50 %); the full
    // campaign is 3 runs (25/50/75). cold_reversal=true fires each pulse right
    // after a small opposite move (variant B — backlash eats pulse energy).
    const float pos = p["position"] | 50.0f;
    const float duty = p["duty"] | 0.6f;
    const std::vector<float> widths = float_list("durations_ms", {1, 2, 3, 5, 8, 12, 20});
    const int repeats = p["repeats"] | 5;
    const uint32_t settle_ms = p["settle_ms"] | 300;
    const int recenter_every = p["recenter_every"] | 5;
    const bool cold = p["cold_reversal"] | false;
    sg.push_back(seg_goto(pos, goto_duty));
    sg.push_back(seg_settle(500));
    int shots = 0;
    for (float s : dir_list()) {
      for (float w : widths) {
        for (int r = 0; r < repeats; r++) {
          if (shots > 0 && recenter_every > 0 && (shots % recenter_every) == 0) {
            sg.push_back(seg_goto(pos, goto_duty));
            sg.push_back(seg_settle(300));
          }
          if (cold) {
            sg.push_back(seg_pulse(-s * duty, 8, 8));
            sg.push_back(seg_settle(200));
          }
          sg.push_back(seg_pulse(s * duty, (uint32_t) w, (uint32_t) w));
          sg.push_back(seg_settle(settle_ms));
          shots++;
        }
      }
    }

  } else if (name == "dither_grid") {
    // M8: one (f, A) point per run — the ear test in a quiet room is the
    // binding instrument, so runs are short and single-point. baseline=true
    // prepends an undithered breakaway ramp for direct comparison.
    const float f = p["freq"] | 100.0f;
    const float amp = p["amp"] | 0.1f;
    const float pos = p["position"] | 50.0f;
    const float step = p["step"] | 0.005f;
    const uint32_t interval = p["interval_ms"] | 20;
    const float thresh = p["move_threshold"] | 0.3f;
    const bool baseline = p["baseline"] | true;
    const float s = dir_sign();
    sg.push_back(seg_goto(pos, goto_duty));
    sg.push_back(seg_settle(500));
    if (baseline) {
      sg.push_back(seg_breakaway(s * step, interval, thresh, 3000));
      sg.push_back(seg_hold_last(300));
      sg.push_back(seg_settle(500));
      sg.push_back(seg_goto(pos, goto_duty));
      sg.push_back(seg_settle(500));
    }
    sg.push_back(seg_breakaway(s * step, interval, thresh, 3000, f, amp));
    sg.push_back(seg_hold_last(300));
    sg.push_back(seg_settle(500));

  } else if (name == "triangle") {
    // M5 companion: slow triangular duty ramp 0 -> peak -> 0 (quasi-static)
    const float peak = p["peak"] | 0.6f;
    const uint32_t ramp_ms = p["ramp_ms"] | 4000;
    const float s = dir_sign();
    const float start = p["start"] | ((s > 0) ? 20.0f : 80.0f);
    sg.push_back(seg_goto(start, goto_duty));
    sg.push_back(seg_settle(500));
    sg.push_back(seg_ramp(0.0f, s * peak, ramp_ms));
    sg.push_back(seg_ramp(s * peak, 0.0f, ramp_ms));
    sg.push_back(seg_settle(500));

  } else if (name == "dither") {
    // bare dither segment (R5): hold bias with square dither superimposed
    const float f = p["freq"] | 100.0f;
    const float amp = p["amp"] | 0.1f;
    const float bias = p["bias"] | 0.0f;
    const uint32_t ms = (uint32_t) ((p["duration_s"] | 5.0f) * 1000.0f);
    sg.push_back(seg_settle(300));
    sg.push_back(seg_dither(f, amp, bias, ms));
    sg.push_back(seg_settle(300));

  } else {
    return false;  // unknown program
  }

  // R4 wall-clock budget: planned duration + 25 % + 5 s, floor at the 30 s default
  uint64_t planned = 0;
  for (const Segment &seg : out.segments)
    planned += seg.duration_ms;
  uint64_t budget = planned + planned / 4 + 5000;
  out.budget_ms = (uint32_t) std::max<uint64_t>(std::min<uint64_t>(budget, kMaxBudgetMs), 30000);
  return true;
}

std::string SliderLab::get_player_state() const {
  switch (this->state_.load()) {
    case State::RUNNING:
      return "running:" + this->run_name_ + ":" + std::to_string(this->seg_idx_pub_.load());
    case State::TAIL:
      return "settling:" + this->run_name_;
    case State::IDLE:
    default:
      return "idle";
  }
}

void SliderLab::loop() {
  const uint32_t ev = this->events_.exchange(0);
  if (ev & EV_ABORT_SWITCH)
    ESP_LOGW(TAG, "run aborted: HA abort switch");
  if (ev & EV_BUDGET)
    ESP_LOGW(TAG, "run aborted: wall-clock budget exceeded (R4)");
  if (ev & EV_THERMAL)
    ESP_LOGW(TAG, "run aborted: thermal budget — 10 s avg |duty| > 0.5 (R4)");
  if (ev & EV_ENDZONE)
    ESP_LOGW(TAG, "end-zone interlock cut torque (position outside [3, 97] %%)");
  if (ev & EV_GOTO_TIMEOUT)
    ESP_LOGW(TAG, "goto timed out before reaching target (wrong invert_direction? slider blocked?)");
  if (ev & EV_FULL)
    ESP_LOGW(TAG, "recorder full — later ticks of this run are not recorded");
  if (ev & EV_DONE) {
    ESP_LOGI(TAG, "experiment '%s' finished%s: %u records (%.1f s) — dump: /lab/dump.csv", this->run_name_.c_str(),
             this->run_aborted_ ? " (ABORTED)" : "", (unsigned) this->count_.load(), this->count_.load() / 1000.0f);
  }
  Program *fin = this->finished_.exchange(nullptr);
  delete fin;
}

// ============================================================================
// HTTP CSV dump (R6) — esp_http_server, chunked streaming from the capture
// buffer. Refused while an experiment runs (or is pending) so R1 stays honest.
// ============================================================================

esp_err_t SliderLab::dump_handler_(httpd_req_t *req) {
  return static_cast<SliderLab *>(req->user_ctx)->handle_dump_(req);
}

esp_err_t SliderLab::handle_dump_(httpd_req_t *req) {
  this->dump_active_.store(true);  // set first: blocks new experiment adoption
  if (this->player_busy_()) {
    this->dump_active_.store(false);
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "experiment running - dump refused to keep the 1 kHz tick honest; retry when idle\n");
    return ESP_OK;
  }

  // Disable Nagle for this socket. Each httpd chunk is three writes (length
  // line, payload, CRLF); with Nagle on, the tiny framing writes ping-pong
  // with the client's delayed ACK — measured ~1.6 s per 4 KB chunk on the
  // bench (2.6 KB/s). With TCP_NODELAY the dump is wire-speed.
  const int sockfd = httpd_req_to_sockfd(req);
  if (sockfd >= 0) {
    int nodelay = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    // dump-latch fix: bound every blocking send. A dead or stalled client
    // makes the next chunk send fail within kDumpSendTimeoutS instead of
    // blocking until the TCP session dies, so dump_active_ always clears.
    struct timeval tv = {};
    tv.tv_sec = kDumpSendTimeoutS;
    tv.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  }

  const uint32_t n = this->count_.load();
  const bool raw = this->run_raw_samples_;

  httpd_resp_set_type(req, "text/csv");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"dump.csv\"");

  const size_t bufsz = 8192;
  char *buf = (char *) malloc(bufsz);
  if (buf == nullptr) {
    this->dump_active_.store(false);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    return ESP_OK;
  }

  int len = snprintf(buf, bufsz,
                     "# slider_lab dump\n"
                     "# fw: esphome %s slider_lab-phase1, built %s %s\n"
                     "# experiment: %s\n"
                     "# params: %s\n"
                     "# tick_us_nominal: 1000\n"
                     "# adc: bits=13 atten=DB_12 samples=4\n"
                     "# records: %u capacity: %u storage: %s\n"
                     "# aborted: %d\n"
                     "# flags: 1=segment_start 2=interlock_endzone 4=interlock_thermal 8=abort 16=goto_active\n",
                     ESPHOME_VERSION, __DATE__, __TIME__, this->run_name_.c_str(), this->run_params_.c_str(),
                     (unsigned) n, (unsigned) this->capacity_, this->buf_in_psram_ ? "psram" : "internal",
                     (int) this->run_aborted_);
  esp_err_t err = httpd_resp_send_chunk(req, buf, len);

  if (err == ESP_OK) {
    len = snprintf(buf, bufsz, "t_ms,adc_raw,duty_milli,seg,flags,tick_us%s\n", raw ? ",s0,s1,s2,s3" : "");
    err = httpd_resp_send_chunk(req, buf, len);
  }

  // Rows are formatted with INTEGER math only. The obvious "%.3f/%.2f" route
  // crashed the rig twice (IllegalInstruction inside newlib _dtoa_r, decoded
  // from the on-device crash reports): high-rate double formatting in the
  // httpd task, concurrent with the main task's float logging, corrupts
  // newlib's shared mprec state on this soft-float S2 — and the reboot wipes
  // the recording. Both fields are exact decimal fixed-point anyway
  // (t = µs, adc mean of 4 samples = quarter counts), so integers are not an
  // approximation; the CSV contract output is unchanged.
  // dump-latch fix: hard wall-clock cap on the whole session — a client that
  // trickle-reads (making just enough progress to dodge the send timeout)
  // must not hold dump_active_ (= the run_experiment lockout) indefinitely.
  const int64_t deadline_us = esp_timer_get_time() + kDumpSessionCapUs;
  size_t used = 0;
  for (uint32_t i = 0; i < n && err == ESP_OK; i++) {
    if (esp_timer_get_time() > deadline_us) {
      err = ESP_ERR_TIMEOUT;
      break;
    }
    const Record &r = this->buf_[i];
    const unsigned t_int = r.t_us / 1000u;
    const unsigned t_frac = r.t_us % 1000u;
    const uint32_t adc_centi = (uint32_t) lrintf(r.adc_mean * 100.0f);
    int m;
    if (raw) {
      m = snprintf(buf + used, bufsz - used, "%u.%03u,%u.%02u,%d,%u,%u,%u,%u,%u,%u,%u\n", t_int, t_frac,
                   (unsigned) (adc_centi / 100u), (unsigned) (adc_centi % 100u), (int) r.duty_milli, (unsigned) r.seg,
                   (unsigned) r.flags, (unsigned) r.tick_us, (unsigned) r.s[0], (unsigned) r.s[1], (unsigned) r.s[2],
                   (unsigned) r.s[3]);
    } else {
      m = snprintf(buf + used, bufsz - used, "%u.%03u,%u.%02u,%d,%u,%u,%u\n", t_int, t_frac,
                   (unsigned) (adc_centi / 100u), (unsigned) (adc_centi % 100u), (int) r.duty_milli, (unsigned) r.seg,
                   (unsigned) r.flags, (unsigned) r.tick_us);
    }
    used += (size_t) m;
    if (used > bufsz - 128) {
      err = httpd_resp_send_chunk(req, buf, used);
      used = 0;
    }
  }
  if (err == ESP_OK && used > 0)
    err = httpd_resp_send_chunk(req, buf, used);
  if (err == ESP_OK)
    err = httpd_resp_send_chunk(req, nullptr, 0);  // end of chunked response

  free(buf);
  this->dump_active_.store(false);
  if (err != ESP_OK) {
    // client went away or stalled mid-stream: terminate cleanly, make httpd
    // close the session. dump_active_ is released on EVERY exit path — a
    // latched flag silently refused all run_experiment calls (bench bug).
    // The recording is untouched — the buffer is only ever cleared when a
    // new experiment arms (start_run_).
    ESP_LOGW(TAG, "dump aborted after %u records (%s)", (unsigned) n,
             err == ESP_ERR_TIMEOUT ? "30 s session cap" : "client stalled/gone, socket send failed");
    return ESP_FAIL;
  }
  return ESP_OK;
}

}  // namespace slider_lab
}  // namespace esphome

#endif  // USE_ESP_IDF
