#ifdef USE_ESP_IDF

#include "slider_servo.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "esphome/core/log.h"
#include "esphome/core/version.h"

#include <esp_heap_caps.h>
#include <esp_system.h>
#include <lwip/sockets.h>

namespace esphome {
namespace slider_servo_hal {

static const char *const TAG = "slider_servo";

namespace {

// ---- hardware constants (identical drive stack to slider_lab, bench-proven) --
constexpr uint32_t kTickUs = 1000;             // 1 kHz loop (spec §1)
constexpr int kAdcSamples = 4;                 // 4 oneshot reads per tick
constexpr uint32_t kPwmFreqHz = 19531;
constexpr ledc_timer_bit_t kPwmRes = LEDC_TIMER_12_BIT;
constexpr uint32_t kPwmFull = 1u << 12;
// R6 dump-session guards (same latch fix as the lab dump handler)
constexpr int kDumpSendTimeoutS = 3;
// 60 s (up from 30): under WiFi+tick contention a 12 s ring can stream slowly
// (~15 KB/s worst case ≈ 40 s). 60 s lets it finish rather than truncate the
// trailing idle. The servo dump pauses only LOGGING, never control, so a long
// dump lockout is low-risk.
constexpr int64_t kDumpSessionCapUs = 60LL * 1000000;

int16_t sat16(long v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}
float clampf(float v, float lo, float hi) { return std::min(std::max(v, lo), hi); }

}  // namespace

// ============================================================================
// setup
// ============================================================================

void SliderServo::setup() {
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

  // --- scope-sync pin: LOW when idle, readable back (INPUT_OUTPUT) ---
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << this->sync_pin_;
  io.mode = GPIO_MODE_INPUT_OUTPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io);
  gpio_set_level((gpio_num_t) this->sync_pin_, 0);

  // --- LEDC 19531 Hz, 12-bit; boot state both outputs 0 = brake-low, silent ---
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode = LEDC_LOW_SPEED_MODE;
  tcfg.duty_resolution = kPwmRes;
  tcfg.timer_num = LEDC_TIMER_0;
  tcfg.freq_hz = kPwmFreqHz;
  tcfg.clk_cfg = LEDC_AUTO_CLK;
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

  // --- ADC oneshot, 13-bit, ATTEN_DB_12 ---
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

  // --- recorder ring: PSRAM first (descending), internal RAM fallback ---
  uint32_t want = this->req_capacity_ > 0 ? this->req_capacity_ : 45000;
  const uint32_t psram_sizes[] = {want, 45000, 30000, 15000, 8000};
  for (uint32_t n : psram_sizes) {
    this->buf_ = (Record *) heap_caps_malloc((size_t) n * sizeof(Record), MALLOC_CAP_SPIRAM);
    if (this->buf_ != nullptr) {
      this->capacity_ = n;
      this->buf_in_psram_ = true;
      break;
    }
  }
  if (this->buf_ == nullptr) {
    ESP_LOGW(TAG, "PSRAM alloc failed - falling back to internal RAM (short trace!)");
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
    ESP_LOGE(TAG, "recorder ring allocation failed entirely");
    this->mark_failed();
    return;
  }

  // --- NVS UnitCalib: load if a valid blob was written by auto-ident (Block 6);
  //     otherwise the nominal compiled-in defaults (safe-but-suboptimal). ---
  this->pref_ = global_preferences->make_preference<StoredCalib>(kCalibMagic);
  StoredCalib sc;
  if (this->pref_.load(&sc) && sc.magic == kCalibMagic && sc.version == kCalibVersion) {
    this->calib_ = sc.calib;
    ESP_LOGI(TAG, "loaded per-unit calib from NVS");
  } else {
    ESP_LOGI(TAG, "no valid NVS calib - using nominal defaults");
  }

  // --- seed the observer at the current position so it starts converged ---
  float p0 = 50.0f;
  {
    int32_t sum = 0;
    int got = 0;
    for (int i = 0; i < 8; i++) {
      int raw = 0;
      if (adc_oneshot_read(this->adc_, (adc_channel_t) this->adc_channel_, &raw) == ESP_OK) {
        sum += raw;
        got++;
      }
    }
    if (got > 0) p0 = clampf(this->raw_to_percent_((float) sum / (float) got), 0.0f, 100.0f);
  }

  // recorder ring disabled inside the core (capacity 0): the wrapper owns the
  // PSRAM ring and taps the core's read-only accessors (T13: control is
  // bit-identical whether or not anything records).
  this->servo_.init(&this->calib_, &this->cfg_, p0, 0, core::SeededBug::NONE);
  this->target_echo_.store(p0);
  this->pos_pct_.store(p0);
  this->p_hat_.store(p0);
  ESP_LOGI(TAG, "core seeded at %.1f %% (IDLE, zero torque until commanded)", p0);

  // --- HTTP dump endpoint ---
  httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
  hcfg.server_port = this->http_port_;
  hcfg.stack_size = 12288;
  hcfg.max_uri_handlers = 2;
  if (httpd_start(&this->httpd_, &hcfg) == ESP_OK) {
    httpd_uri_t uri = {};
    uri.uri = "/servo/dump.csv";
    uri.method = HTTP_GET;
    uri.handler = SliderServo::dump_handler_;
    uri.user_ctx = this;
    httpd_register_uri_handler(this->httpd_, &uri);
  } else {
    ESP_LOGE(TAG, "esp_http_server failed to start on port %u", this->http_port_);
    // not fatal: the tick + interlocks keep running, dump unavailable
  }

  // --- deterministic 1 kHz tick: GPTimer ISR -> notify -> prio-24 task ---
  this->rec_t0_us_ = esp_timer_get_time();
  this->ready_ = true;
  if (xTaskCreate(SliderServo::tick_task_trampoline_, "slider_servo_tick", 4096, this, 24, &this->tick_task_) != pdPASS) {
    ESP_LOGE(TAG, "tick task creation failed");
    this->mark_failed();
    return;
  }
  gptimer_config_t gcfg = {};
  gcfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  gcfg.direction = GPTIMER_COUNT_UP;
  gcfg.resolution_hz = 1000000;
  gptimer_event_callbacks_t cbs = {};
  cbs.on_alarm = SliderServo::timer_isr_;
  gptimer_alarm_config_t acfg = {};
  acfg.alarm_count = kTickUs;
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

void SliderServo::dump_config() {
  ESP_LOGCONFIG(TAG, "slider_servo (ESPHome servo, core = pure-C++ §2 graph):");
  ESP_LOGCONFIG(TAG, "  ADC1 ch%u, 13-bit, ATTEN_DB_12, %d samples/tick", this->adc_channel_, kAdcSamples);
  ESP_LOGCONFIG(TAG, "  L9110 AIN1=GPIO%u AIN2=GPIO%u, LEDC %u Hz 12-bit, FAST=brake-low",
                this->ain1_pin_, this->ain2_pin_, (unsigned) kPwmFreqHz);
  ESP_LOGCONFIG(TAG, "  scope sync GPIO%u, invert_direction: %s", this->sync_pin_,
                this->invert_direction_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  endpoints: raw %.0f -> 0 %%, raw %.0f -> 100 %%", this->raw0_, this->raw100_);
  ESP_LOGCONFIG(TAG, "  recorder: %u records (%.1f s) in %s", (unsigned) this->capacity_,
                this->capacity_ / 1000.0f, this->buf_in_psram_ ? "PSRAM" : "internal RAM (fallback!)");
  ESP_LOGCONFIG(TAG, "  cfg: tol %.2f %%, v_cruise %.0f, v_min %.0f, settle %d/%d ms, u_max %.2f, thermal %.2f",
                this->cfg_.tol, this->cfg_.v_cruise, this->cfg_.v_min_cmd, this->cfg_.impulse_settle_ms,
                this->cfg_.impulse_settle_short_ms, this->cfg_.u_max, this->cfg_.thermal_avg_max);
  ESP_LOGCONFIG(TAG, "  dump: http://<ip>:%u/servo/dump.csv", this->http_port_);
}

// ============================================================================
// 1 kHz tick (dedicated task, prio 24) — NOTHING here may block.
// ============================================================================

bool IRAM_ATTR SliderServo::timer_isr_(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg) {
  auto *self = static_cast<SliderServo *>(arg);
  BaseType_t hp_woken = pdFALSE;
  vTaskNotifyGiveFromISR(self->tick_task_, &hp_woken);
  return hp_woken == pdTRUE;
}

void SliderServo::tick_task_trampoline_(void *arg) {
  auto *self = static_cast<SliderServo *>(arg);
  while (true) {
    // clear-on-exit: coalesce missed notifications into one tick (a t gap is
    // honest; a catch-up burst would fake the timeline). The count is the A9
    // missed-tick metric.
    uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (n > 1) self->missed_ticks_.fetch_add(n - 1, std::memory_order_relaxed);
    self->tick_();
  }
}

void SliderServo::tick_() {
  const int64_t t_entry = esp_timer_get_time();

  // 1) sample ADC — 4 oneshot reads, mean, count failures (never react to the
  //    error in-tick: 2026-07-17 the ADC read flat 0 for minutes; only OTA
  //    reboot cleared it — the counter names the failure instead of silence).
  int32_t sum = 0;
  for (int i = 0; i < kAdcSamples; i++) {
    int raw = 0;
    if (adc_oneshot_read(this->adc_, (adc_channel_t) this->adc_channel_, &raw) != ESP_OK)
      this->adc_err_count_.fetch_add(1, std::memory_order_relaxed);
    sum += raw;
  }
  const float adc_mean = (float) sum / (float) kAdcSamples;
  const float z = this->raw_to_percent_(adc_mean);

  // 2) consume commands latched by the HA lambdas (main task). The core
  //    controller is single-threaded — only the tick drives it.
  this->servo_.set_abort(this->abort_req_.load());
  if (this->release_req_.exchange(false))
    this->servo_.release();
  const uint32_t seq = this->cmd_seq_.load();
  if (seq != this->last_seq_) {
    this->last_seq_ = seq;
    this->servo_.set_target(this->cmd_target_.load(), this->cmd_speed_.load());
  }

  // 3) one control evaluation: observer -> §2 graph -> L1. Returns applied duty.
  const float duty = this->servo_.tick(z);

  // 4) drive the L9110 (FAST = brake-low recipe, identical to slider_lab)
  this->apply_duty_(duty);

  // 5) scope sync: HIGH while the controller is actively working
  const core::State st = this->servo_.controller().state();
  const bool driving = st == core::State::MOVE || st == core::State::BRAKE ||
                       st == core::State::PULSE || st == core::State::CRAWL ||
                       st == core::State::GLIDE;
  gpio_set_level((gpio_num_t) this->sync_pin_, driving ? 1 : 0);

  // 6) snapshot for sensors + recorder
  const uint16_t tick_us = (uint16_t) (esp_timer_get_time() - t_entry);
  this->snapshot_(z, duty, tick_us);
  this->last_tick_us_.store(tick_us);
  const int bucket = std::min((int) (tick_us / kHistStepUs), kHistBuckets - 1);
  this->tick_hist_[bucket]++;
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
// duty -> L9110 via LEDC. FAST decay recipe (brake-low), verbatim from
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
  const ledc_channel_t ch = (idx == 0) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

// ============================================================================
// main-task surface: sensor getters + deferred config log
// ============================================================================

uint16_t SliderServo::get_tick_p99_us() const {
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

void SliderServo::save_calib() {
  StoredCalib sc{kCalibMagic, kCalibVersion, this->calib_};
  if (this->pref_.save(&sc)) {
    global_preferences->sync();
    ESP_LOGI(TAG, "saved per-unit calib to NVS");
  } else {
    ESP_LOGW(TAG, "NVS calib save failed");
  }
}

void SliderServo::loop() {
  // nothing deferred to log yet; sensors read the atomics directly.
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
                     "# fw: esphome %s, built %s %s\n"
                     "# tick_us_nominal: 1000\n"
                     "# adc: bits=13 atten=DB_12 samples=4  endpoints: raw%0.f=0%% raw%0.f=100%%\n"
                     "# records: %u capacity: %u storage: %s  missed_ticks: %u  adc_errors: %u\n"
                     "# state: 0=idle 1=move 2=brake 3=pulse 4=wait 5=crawl 6=released\n"
                     "# settle: 0=none 1=reached 2=gave_up 3=adopted\n"
                     "# release: 0=none 1=grab 2=interlock 3=watchdog 4=fault 5=command\n"
                     "# columns: z/p_hat in %%, v_hat/v_des in %%/s, u_cmd/u_applied/ff in duty\n",
                     ESPHOME_VERSION, __DATE__, __TIME__, this->raw0_, this->raw100_, (unsigned) count,
                     (unsigned) this->capacity_, this->buf_in_psram_ ? "psram" : "internal",
                     (unsigned) this->missed_ticks_.load(), (unsigned) this->adc_err_count_.load());
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
    ESP_LOGW(TAG, "dump aborted after %u records (%s)", (unsigned) count,
             err == ESP_ERR_TIMEOUT ? "30 s session cap" : "client stalled/gone");
    return ESP_FAIL;
  }
  return ESP_OK;
}

}  // namespace slider_servo_hal
}  // namespace esphome

#endif  // USE_ESP_IDF
