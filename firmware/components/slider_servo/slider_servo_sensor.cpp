#include "slider_servo_sensor.h"

#if defined(USE_ESP_IDF) && defined(USE_SENSOR)

#include "esphome/core/log.h"

namespace esphome {
namespace slider_servo_hal {

static const char *const TAG = "slider_servo.sensor";

void SliderServoSensor::update() {
  switch (this->type_) {
    case ServoSensorType::POSITION:
      this->publish_state(this->servo_->get_position_percent());
      break;
    case ServoSensorType::WIPER_VOLTAGE:
      this->publish_state(this->hub_->raw_to_volts(this->atten_, this->servo_->get_wiper_raw()));
      break;
    case ServoSensorType::ADC:
      // open a request; loop() fills it from safe shared-ADC windows. A still
      // open request just continues (the interval was shorter than the fill).
      if (!this->pending_) {
        this->pending_ = true;
        this->got_ = 0;
        this->sum_ = 0;
      }
      break;
  }
}

void SliderServoSensor::loop() {
  if (!this->pending_)
    return;
  int raw = 0;
  if (!this->hub_->try_read_shared(this->slot_, &raw))
    return;  // not a safe window (or failed) — retry next loop pass
  this->sum_ += raw;
  if (++this->got_ < this->samples_)
    return;
  this->pending_ = false;
  const int mean = (int) (this->sum_ / this->got_);
  this->publish_state(this->raw_ ? (float) mean : this->hub_->raw_to_volts(this->atten_, mean));
}

void SliderServoSensor::dump_config() {
  LOG_SENSOR("", "slider_servo sensor", this);
  switch (this->type_) {
    case ServoSensorType::POSITION:
      ESP_LOGCONFIG(TAG, "  type: position of '%s'", this->servo_->get_name_id());
      break;
    case ServoSensorType::WIPER_VOLTAGE:
      ESP_LOGCONFIG(TAG, "  type: wiper voltage of '%s'", this->servo_->get_name_id());
      break;
    case ServoSensorType::ADC:
      ESP_LOGCONFIG(TAG, "  type: extra ADC1 slot %u, atten %u, %u sample(s)%s", this->slot_, this->atten_,
                    this->samples_, this->raw_ ? ", raw counts" : "");
      break;
  }
  LOG_UPDATE_INTERVAL(this);
}

}  // namespace slider_servo_hal
}  // namespace esphome

#endif  // USE_ESP_IDF && USE_SENSOR
