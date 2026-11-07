#pragma once

// slider_servo sensor platform: calibrated position / wiper voltage of one
// instance, or a further ADC1 channel read through the hub's shared-ADC
// window (kueche-mief-spec R2). Main task only — never touches the tick.

#include "esphome/core/defines.h"  // USE_SENSOR lives here, not on the command line

#if defined(USE_ESP_IDF) && defined(USE_SENSOR)

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#include "slider_servo.h"

namespace esphome {
namespace slider_servo_hal {

enum class ServoSensorType : uint8_t { POSITION = 0, WIPER_VOLTAGE = 1, ADC = 2 };

class SliderServoSensor : public sensor::Sensor, public PollingComponent {
 public:
  void set_type(ServoSensorType t) { this->type_ = t; }
  void set_hub(SliderServoHub *hub) { this->hub_ = hub; }
  void set_servo(SliderServo *servo) { this->servo_ = servo; }
  void set_extra(uint8_t slot, uint8_t atten, uint8_t samples, bool raw) {
    this->slot_ = slot;
    this->atten_ = atten;
    this->samples_ = samples > 0 ? samples : 1;
    this->raw_ = raw;
  }

  void update() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  ServoSensorType type_{ServoSensorType::POSITION};
  SliderServoHub *hub_{nullptr};
  SliderServo *servo_{nullptr};
  uint8_t slot_{0};
  uint8_t atten_{3};
  uint8_t samples_{1};
  bool raw_{false};
  // ADC type: a request opened by update(), filled one read per loop pass
  bool pending_{false};
  uint8_t got_{0};
  int32_t sum_{0};
};

}  // namespace slider_servo_hal
}  // namespace esphome

#endif  // USE_ESP_IDF && USE_SENSOR
