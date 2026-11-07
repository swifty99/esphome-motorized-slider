#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <esp_adc/adc_oneshot.h>

#include <map>

namespace esphome {
namespace shared_adc {

class SharedAdcManager : public Component {
 public:
  void setup() override;
  void dump_config() override;

  bool ensure_channel(adc_unit_t unit, adc_channel_t channel, adc_atten_t attenuation);
  adc_oneshot_unit_handle_t get_handle(adc_unit_t unit) const;
  esp_err_t read_raw(adc_unit_t unit, adc_channel_t channel, int *result);

 private:
  static constexpr int kMaxUnits = 2;

  struct ChannelState {
    adc_atten_t attenuation;
    bool configured;
  };

  struct UnitState {
    adc_oneshot_unit_handle_t handle{nullptr};
    bool initialized{false};
    std::map<int, ChannelState> channels;
  };

  UnitState units_[kMaxUnits];

  bool ensure_unit_(adc_unit_t unit);
};

}  // namespace shared_adc
}  // namespace esphome
