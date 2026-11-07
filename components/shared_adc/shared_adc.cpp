#include "shared_adc.h"

#include "esphome/core/log.h"

namespace esphome {
namespace shared_adc {

static const char *const TAG = "shared_adc";

void SharedAdcManager::setup() {
  ESP_LOGCONFIG(TAG, "Initializing shared ADC manager");
}

void SharedAdcManager::dump_config() {
  ESP_LOGCONFIG(TAG, "Shared ADC manager");
  for (int unit = 0; unit < kMaxUnits; ++unit) {
    const auto &state = this->units_[unit];
    if (state.initialized) {
      ESP_LOGCONFIG(TAG, "  Unit %d: %zu configured channel(s)", unit,
                    static_cast<size_t>(state.channels.size()));
    }
  }
}

bool SharedAdcManager::ensure_channel(adc_unit_t unit, adc_channel_t channel, adc_atten_t attenuation) {
  if (!this->ensure_unit_(unit)) {
    return false;
  }

  auto &unit_state = this->units_[unit];
  auto key = static_cast<int>(channel);
  auto it = unit_state.channels.find(key);
  if (it != unit_state.channels.end()) {
    if (it->second.attenuation != attenuation) {
      ESP_LOGW(TAG, "ADC unit %d channel %d already configured with different attenuation", unit, channel);
    }
    return it->second.configured;
  }

  adc_oneshot_chan_cfg_t channel_config = {
      .atten = attenuation,
      .bitwidth = ADC_BITWIDTH_13,
  };

  esp_err_t err = adc_oneshot_config_channel(unit_state.handle, channel, &channel_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure ADC unit %d channel %d (err=%d)", unit, channel, err);
    return false;
  }

  unit_state.channels.emplace(key, ChannelState{attenuation, true});
  return true;
}

adc_oneshot_unit_handle_t SharedAdcManager::get_handle(adc_unit_t unit) const {
  if (unit >= kMaxUnits) {
    return nullptr;
  }
  return this->units_[unit].handle;
}

esp_err_t SharedAdcManager::read_raw(adc_unit_t unit, adc_channel_t channel, int *result) {
  if (result == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  if (unit >= kMaxUnits) {
    ESP_LOGE(TAG, "ADC unit %d out of range", unit);
    return ESP_ERR_INVALID_ARG;
  }

  auto &unit_state = this->units_[unit];
  if (!unit_state.initialized || unit_state.handle == nullptr) {
    ESP_LOGE(TAG, "ADC unit %d not initialized", unit);
    return ESP_ERR_INVALID_STATE;
  }

  auto key = static_cast<int>(channel);
  if (unit_state.channels.find(key) == unit_state.channels.end()) {
    ESP_LOGE(TAG, "ADC unit %d channel %d not configured", unit, channel);
    return ESP_ERR_INVALID_STATE;
  }

  return adc_oneshot_read(unit_state.handle, channel, result);
}

bool SharedAdcManager::ensure_unit_(adc_unit_t unit) {
  if (unit >= kMaxUnits) {
    ESP_LOGE(TAG, "ADC unit %d out of supported range", unit);
    return false;
  }

  auto &unit_state = this->units_[unit];
  if (unit_state.initialized) {
    return true;
  }

  adc_oneshot_unit_init_cfg_t unit_config = {};
  unit_config.unit_id = unit;
  unit_config.ulp_mode = ADC_ULP_MODE_DISABLE;
#if USE_ESP32_VARIANT_ESP32C3 || USE_ESP32_VARIANT_ESP32C5 || USE_ESP32_VARIANT_ESP32C6 || \
    USE_ESP32_VARIANT_ESP32C61 || USE_ESP32_VARIANT_ESP32H2
  unit_config.clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
#endif

  esp_err_t err = adc_oneshot_new_unit(&unit_config, &unit_state.handle);
  if (err == ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "ADC unit %d already in use by another component", unit);
    return false;
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ADC unit %d (err=%d)", unit, err);
    return false;
  }

  unit_state.initialized = true;
  ESP_LOGCONFIG(TAG, "ADC unit %d initialized", unit);
  return true;
}

}  // namespace shared_adc
}  // namespace esphome
