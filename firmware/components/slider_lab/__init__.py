"""slider_lab — Phase-1 lab-rig experiment firmware (motorslider project).

Clean-room ESPHome external component. Plays scripted open-loop torque
profiles on the slider rig, records raw data at 1 kHz, dumps CSV over HTTP.
Spec: docs/lab-rig-spec.md (R1-R9).

The ADC1 pin-to-channel tables below follow the ESP-IDF adc mapping
(the one item of the legacy component explicitly on the allowed reuse list).
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID
from esphome.components.esp32 import get_esp32_variant
from esphome.components.esp32.const import (
    VARIANT_ESP32,
    VARIANT_ESP32S2,
    VARIANT_ESP32S3,
)

CODEOWNERS = ["@jan"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["json"]

CONF_ADC_PIN = "adc_pin"
CONF_AIN1_PIN = "ain1_pin"
CONF_AIN2_PIN = "ain2_pin"
CONF_SYNC_PIN = "sync_pin"
CONF_HTTP_PORT = "http_port"
CONF_INVERT_DIRECTION = "invert_direction"
CONF_ENABLED = "enabled"

# ADC1 GPIO -> channel (ESP-IDF adc mapping; rig board is an ESP32-S2: GPIO9 -> ch8)
ADC1_PIN_TO_CHANNEL = {
    VARIANT_ESP32: {36: 0, 37: 1, 38: 2, 39: 3, 32: 4, 33: 5, 34: 6, 35: 7},
    VARIANT_ESP32S2: {1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5, 7: 6, 8: 7, 9: 8, 10: 9},
    VARIANT_ESP32S3: {1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5, 7: 6, 8: 7, 9: 8, 10: 9},
}

slider_lab_ns = cg.esphome_ns.namespace("slider_lab")
SliderLab = slider_lab_ns.class_("SliderLab", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SliderLab),
            cv.Optional(CONF_ADC_PIN, default=9): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_AIN1_PIN, default=11): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_AIN2_PIN, default=10): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_SYNC_PIN, default=14): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_HTTP_PORT, default=80): cv.port,
            cv.Optional(CONF_INVERT_DIRECTION, default=False): cv.boolean,
            # Motor-ownership arbiter: set false when slider_servo owns the
            # bridge in the same firmware — setup() then returns before ANY HAL
            # init (no LEDC/ADC/GPTimer/tick/HTTP), so there is no double PWM
            # setup. The HA entities still exist (dormant, benign values).
            cv.Optional(CONF_ENABLED, default=True): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    variant = get_esp32_variant()
    table = ADC1_PIN_TO_CHANNEL.get(variant)
    if table is None:
        raise cv.Invalid(f"slider_lab: ESP32 variant {variant} is not supported")
    pin = config[CONF_ADC_PIN]
    if pin not in table:
        raise cv.Invalid(f"slider_lab: {variant} has no ADC1 channel on GPIO{pin}")

    cg.add(var.set_adc_channel(table[pin]))
    cg.add(
        var.set_pins(
            config[CONF_AIN1_PIN], config[CONF_AIN2_PIN], config[CONF_SYNC_PIN]
        )
    )
    cg.add(var.set_http_port(config[CONF_HTTP_PORT]))
    cg.add(var.set_invert_direction(config[CONF_INVERT_DIRECTION]))
    cg.add(var.set_enabled(config[CONF_ENABLED]))
