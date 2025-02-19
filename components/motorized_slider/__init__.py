import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import  output, sensor
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_NUMBER,
)

from esphome.core import CORE
from esphome.components.esp32 import get_esp32_variant
from esphome.components.esp32.const import (
    VARIANT_ESP32,
    VARIANT_ESP32C2,
    VARIANT_ESP32C3,
    VARIANT_ESP32C6,
    VARIANT_ESP32H2,
    VARIANT_ESP32S2,
    VARIANT_ESP32S3,
)
# __init__



CODEOWNERS = ["@swifty99"]

DEPENDENCIES = []

adc1_channel_t = cg.global_ns.enum("adc1_channel_t")

# From https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/adc_common.h
# pin to adc1 channel mapping
ESP32_VARIANT_ADC1_PIN_TO_CHANNEL = {
    VARIANT_ESP32: {
        36: adc1_channel_t.ADC1_CHANNEL_0,
        37: adc1_channel_t.ADC1_CHANNEL_1,
        38: adc1_channel_t.ADC1_CHANNEL_2,
        39: adc1_channel_t.ADC1_CHANNEL_3,
        32: adc1_channel_t.ADC1_CHANNEL_4,
        33: adc1_channel_t.ADC1_CHANNEL_5,
        34: adc1_channel_t.ADC1_CHANNEL_6,
        35: adc1_channel_t.ADC1_CHANNEL_7,
    },
    VARIANT_ESP32S2: {
        1: adc1_channel_t.ADC1_CHANNEL_0,
        2: adc1_channel_t.ADC1_CHANNEL_1,
        3: adc1_channel_t.ADC1_CHANNEL_2,
        4: adc1_channel_t.ADC1_CHANNEL_3,
        5: adc1_channel_t.ADC1_CHANNEL_4,
        6: adc1_channel_t.ADC1_CHANNEL_5,
        7: adc1_channel_t.ADC1_CHANNEL_6,
        8: adc1_channel_t.ADC1_CHANNEL_7,
        9: adc1_channel_t.ADC1_CHANNEL_8,
        10: adc1_channel_t.ADC1_CHANNEL_9,
    },
    VARIANT_ESP32S3: {
        1: adc1_channel_t.ADC1_CHANNEL_0,
        2: adc1_channel_t.ADC1_CHANNEL_1,
        3: adc1_channel_t.ADC1_CHANNEL_2,
        4: adc1_channel_t.ADC1_CHANNEL_3,
        5: adc1_channel_t.ADC1_CHANNEL_4,
        6: adc1_channel_t.ADC1_CHANNEL_5,
        7: adc1_channel_t.ADC1_CHANNEL_6,
        8: adc1_channel_t.ADC1_CHANNEL_7,
        9: adc1_channel_t.ADC1_CHANNEL_8,
        10: adc1_channel_t.ADC1_CHANNEL_9,
    },
    VARIANT_ESP32C3: {
        0: adc1_channel_t.ADC1_CHANNEL_0,
        1: adc1_channel_t.ADC1_CHANNEL_1,
        2: adc1_channel_t.ADC1_CHANNEL_2,
        3: adc1_channel_t.ADC1_CHANNEL_3,
        4: adc1_channel_t.ADC1_CHANNEL_4,
    },
    VARIANT_ESP32C2: {
        0: adc1_channel_t.ADC1_CHANNEL_0,
        1: adc1_channel_t.ADC1_CHANNEL_1,
        2: adc1_channel_t.ADC1_CHANNEL_2,
        3: adc1_channel_t.ADC1_CHANNEL_3,
        4: adc1_channel_t.ADC1_CHANNEL_4,
    },
    VARIANT_ESP32C6: {
        0: adc1_channel_t.ADC1_CHANNEL_0,
        1: adc1_channel_t.ADC1_CHANNEL_1,
        2: adc1_channel_t.ADC1_CHANNEL_2,
        3: adc1_channel_t.ADC1_CHANNEL_3,
        4: adc1_channel_t.ADC1_CHANNEL_4,
        5: adc1_channel_t.ADC1_CHANNEL_5,
        6: adc1_channel_t.ADC1_CHANNEL_6,
    },
    VARIANT_ESP32H2: {
        0: adc1_channel_t.ADC1_CHANNEL_0,
        1: adc1_channel_t.ADC1_CHANNEL_1,
        2: adc1_channel_t.ADC1_CHANNEL_2,
        3: adc1_channel_t.ADC1_CHANNEL_3,
        4: adc1_channel_t.ADC1_CHANNEL_4,
    },
}



def validate_adc_pin(value):
    if CORE.is_esp32:
        conf = pins.internal_gpio_input_pin_schema(value)
        value = conf[CONF_NUMBER]
        variant = get_esp32_variant()
        if (
            variant not in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL
        ):
            raise cv.Invalid(f"This ESP32 variant ({variant}) is not supported")

        if (
            value not in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant]
        ):
            raise cv.Invalid(f"{variant} doesn't support ADC on this pin")
        

        return conf

    # else:
    raise NotImplementedError



# Definieren Sie C++-Namespace und Klassen
ff_slider_ns = cg.esphome_ns.namespace("motorized_slider")
MotorController = ff_slider_ns.class_('MotorController', cg.Component)

# YAML-Konfigurationsparameter
CONF_PWM_FORWARD = "pwm_forward"
CONF_PWM_BACKWARD = "pwm_backward"
CONF_FEEDBACK_SENSOR = "sensor_pin"
CONF_RASTPUNKTE = "rastpunkte"
CONF_TARGET_POSITION = "target_position"
CONF_SLIDER_POSITION_SENSOR = "slider_position_sensor"
CONF_POSITION = "position"
CONF_STIFFNESS = "stiffness"

RastpunktSchema = cv.Schema({
    cv.Required(CONF_POSITION): cv.All(cv.float_, cv.Range(min=0.0, max=100.0)),
    cv.Required(CONF_STIFFNESS): cv.All(cv.float_, cv.Range(min=0.0, max=1.0)),
})


CONFIG_SCHEMA = cv.Schema({
        cv.GenerateID(): cv.declare_id(MotorController),
        cv.Required(CONF_PWM_FORWARD): cv.use_id(output.FloatOutput),
        cv.Required(CONF_PWM_BACKWARD): cv.use_id(output.FloatOutput),
        cv.Optional(CONF_RASTPUNKTE, default=[]): cv.ensure_list(RastpunktSchema),
        cv.Optional(CONF_TARGET_POSITION, default=0.0): cv.float_,    
        cv.Optional(CONF_SLIDER_POSITION_SENSOR): sensor.sensor_schema(
            unit_of_measurement="%",
            accuracy_decimals=2,
            )
    }).extend(cv.COMPONENT_SCHEMA.extend({
        cv.Required(CONF_FEEDBACK_SENSOR): validate_adc_pin,
        }
    ))
        


# Configuration setup
async def to_code(config):
    # Initialize the MotorDriver with the PWM outputs
    pwm_forward = await cg.get_variable(config[CONF_PWM_FORWARD])
    pwm_backward = await cg.get_variable(config[CONF_PWM_BACKWARD])


    # Initialize the MotorController
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_pwm_forward(pwm_forward))
    cg.add(var.set_pwm_backward(pwm_backward))
    cg.add(var.set_target_position(config[CONF_TARGET_POSITION]))

    # add the pin

    variant = get_esp32_variant()
    pin_num = config[CONF_FEEDBACK_SENSOR][CONF_NUMBER]
    chan = ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant][pin_num]
    cg.add(var.set_adcchannel(chan))

    # Add rastpunkte
    for rastpunkt in config[CONF_RASTPUNKTE]:
        pos = rastpunkt[CONF_POSITION]
        stiffness = rastpunkt[CONF_STIFFNESS]
        cg.add(var.add_rastpunkt(pos, stiffness))

    # Add slider position sensor
    if CONF_SLIDER_POSITION_SENSOR in config:
        slider_position_sensor = await sensor.new_sensor(config[CONF_SLIDER_POSITION_SENSOR])
        cg.add(var.set_slider_position_sensor(slider_position_sensor))

    # Register the MotorController component
    await cg.register_component(var, config)