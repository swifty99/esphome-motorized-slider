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

adc_channel_t = cg.global_ns.enum("adc_channel_t")
adc_unit_t = cg.global_ns.enum("adc_unit_t")

# From https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/adc_common.h
# pin to adc1 channel mapping
ESP32_VARIANT_ADC1_PIN_TO_CHANNEL = {
    VARIANT_ESP32: {
        36: adc_channel_t.ADC_CHANNEL_0,
        37: adc_channel_t.ADC_CHANNEL_1,
        38: adc_channel_t.ADC_CHANNEL_2,
        39: adc_channel_t.ADC_CHANNEL_3,
        32: adc_channel_t.ADC_CHANNEL_4,
        33: adc_channel_t.ADC_CHANNEL_5,
        34: adc_channel_t.ADC_CHANNEL_6,
        35: adc_channel_t.ADC_CHANNEL_7,
    },
    VARIANT_ESP32S2: {
        1: adc_channel_t.ADC_CHANNEL_0,
        2: adc_channel_t.ADC_CHANNEL_1,
        3: adc_channel_t.ADC_CHANNEL_2,
        4: adc_channel_t.ADC_CHANNEL_3,
        5: adc_channel_t.ADC_CHANNEL_4,
        6: adc_channel_t.ADC_CHANNEL_5,
        7: adc_channel_t.ADC_CHANNEL_6,
        8: adc_channel_t.ADC_CHANNEL_7,
        9: adc_channel_t.ADC_CHANNEL_8,
        10: adc_channel_t.ADC_CHANNEL_9,
    },
    VARIANT_ESP32S3: {
        1: adc_channel_t.ADC_CHANNEL_0,
        2: adc_channel_t.ADC_CHANNEL_1,
        3: adc_channel_t.ADC_CHANNEL_2,
        4: adc_channel_t.ADC_CHANNEL_3,
        5: adc_channel_t.ADC_CHANNEL_4,
        6: adc_channel_t.ADC_CHANNEL_5,
        7: adc_channel_t.ADC_CHANNEL_6,
        8: adc_channel_t.ADC_CHANNEL_7,
        9: adc_channel_t.ADC_CHANNEL_8,
        10: adc_channel_t.ADC_CHANNEL_9,
    },
    VARIANT_ESP32C3: {
        0: adc_channel_t.ADC_CHANNEL_0,
        1: adc_channel_t.ADC_CHANNEL_1,
        2: adc_channel_t.ADC_CHANNEL_2,
        3: adc_channel_t.ADC_CHANNEL_3,
        4: adc_channel_t.ADC_CHANNEL_4,
    },
    VARIANT_ESP32C2: {
        0: adc_channel_t.ADC_CHANNEL_0,
        1: adc_channel_t.ADC_CHANNEL_1,
        2: adc_channel_t.ADC_CHANNEL_2,
        3: adc_channel_t.ADC_CHANNEL_3,
        4: adc_channel_t.ADC_CHANNEL_4,
    },
    VARIANT_ESP32C6: {
        0: adc_channel_t.ADC_CHANNEL_0,
        1: adc_channel_t.ADC_CHANNEL_1,
        2: adc_channel_t.ADC_CHANNEL_2,
        3: adc_channel_t.ADC_CHANNEL_3,
        4: adc_channel_t.ADC_CHANNEL_4,
        5: adc_channel_t.ADC_CHANNEL_5,
        6: adc_channel_t.ADC_CHANNEL_6,
    },
    VARIANT_ESP32H2: {
        0: adc_channel_t.ADC_CHANNEL_0,
        1: adc_channel_t.ADC_CHANNEL_1,
        2: adc_channel_t.ADC_CHANNEL_2,
        3: adc_channel_t.ADC_CHANNEL_3,
        4: adc_channel_t.ADC_CHANNEL_4,
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
CONF_DITHER_FREQUENCY = "dither_frequency"
CONF_DITHER_AMPLITUDE = "dither_amplitude"
CONF_DITHER_IDLE_RATIO = "dither_idle_ratio"
CONF_STATIC_FRICTION_FORWARD = "static_friction_forward"
CONF_STATIC_FRICTION_BACKWARD = "static_friction_backward"
CONF_FRICTION_VELOCITY_THRESHOLD = "friction_velocity_threshold"
CONF_FRICTION_DYNAMIC_GAIN = "friction_dynamic_gain"
CONF_FRICTION_COMMAND_THRESHOLD = "friction_command_threshold"
CONF_OVERRIDE_POSITION_THRESHOLD = "override_position_threshold"
CONF_OVERRIDE_VELOCITY_THRESHOLD = "override_velocity_threshold"
CONF_OVERRIDE_DEBOUNCE_TIME = "override_debounce_time"
CONF_MANUAL_OVERRIDE_DETECTION = "manual_override_detection"
CONF_DIAGNOSTIC_LOGGING = "diagnostic_logging"
CONF_PROCESS_NOISE = "process_noise"
CONF_FRICTION_DEADBAND = "friction_deadband"
CONF_PID_KP = "pid_kp"
CONF_PID_KI = "pid_ki"
CONF_PID_KD = "pid_kd"
CONF_RASTPUNKTE_ENABLED = "rastpunkte_enabled"

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
        cv.Optional(CONF_DITHER_FREQUENCY, default=50): cv.int_,
        cv.Optional(CONF_DITHER_AMPLITUDE, default=0.0): cv.float_,
        cv.Optional(CONF_DITHER_IDLE_RATIO, default=0.0): cv.All(cv.float_, cv.Range(min=0.0, max=0.49)),
        cv.Optional(CONF_STATIC_FRICTION_FORWARD, default=0.35): cv.All(cv.float_, cv.Range(min=0.0, max=1.0)),
        cv.Optional(CONF_STATIC_FRICTION_BACKWARD, default=0.35): cv.All(cv.float_, cv.Range(min=0.0, max=1.0)),
        cv.Optional(CONF_FRICTION_VELOCITY_THRESHOLD, default=12.0): cv.All(cv.float_, cv.Range(min=0.0, max=200.0)),
        cv.Optional(CONF_FRICTION_DYNAMIC_GAIN, default=0.0005): cv.All(cv.float_, cv.Range(min=0.0, max=0.01)),
        cv.Optional(CONF_FRICTION_COMMAND_THRESHOLD, default=0.02): cv.All(cv.float_, cv.Range(min=0.0, max=0.5)),
        cv.Optional(CONF_FRICTION_DEADBAND, default=2.0): cv.All(cv.float_, cv.Range(min=0.0, max=20.0)),
        cv.Optional(CONF_OVERRIDE_POSITION_THRESHOLD, default=5.0): cv.All(cv.float_, cv.Range(min=0.0, max=50.0)),
        cv.Optional(CONF_OVERRIDE_VELOCITY_THRESHOLD, default=70.0): cv.All(cv.float_, cv.Range(min=0.0, max=500.0)),
        cv.Optional(CONF_OVERRIDE_DEBOUNCE_TIME, default=200): cv.All(cv.int_, cv.Range(min=0, max=5000)),
        cv.Optional(CONF_MANUAL_OVERRIDE_DETECTION, default=True): cv.boolean,
        cv.Optional(CONF_DIAGNOSTIC_LOGGING, default=False): cv.boolean,
        cv.Optional(CONF_PROCESS_NOISE, default=0.01): cv.All(cv.float_, cv.Range(min=0.0, max=0.1)),
        cv.Optional(CONF_PID_KP, default=0.05): cv.float_,
        cv.Optional(CONF_PID_KI, default=0.005): cv.float_,
        cv.Optional(CONF_PID_KD, default=0.0): cv.float_,
        cv.Optional(CONF_RASTPUNKTE_ENABLED, default=True): cv.boolean,
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
    cg.add(var.set_dither_frequency(config[CONF_DITHER_FREQUENCY]))
    cg.add(var.set_dither_amplitude(config[CONF_DITHER_AMPLITUDE]))
    cg.add(var.set_dither_idle_ratio(config[CONF_DITHER_IDLE_RATIO]))
    cg.add(var.set_static_friction_forward(config[CONF_STATIC_FRICTION_FORWARD]))
    cg.add(var.set_static_friction_backward(config[CONF_STATIC_FRICTION_BACKWARD]))
    cg.add(var.set_friction_velocity_threshold(config[CONF_FRICTION_VELOCITY_THRESHOLD]))
    cg.add(var.set_friction_dynamic_gain(config[CONF_FRICTION_DYNAMIC_GAIN]))
    cg.add(var.set_friction_command_threshold(config[CONF_FRICTION_COMMAND_THRESHOLD]))
    cg.add(var.set_friction_deadband(config[CONF_FRICTION_DEADBAND]))
    cg.add(var.set_override_position_threshold(config[CONF_OVERRIDE_POSITION_THRESHOLD]))
    cg.add(var.set_override_velocity_threshold(config[CONF_OVERRIDE_VELOCITY_THRESHOLD]))
    cg.add(var.set_override_debounce_time(config[CONF_OVERRIDE_DEBOUNCE_TIME]))
    cg.add(var.set_override_detection(config[CONF_MANUAL_OVERRIDE_DETECTION]))
    cg.add(var.set_diagnostic_logging(config[CONF_DIAGNOSTIC_LOGGING]))
    cg.add(var.set_process_noise(config[CONF_PROCESS_NOISE]))
    cg.add(var.set_pid_kp(config[CONF_PID_KP]))
    cg.add(var.set_pid_ki(config[CONF_PID_KI]))
    cg.add(var.set_pid_kd(config[CONF_PID_KD]))
    cg.add(var.set_rastpunkte_enabled(config[CONF_RASTPUNKTE_ENABLED]))

    # add the pin

    variant = get_esp32_variant()
    pin_num = config[CONF_FEEDBACK_SENSOR][CONF_NUMBER]
    chan = ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant][pin_num]
    cg.add(var.set_adc_channel(adc_unit_t.ADC_UNIT_1, chan))

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