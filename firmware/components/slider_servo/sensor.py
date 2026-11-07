"""slider_servo sensor platform (kueche-mief-spec R2/R3).

`slider_servo` owns ADC1 (one adc_oneshot unit for every instance), so the
ESPHome `adc` platform cannot run on ADC1 next to it. This platform delivers
what the board needs instead — ID, name, unit, device_class and filters are
the user's, exactly like any other sensor:

  type: position       calibrated position of one instance [%] (the tick's z,
                       raw_at_0/raw_at_100 endpoints, later the ident's)
  type: wiper_voltage  that instance's wiper voltage [V] (12 dB, IDF calib) —
                       lets a legacy filter chain (multiply / calibrate_linear)
                       run unchanged on the same physical quantity
  type: adc            any further ADC1 pin [V] (or raw counts), sampled in the
                       MAIN loop through the hub's shared-ADC window, never in
                       the 1 kHz tick (see the locking model in slider_servo.h)
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor
from esphome.const import (
    CONF_ATTENUATION,
    CONF_PIN,
    CONF_RAW,

    CONF_TYPE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    UNIT_VOLT,
)
from esphome.core import CORE

from . import SliderServo, adc1_channel, get_hub, slider_servo_ns

DEPENDENCIES = ["slider_servo"]

CONF_SLIDER_SERVO_ID = "slider_servo_id"
CONF_SAMPLES = "samples"
MAX_EXTRA_CHANNELS = 8  # == SliderServoHub::kMaxExtraChannels
_DATA_EXTRA = "slider_servo_next_extra"

# adc_atten_t values (IDF): DB_0 = 0, DB_2_5 = 1, DB_6 = 2, DB_12 = 3
ATTENUATIONS = {"0db": 0, "2.5db": 1, "6db": 2, "12db": 3}
WIPER_ATTEN = 3  # wipers always run at 12 dB (slider_servo.cpp hw_setup_)

SliderServoSensor = slider_servo_ns.class_("SliderServoSensor", sensor.Sensor, cg.PollingComponent)
ServoSensorType = slider_servo_ns.enum("ServoSensorType", is_class=True)

TYPE_POSITION = "position"
TYPE_WIPER_VOLTAGE = "wiper_voltage"
TYPE_ADC = "adc"

_INSTANCE = {cv.Required(CONF_SLIDER_SERVO_ID): cv.use_id(SliderServo)}

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_POSITION: sensor.sensor_schema(
            SliderServoSensor,
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        )
        .extend(_INSTANCE)
        .extend(cv.polling_component_schema("100ms")),
        TYPE_WIPER_VOLTAGE: sensor.sensor_schema(
            SliderServoSensor,
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        )
        .extend(_INSTANCE)
        .extend(cv.polling_component_schema("100ms")),
        TYPE_ADC: sensor.sensor_schema(
            SliderServoSensor,
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        )
        .extend(
            {
                cv.Required(CONF_PIN): pins.internal_gpio_input_pin_number,
                cv.Optional(CONF_ATTENUATION, default="12db"): cv.one_of(*ATTENUATIONS, lower=True),
                # averaged over consecutive main-loop windows (one read per pass)
                cv.Optional(CONF_SAMPLES, default=1): cv.int_range(min=1, max=64),
                cv.Optional(CONF_RAW, default=False): cv.boolean,
            }
        )
        .extend(cv.polling_component_schema("1s")),
    },
    lower=True,
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    hub = await get_hub()
    cg.add(var.set_hub(hub))

    kind = config[CONF_TYPE]
    if kind == TYPE_ADC:
        slot = CORE.data.get(_DATA_EXTRA, 0)
        if slot >= MAX_EXTRA_CHANNELS:
            raise cv.Invalid(f"slider_servo: at most {MAX_EXTRA_CHANNELS} extra ADC1 channels")
        CORE.data[_DATA_EXTRA] = slot + 1
        atten = ATTENUATIONS[config[CONF_ATTENUATION]]
        cg.add(hub.add_extra_channel(adc1_channel(config[CONF_PIN]), atten))
        cg.add(var.set_type(ServoSensorType.ADC))
        cg.add(var.set_extra(slot, atten, config[CONF_SAMPLES], config[CONF_RAW]))
        return

    servo = await cg.get_variable(config[CONF_SLIDER_SERVO_ID])
    cg.add(var.set_servo(servo))
    if kind == TYPE_POSITION:
        cg.add(var.set_type(ServoSensorType.POSITION))
    else:
        cg.add(var.set_type(ServoSensorType.WIPER_VOLTAGE))
        cg.add(var.set_extra(0, WIPER_ATTEN, 1, False))
