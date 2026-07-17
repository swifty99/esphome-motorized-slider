"""slider_servo — the ESPHome servo component (motorslider project).

The delivered product: the pure-C++ control core (core/*, spec §2 graph + EKF
observer + endgame + CRAWL + L1) wrapped as an ESPHome external component. This
codegen wires the config to the HAL wrapper (slider_servo.cpp). The core is
never touched here — it is the same code the SIL harness (sil/) validates.

Drive stack is identical to slider_lab (bench-proven): ADC1 oneshot on GPIO9,
L9110 AIN1=GPIO11/AIN2=GPIO10 at 19531 Hz, FAST = brake-low decay.
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

CONF_ADC_PIN = "adc_pin"
CONF_AIN1_PIN = "ain1_pin"
CONF_AIN2_PIN = "ain2_pin"
CONF_SYNC_PIN = "sync_pin"
CONF_HTTP_PORT = "http_port"
CONF_INVERT_DIRECTION = "invert_direction"
CONF_RAW_AT_0 = "raw_at_0"
CONF_RAW_AT_100 = "raw_at_100"
CONF_RECORDER_CAPACITY = "recorder_capacity"
CONF_TOL = "tol"
CONF_V_CRUISE = "v_cruise"
CONF_V_MIN_CMD = "v_min_cmd"
CONF_IMPULSE_SETTLE_MS = "impulse_settle_ms"
CONF_IMPULSE_SETTLE_SHORT_MS = "impulse_settle_short_ms"
CONF_IMPULSE_AIM = "impulse_aim"
CONF_IMPULSE_MAX = "impulse_max"
CONF_CRAWL_IMPULSE_DUTY = "crawl_impulse_duty"
CONF_CRAWL_MAX_WIDTH_MS = "crawl_max_width_ms"
CONF_CRAWL_STEP = "crawl_step"
CONF_CRAWL_W_GAIN = "crawl_w_gain"
CONF_CRAWL_SETTLE_MS = "crawl_settle_ms"
CONF_CRAWL_LAND_ZONE = "crawl_land_zone"
CONF_GLIDE = "glide"
CONF_GLIDE_FLOOR = "glide_floor"
CONF_GLIDE_STEP = "glide_step"
CONF_GLIDE_SETTLE_MS = "glide_settle_ms"
CONF_V_ROLL = "v_roll"
CONF_GLIDE_PROG_WIN = "glide_prog_win"
CONF_GLIDE_RUNAWAY_V = "glide_runaway_v"
CONF_GLIDE_MAX_GAP_MS = "glide_max_gap_ms"
CONF_COAST_ZONE = "coast_zone"
CONF_THERMAL_AVG_MAX = "thermal_avg_max"
CONF_U_MAX = "u_max"

# ADC1 GPIO -> channel (ESP-IDF adc mapping; rig board is an ESP32-S2: GPIO9 -> ch8)
ADC1_PIN_TO_CHANNEL = {
    VARIANT_ESP32: {36: 0, 37: 1, 38: 2, 39: 3, 32: 4, 33: 5, 34: 6, 35: 7},
    VARIANT_ESP32S2: {1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5, 7: 6, 8: 7, 9: 8, 10: 9},
    VARIANT_ESP32S3: {1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5, 7: 6, 8: 7, 9: 8, 10: 9},
}

# C++ namespace is slider_servo_hal (not slider_servo): the pure-C++ control
# core lives in the global ::slider_servo namespace, and ESPHome's generated
# main.cpp does `using namespace esphome;` — so an esphome::slider_servo would
# be ambiguous with ::slider_servo at global scope. The component/YAML key and
# the log tag stay "slider_servo".
slider_servo_ns = cg.esphome_ns.namespace("slider_servo_hal")
SliderServo = slider_servo_ns.class_("SliderServo", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SliderServo),
            cv.Optional(CONF_ADC_PIN, default=9): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_AIN1_PIN, default=11): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_AIN2_PIN, default=10): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_SYNC_PIN, default=14): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_HTTP_PORT, default=80): cv.port,
            cv.Optional(CONF_INVERT_DIRECTION, default=False): cv.boolean,
            # endpoint calibration (counts->%): raw 8000 -> 0 %, raw 100 -> 100 %
            cv.Optional(CONF_RAW_AT_0, default=8000.0): cv.float_,
            cv.Optional(CONF_RAW_AT_100, default=100.0): cv.float_,
            cv.Optional(CONF_RECORDER_CAPACITY, default=45000): cv.int_range(min=1000, max=75000),
            # §5 tunables (HA? column). Recompile-to-retune during HIL bring-up;
            # defaults here match ServoConfig so an empty block is the spec default.
            cv.Optional(CONF_TOL, default=0.30): cv.float_range(min=0.05, max=2.0),
            cv.Optional(CONF_V_CRUISE, default=180.0): cv.float_range(min=20.0, max=400.0),
            cv.Optional(CONF_V_MIN_CMD, default=120.0): cv.float_range(min=20.0, max=300.0),
            cv.Optional(CONF_IMPULSE_SETTLE_MS, default=120): cv.int_range(min=20, max=1000),
            cv.Optional(CONF_IMPULSE_SETTLE_SHORT_MS, default=70): cv.int_range(min=20, max=1000),
            cv.Optional(CONF_IMPULSE_AIM, default=0.90): cv.float_range(min=0.5, max=1.0),
            cv.Optional(CONF_IMPULSE_MAX, default=8): cv.int_range(min=3, max=20),
            # crawl acoustic knobs: lower duty = softer clack (0.72 just above
            # worst breakaway); smaller max width = smaller per-clack lurch.
            cv.Optional(CONF_CRAWL_IMPULSE_DUTY, default=0.80): cv.float_range(min=0.6, max=0.9),
            cv.Optional(CONF_CRAWL_MAX_WIDTH_MS, default=20.0): cv.float_range(min=2.0, max=50.0),
            # self-adjusting crawl step (default 0.0 = disabled, legacy dosing):
            # constant small per-pulse travel, width self-corrects from measured
            # travel of the previous pulse (regular, quiet, table-error robust).
            cv.Optional(CONF_CRAWL_STEP, default=0.0): cv.float_range(min=0.0, max=3.0),
            cv.Optional(CONF_CRAWL_W_GAIN, default=0.30): cv.float_range(min=0.0, max=1.0),
            # crawl-exclusive inter-pulse settle (0 = endgame short/long default);
            # shorter = tighter cadence / smaller steps at a given rate (smoothness).
            cv.Optional(CONF_CRAWL_SETTLE_MS, default=0): cv.int_range(min=0, max=1000),
            # crawl carries to within this %-of-target before the endgame seats
            # (0 = endgame_zone; small = no settling lunge/judder)
            cv.Optional(CONF_CRAWL_LAND_ZONE, default=0.0): cv.float_range(min=0.0, max=3.5),
            # GLIDE (glide-mode-spec.md): continuous keep-rolling slow mode.
            # glide=0 (default) => slow moves use CRAWL, byte-identical. glide>0
            # routes them to State::GLIDE (never stop, run at glide_floor).
            cv.Optional(CONF_GLIDE, default=0.0): cv.float_range(min=0.0, max=1.0),
            cv.Optional(CONF_GLIDE_FLOOR, default=40.0): cv.float_range(min=10.0, max=120.0),
            cv.Optional(CONF_GLIDE_STEP, default=0.4): cv.float_range(min=0.1, max=3.0),
            cv.Optional(CONF_GLIDE_SETTLE_MS, default=3): cv.int_range(min=0, max=200),
            cv.Optional(CONF_V_ROLL, default=30.0): cv.float_range(min=5.0, max=120.0),
            cv.Optional(CONF_GLIDE_PROG_WIN, default=12): cv.int_range(min=2, max=64),
            cv.Optional(CONF_GLIDE_RUNAWAY_V, default=200.0): cv.float_range(min=50.0, max=500.0),
            cv.Optional(CONF_GLIDE_MAX_GAP_MS, default=20): cv.int_range(min=4, max=200),
            cv.Optional(CONF_COAST_ZONE, default=0.8): cv.float_range(min=0.0, max=3.5),
            cv.Optional(CONF_THERMAL_AVG_MAX, default=0.50): cv.float_range(min=0.1, max=1.0),
            cv.Optional(CONF_U_MAX, default=0.90): cv.float_range(min=0.3, max=1.0),
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
        raise cv.Invalid(f"slider_servo: ESP32 variant {variant} is not supported")
    pin = config[CONF_ADC_PIN]
    if pin not in table:
        raise cv.Invalid(f"slider_servo: {variant} has no ADC1 channel on GPIO{pin}")

    cg.add(var.set_adc_channel(table[pin]))
    cg.add(var.set_pins(config[CONF_AIN1_PIN], config[CONF_AIN2_PIN], config[CONF_SYNC_PIN]))
    cg.add(var.set_http_port(config[CONF_HTTP_PORT]))
    cg.add(var.set_invert_direction(config[CONF_INVERT_DIRECTION]))
    cg.add(var.set_raw_endpoints(config[CONF_RAW_AT_0], config[CONF_RAW_AT_100]))
    cg.add(var.set_recorder_capacity(config[CONF_RECORDER_CAPACITY]))
    cg.add(var.set_tol(config[CONF_TOL]))
    cg.add(var.set_v_cruise(config[CONF_V_CRUISE]))
    cg.add(var.set_v_min_cmd(config[CONF_V_MIN_CMD]))
    cg.add(var.set_impulse_settle_ms(config[CONF_IMPULSE_SETTLE_MS]))
    cg.add(var.set_impulse_settle_short_ms(config[CONF_IMPULSE_SETTLE_SHORT_MS]))
    cg.add(var.set_impulse_aim(config[CONF_IMPULSE_AIM]))
    cg.add(var.set_impulse_max(config[CONF_IMPULSE_MAX]))
    cg.add(var.set_crawl_impulse_duty(config[CONF_CRAWL_IMPULSE_DUTY]))
    cg.add(var.set_crawl_max_width_ms(config[CONF_CRAWL_MAX_WIDTH_MS]))
    cg.add(var.set_crawl_step(config[CONF_CRAWL_STEP]))
    cg.add(var.set_crawl_w_gain(config[CONF_CRAWL_W_GAIN]))
    cg.add(var.set_crawl_settle_ms(config[CONF_CRAWL_SETTLE_MS]))
    cg.add(var.set_crawl_land_zone(config[CONF_CRAWL_LAND_ZONE]))
    cg.add(var.set_glide(config[CONF_GLIDE]))
    cg.add(var.set_glide_floor(config[CONF_GLIDE_FLOOR]))
    cg.add(var.set_glide_step(config[CONF_GLIDE_STEP]))
    cg.add(var.set_glide_settle_ms(config[CONF_GLIDE_SETTLE_MS]))
    cg.add(var.set_v_roll(config[CONF_V_ROLL]))
    cg.add(var.set_glide_prog_win(config[CONF_GLIDE_PROG_WIN]))
    cg.add(var.set_glide_runaway_v(config[CONF_GLIDE_RUNAWAY_V]))
    cg.add(var.set_glide_max_gap_ms(config[CONF_GLIDE_MAX_GAP_MS]))
    cg.add(var.set_coast_zone(config[CONF_COAST_ZONE]))
    cg.add(var.set_thermal_avg_max(config[CONF_THERMAL_AVG_MAX]))
    cg.add(var.set_u_max(config[CONF_U_MAX]))
