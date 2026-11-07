"""slider_servo — the ESPHome servo component (motorslider project).

The delivered product: the pure-C++ control core (spec §2 graph + EKF observer
+ endgame + CRAWL + L1) wrapped as an ESPHome external component. This codegen
wires the config to the HAL wrapper (slider_servo.cpp). The core is never
touched here — it is the same code the SIL harness (sil/) validates.

Multi-instance (kueche-mief-spec R1): `slider_servo:` is MULTI_CONF — a single
mapping (the rig) or a list of up to 4 sliders. Why MULTI_CONF and not a hub
block with a `servos:` list: the rig YAML stays byte-identical, every instance
keeps the familiar `id(servo).command_target(x)` surface, and the shared
resources (ADC1 unit, GPTimer tick, httpd, arbiter) live in ONE implicit hub
object that codegen creates once — there is nothing hub-level a user needs to
configure except http_port, which all instances must agree on.

Drive stack is identical to slider_lab (bench-proven): ADC1 oneshot 13 bit
12 dB, H-bridge AIN1/AIN2 at 19531 Hz, FAST = brake-low decay.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome import pins
from esphome.const import CONF_ID
from esphome.core import CORE, ID
from esphome.components.esp32 import get_esp32_variant
from esphome.components.esp32.const import (
    VARIANT_ESP32,
    VARIANT_ESP32S2,
    VARIANT_ESP32S3,
)

CODEOWNERS = ["@jan"]
DEPENDENCIES = ["esp32"]

MAX_INSTANCES = 4  # == SliderServoHub::kMaxInstances
MULTI_CONF = MAX_INSTANCES
LEDC_CHANNELS = 8  # LEDC low-speed channels on S2/S3 (and per speed mode on ESP32)

CONF_INDEX = "index"
CONF_ADC_PIN = "adc_pin"
CONF_AIN1_PIN = "ain1_pin"
CONF_AIN2_PIN = "ain2_pin"
CONF_SYNC_PIN = "sync_pin"
CONF_HTTP_PORT = "http_port"
CONF_IDLE_PATH = "idle_path"
CONF_IDLE_SAMPLE_DIVIDER = "idle_sample_divider"
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
CONF_THERMAL_TAU_S = "thermal_tau_s"
CONF_THERMAL_AVG_MAX = "thermal_avg_max"
CONF_THERMAL_RESUME = "thermal_resume"
CONF_U_MAX = "u_max"

IDLE_PATH_AUTO = "auto"
IDLE_PATH_FULL = "full"
IDLE_PATH_CHEAP = "cheap"

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
SliderServoHub = slider_servo_ns.class_("SliderServoHub")

HUB_ID = "slider_servo_hub"
_DATA_HUB = "slider_servo_hub_var"
_DATA_SLOT = "slider_servo_next_slot"


def adc1_channel(pin):
    """GPIO -> ADC1 channel for the configured variant (raises cv.Invalid)."""
    variant = get_esp32_variant()
    table = ADC1_PIN_TO_CHANNEL.get(variant)
    if table is None:
        raise cv.Invalid(f"slider_servo: ESP32 variant {variant} is not supported")
    if pin not in table:
        raise cv.Invalid(f"slider_servo: {variant} has no ADC1 channel on GPIO{pin}")
    return table[pin]


async def get_hub():
    """The one shared hub (ADC1 owner, tick, httpd, arbiter), created lazily."""
    if _DATA_HUB not in CORE.data:
        hub_id = ID(HUB_ID, is_declaration=True, type=SliderServoHub)
        CORE.data[_DATA_HUB] = cg.new_Pvariable(hub_id)
    return CORE.data[_DATA_HUB]


def _validate_thermal(config):
    if config[CONF_THERMAL_RESUME] >= config[CONF_THERMAL_AVG_MAX]:
        raise cv.Invalid(
            f"{CONF_THERMAL_RESUME} ({config[CONF_THERMAL_RESUME]}) must be below "
            f"{CONF_THERMAL_AVG_MAX} ({config[CONF_THERMAL_AVG_MAX]}) — hysteresis"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SliderServo),
            # NVS slot: key 'SVRC' + index. Default = position in the list, so a
            # single-instance config (the rig) is index 0 == its existing blob.
            # Set it explicitly for multi-instance configs: reordering the YAML
            # must not swap two sliders' identified calibrations.
            cv.Optional(CONF_INDEX): cv.int_range(min=0, max=MAX_INSTANCES - 1),
            cv.Optional(CONF_ADC_PIN, default=9): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_AIN1_PIN, default=11): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_AIN2_PIN, default=10): pins.internal_gpio_output_pin_number,
            # optional scope-sync output (HIGH while driving). Omit it where the
            # pin is taken (kueche-mief: GPIO14 = I2C SCL).
            cv.Optional(CONF_SYNC_PIN): pins.internal_gpio_output_pin_number,
            # one httpd for all instances: every instance must name the same port
            cv.Optional(CONF_HTTP_PORT, default=80): cv.port,
            # IDLE non-owner tick: "full" runs the whole core every tick (rig
            # behaviour), "cheap" only samples + publishes (R1 budget); "auto"
            # = full for one instance, cheap for several.
            cv.Optional(CONF_IDLE_PATH, default=IDLE_PATH_AUTO): cv.one_of(
                IDLE_PATH_AUTO, IDLE_PATH_FULL, IDLE_PATH_CHEAP, lower=True
            ),
            # cheap path samples every N ticks: 1 = 1 kHz ... 10 = 100 Hz (R1 floor)
            cv.Optional(CONF_IDLE_SAMPLE_DIVIDER, default=1): cv.int_range(min=1, max=10),
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
            # L1 thermal (Jan 2026-11-07, realistic trip): EMA over |duty| with
            # tau 120 s trips at 0.95 (~6 min continuous full duty; the hardware
            # takes 5 min), resumes below 0.60. Defaults == the core's new values.
            cv.Optional(CONF_THERMAL_TAU_S, default=120.0): cv.float_range(min=1.0, max=600.0),
            cv.Optional(CONF_THERMAL_AVG_MAX, default=0.95): cv.float_range(min=0.1, max=1.0),
            cv.Optional(CONF_THERMAL_RESUME, default=0.60): cv.float_range(min=0.05, max=1.0),
            cv.Optional(CONF_U_MAX, default=0.90): cv.float_range(min=0.3, max=1.0),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    _validate_thermal,
)


def _final_validate(config):
    """Cross-instance checks (runs per list entry, sees the whole list)."""
    full = fv.full_config.get()
    entries = full.get("slider_servo", [])
    if not isinstance(entries, list):
        entries = [entries]

    ports = {e[CONF_HTTP_PORT] for e in entries}
    if len(ports) > 1:
        raise cv.Invalid(f"slider_servo: all instances share ONE httpd — http_port differs: {sorted(ports)}")

    indices = [e.get(CONF_INDEX, pos) for pos, e in enumerate(entries)]
    if len(set(indices)) != len(indices):
        raise cv.Invalid(f"slider_servo: NVS index must be unique per instance, got {indices}")

    for e in entries:
        adc1_channel(e[CONF_ADC_PIN])  # raises if the wiper is not on ADC1

    # R2: slider_servo owns the ADC1 unit. A second owner (ESPHome `adc` on an
    # ADC1 pin) would fail adc_oneshot_new_unit at runtime — refuse it here.
    adc1_pins = ADC1_PIN_TO_CHANNEL.get(get_esp32_variant(), {})
    for s in full.get("sensor", []):
        if s.get("platform") != "adc":
            continue
        pin = s.get("pin")
        number = pin.get("number") if isinstance(pin, dict) else pin
        if number in adc1_pins:
            raise cv.Invalid(
                f"slider_servo owns ADC1: sensor '{s.get('name', s.get('id'))}' uses platform adc on "
                f"GPIO{number} (ADC1). Use `platform: slider_servo` with `type: adc` instead."
            )

    # The servo takes LEDC channels from the top (7/6, 5/4, ...) on the last
    # timer; ESPHome's ledc outputs allocate channel 0, 1, ... with timer =
    # channel / 2. They must stay below ours.
    outputs = full.get("output", [])
    ledc = [o for o in outputs if o.get("platform") == "ledc"]
    lowest_servo = LEDC_CHANNELS - 2 * len(entries)
    used = []
    for pos, o in enumerate(ledc):
        used.append(o.get("channel", pos))
    if used and max(used) >= lowest_servo:
        raise cv.Invalid(
            f"slider_servo: {len(entries)} instance(s) use LEDC channels {lowest_servo}..{LEDC_CHANNELS - 1}; "
            f"ESPHome ledc outputs occupy {sorted(used)} — too many ledc outputs for this board"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    hub = await get_hub()
    slot = CORE.data.get(_DATA_SLOT, 0)
    CORE.data[_DATA_SLOT] = slot + 1

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # how many instances in total decides the "auto" idle path
    entries = CORE.config.get("slider_servo", [])
    n_instances = len(entries) if isinstance(entries, list) else 1
    idle_path = config[CONF_IDLE_PATH]
    if idle_path == IDLE_PATH_AUTO:
        idle_path = IDLE_PATH_FULL if n_instances == 1 else IDLE_PATH_CHEAP

    cg.add(hub.register_servo(var))
    cg.add(hub.set_http_port(config[CONF_HTTP_PORT]))
    cg.add(var.set_hub(hub))
    cg.add(var.set_index(config.get(CONF_INDEX, slot)))
    cg.add(var.set_name_id(str(config[CONF_ID].id)))
    cg.add(var.set_adc_channel(adc1_channel(config[CONF_ADC_PIN])))
    cg.add(var.set_pins(config[CONF_AIN1_PIN], config[CONF_AIN2_PIN]))
    if CONF_SYNC_PIN in config:
        cg.add(var.set_sync_pin(config[CONF_SYNC_PIN]))
    cg.add(var.set_idle_full(idle_path == IDLE_PATH_FULL))
    cg.add(var.set_idle_sample_divider(config[CONF_IDLE_SAMPLE_DIVIDER]))
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
    cg.add(var.set_thermal_tau_s(config[CONF_THERMAL_TAU_S]))
    cg.add(var.set_thermal_avg_max(config[CONF_THERMAL_AVG_MAX]))
    cg.add(var.set_thermal_resume(config[CONF_THERMAL_RESUME]))
    cg.add(var.set_u_max(config[CONF_U_MAX]))
