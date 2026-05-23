import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_FREQUENCY, CONF_ID, CONF_NUMBER, CONF_SCL, CONF_SDA, CONF_TIMEOUT
import esphome.pins as pins

tof_overdoor_counter_ns = cg.esphome_ns.namespace("tof_overdoor_counter")
TofOverdoorCounter = tof_overdoor_counter_ns.class_("TofOverdoorCounter", cg.PollingComponent)
OperatingMode = tof_overdoor_counter_ns.enum("OperatingMode")
SensorDistanceMode = tof_overdoor_counter_ns.enum("SensorDistanceMode")

CONF_BASE_ADDRESS = "base_address"
CONF_BASELINE_TOLERANCE = "baseline_tolerance"
CONF_BLOCKED_TIMEOUT = "blocked_timeout"
CONF_CALIBRATION_SAMPLES = "calibration_samples"
CONF_COOLDOWN = "cooldown"
CONF_DEBOUNCE = "debounce"
CONF_DISTANCE_MODE = "distance_mode"
CONF_DEBUG_LOGGING = "debug_logging"
CONF_DEBUG_SAMPLE_INTERVAL = "debug_sample_interval"
CONF_INIT_RETRIES = "init_retries"
CONF_INVERT_DIRECTION = "invert_direction"
CONF_INTERMEASUREMENT = "intermeasurement_period"
CONF_MIN_ACTIVE_DURATION = "min_active_duration"
CONF_MIN_EVENT_SENSORS = "min_event_sensors"
CONF_MAX_PEOPLE_INSIDE = "max_people_inside"
CONF_MINIMUM_CLEAR_DISTANCE = "minimum_clear_distance"
CONF_MIN_VALID_SENSORS = "min_valid_sensors"
CONF_MODE = "mode"
CONF_POST_ADDRESS_DELAY = "post_address_delay"
CONF_RELEASE_DELTA = "release_delta"
CONF_SAMPLING = "sampling"
CONF_SEQUENCE_TIMEOUT = "sequence_timeout"
CONF_STANDING_TIMEOUT = "standing_timeout"
CONF_TIMING_BUDGET = "timing_budget"
CONF_TRIGGER_DELTA = "trigger_delta"
CONF_DIRECTION_WINDOW = "direction_window"
CONF_WAKE_DELAY = "wake_delay"
CONF_XSHUT_PINS = "xshut_pins"
CONF_AUTO_SAVE_ENABLED = "auto_save_enabled"


MODE_OPTIONS = {
    "monitor": OperatingMode.MONITOR,
    "count": OperatingMode.COUNT,
}

DISTANCE_MODE_OPTIONS = {
    "short": SensorDistanceMode.DISTANCE_MODE_SHORT,
    "long": SensorDistanceMode.DISTANCE_MODE_LONG,
}


def frequency_as_hz(value):
    return int(cv.frequency(value))


def validate_xshut_pins(value):
    pins_list = cv.ensure_list(pins.gpio_output_pin_schema)(value)
    if len(pins_list) != 4:
        raise cv.Invalid("Exactly 4 XSHUT pins are required for the over-door counter")
    return pins_list


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TofOverdoorCounter),
            cv.Optional(CONF_SDA, default=21): cv.int_range(min=0, max=39),
            cv.Optional(CONF_SCL, default=22): cv.int_range(min=0, max=39),
            cv.Optional(CONF_FREQUENCY, default="100kHz"): frequency_as_hz,
            cv.Optional(CONF_TIMEOUT, default="1500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_BASE_ADDRESS, default=0x30): cv.int_range(min=0x08, max=0x77),
            cv.Optional(CONF_WAKE_DELAY, default="60ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_POST_ADDRESS_DELAY, default="80ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_DISTANCE_MODE, default="long"): cv.enum(DISTANCE_MODE_OPTIONS, lower=True),
            cv.Optional(CONF_TIMING_BUDGET, default="33ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_INTERMEASUREMENT, default="33ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_INIT_RETRIES, default=3): cv.int_range(min=1, max=5),
            cv.Optional(CONF_SAMPLING, default=3): cv.int_range(min=1, max=8),
            cv.Optional(CONF_TRIGGER_DELTA, default="350mm"): cv.distance,
            cv.Optional(CONF_RELEASE_DELTA, default="220mm"): cv.distance,
            cv.Optional(CONF_BASELINE_TOLERANCE, default="80mm"): cv.distance,
            cv.Optional(CONF_SEQUENCE_TIMEOUT, default="2s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_DEBOUNCE, default="45ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_COOLDOWN, default="600ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_BLOCKED_TIMEOUT, default="1800ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_STANDING_TIMEOUT, default="2200ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MIN_EVENT_SENSORS, default=2): cv.int_range(min=2, max=4),
            cv.Optional(CONF_MIN_ACTIVE_DURATION, default="35ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_DIRECTION_WINDOW, default="90ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MINIMUM_CLEAR_DISTANCE, default="600mm"): cv.distance,
            cv.Optional(CONF_CALIBRATION_SAMPLES, default=24): cv.int_range(min=8, max=100),
            cv.Optional(CONF_MIN_VALID_SENSORS, default=3): cv.int_range(min=2, max=4),
            cv.Optional(CONF_MAX_PEOPLE_INSIDE, default=50): cv.int_range(min=1, max=500),
            cv.Optional(CONF_AUTO_SAVE_ENABLED, default=True): cv.boolean,
            cv.Optional(CONF_INVERT_DIRECTION, default=False): cv.boolean,
            cv.Optional(CONF_DEBUG_LOGGING, default=False): cv.boolean,
            cv.Optional(CONF_DEBUG_SAMPLE_INTERVAL, default="250ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MODE, default="count"): cv.enum(MODE_OPTIONS, lower=True),
            cv.Required(CONF_XSHUT_PINS): validate_xshut_pins,
        }
    )
    .extend(cv.polling_component_schema("10ms"))
)


async def to_code(config):
    cg.add_library("Wire", None)
    cg.add_library("rneurink", "1.2.3", "VL53L1X_ULD")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_sda_pin(config[CONF_SDA]))
    cg.add(var.set_scl_pin(config[CONF_SCL]))
    cg.add(var.set_i2c_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_timeout_ms(config[CONF_TIMEOUT]))
    cg.add(var.set_base_address(config[CONF_BASE_ADDRESS]))
    cg.add(var.set_wake_delay_ms(config[CONF_WAKE_DELAY]))
    cg.add(var.set_post_address_delay_ms(config[CONF_POST_ADDRESS_DELAY]))
    cg.add(var.set_distance_mode(config[CONF_DISTANCE_MODE]))
    cg.add(var.set_timing_budget_ms(config[CONF_TIMING_BUDGET]))
    cg.add(var.set_intermeasurement_ms(config[CONF_INTERMEASUREMENT]))
    cg.add(var.set_init_retries(config[CONF_INIT_RETRIES]))
    cg.add(var.set_sampling_size(config[CONF_SAMPLING]))
    cg.add(var.set_trigger_delta_mm(config[CONF_TRIGGER_DELTA]))
    cg.add(var.set_release_delta_mm(config[CONF_RELEASE_DELTA]))
    cg.add(var.set_baseline_tolerance_mm(config[CONF_BASELINE_TOLERANCE]))
    cg.add(var.set_sequence_timeout_ms(config[CONF_SEQUENCE_TIMEOUT]))
    cg.add(var.set_debounce_ms(config[CONF_DEBOUNCE]))
    cg.add(var.set_cooldown_ms(config[CONF_COOLDOWN]))
    cg.add(var.set_blocked_timeout_ms(config[CONF_BLOCKED_TIMEOUT]))
    cg.add(var.set_standing_timeout_ms(config[CONF_STANDING_TIMEOUT]))
    cg.add(var.set_min_event_sensors(config[CONF_MIN_EVENT_SENSORS]))
    cg.add(var.set_min_active_duration_ms(config[CONF_MIN_ACTIVE_DURATION]))
    cg.add(var.set_direction_window_ms(config[CONF_DIRECTION_WINDOW]))
    cg.add(var.set_minimum_clear_distance_mm(config[CONF_MINIMUM_CLEAR_DISTANCE]))
    cg.add(var.set_calibration_samples(config[CONF_CALIBRATION_SAMPLES]))
    cg.add(var.set_min_valid_sensors(config[CONF_MIN_VALID_SENSORS]))
    cg.add(var.set_max_people_inside(config[CONF_MAX_PEOPLE_INSIDE]))
    cg.add(var.set_auto_save_enabled(config[CONF_AUTO_SAVE_ENABLED]))
    cg.add(var.set_invert_direction(config[CONF_INVERT_DIRECTION]))
    cg.add(var.set_debug_logging(config[CONF_DEBUG_LOGGING]))
    cg.add(var.set_debug_sample_interval_ms(config[CONF_DEBUG_SAMPLE_INTERVAL]))
    cg.add(var.set_mode(config[CONF_MODE]))

    for pin_config in config[CONF_XSHUT_PINS]:
        pin = await cg.gpio_pin_expression(pin_config)
        cg.add(var.add_xshut_pin(pin, pin_config[CONF_NUMBER]))
