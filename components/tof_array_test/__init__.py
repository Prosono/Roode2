import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_FREQUENCY, CONF_ID, CONF_NUMBER, CONF_SCL, CONF_SDA, CONF_TIMEOUT
import esphome.pins as pins

AUTO_LOAD = ["sensor", "text_sensor", "button"]

tof_array_test_ns = cg.esphome_ns.namespace("tof_array_test")
TofArrayTest = tof_array_test_ns.class_("TofArrayTest", cg.PollingComponent)
SensorDistanceMode = tof_array_test_ns.enum("SensorDistanceMode")
SensorProbeMode = tof_array_test_ns.enum("SensorProbeMode")

CONF_BASE_ADDRESS = "base_address"
CONF_CENTER = "center"
CONF_DISTANCE_MODE = "distance_mode"
CONF_HEIGHT = "height"
CONF_INIT_RETRIES = "init_retries"
CONF_INTERMEASUREMENT_PERIOD = "intermeasurement_period"
CONF_POST_ADDRESS_DELAY = "post_address_delay"
CONF_PROBE_MODE = "probe_mode"
CONF_ROI = "roi"
CONF_TIMING_BUDGET = "timing_budget"
CONF_WAKE_DELAY = "wake_delay"
CONF_WIDTH = "width"
CONF_XSHUT_PINS = "xshut_pins"

DISTANCE_MODE_SHORT = "short"
DISTANCE_MODE_LONG = "long"
DISTANCE_MODES = {
    DISTANCE_MODE_SHORT: SensorDistanceMode.DISTANCE_MODE_SHORT,
    DISTANCE_MODE_LONG: SensorDistanceMode.DISTANCE_MODE_LONG,
}

PROBE_MODE_FULL_INIT = "full_init"
PROBE_MODE_MICRO_PROBE = "micro_probe"
PROBE_MODES = {
    PROBE_MODE_FULL_INIT: SensorProbeMode.PROBE_MODE_FULL_INIT,
    PROBE_MODE_MICRO_PROBE: SensorProbeMode.PROBE_MODE_MICRO_PROBE,
}


def frequency_as_hz(value):
    return int(cv.frequency(value))


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TofArrayTest),
            cv.Optional(CONF_SDA, default=21): cv.int_range(min=0, max=39),
            cv.Optional(CONF_SCL, default=22): cv.int_range(min=0, max=39),
            cv.Optional(CONF_FREQUENCY, default="400kHz"): frequency_as_hz,
            cv.Optional(CONF_TIMEOUT, default="1500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_BASE_ADDRESS, default=0x30): cv.int_range(min=0x08, max=0x77),
            cv.Optional(CONF_WAKE_DELAY, default="20ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_POST_ADDRESS_DELAY, default="30ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_DISTANCE_MODE, default=DISTANCE_MODE_LONG): cv.enum(DISTANCE_MODES, lower=True),
            cv.Optional(CONF_PROBE_MODE, default=PROBE_MODE_FULL_INIT): cv.enum(PROBE_MODES, lower=True),
            cv.Optional(CONF_TIMING_BUDGET, default="33ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_INTERMEASUREMENT_PERIOD, default="37ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_INIT_RETRIES, default=3): cv.int_range(min=1, max=5),
            cv.Required(CONF_XSHUT_PINS): cv.ensure_list(pins.gpio_output_pin_schema),
            cv.Optional(CONF_ROI, default={}): cv.Schema(
                {
                    cv.Optional(CONF_WIDTH, default=16): cv.int_range(min=4, max=16),
                    cv.Optional(CONF_HEIGHT, default=16): cv.int_range(min=4, max=16),
                    cv.Optional(CONF_CENTER, default=199): cv.uint8_t,
                }
            ),
        }
    )
    .extend(cv.polling_component_schema("500ms"))
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
    cg.add(var.set_probe_mode(config[CONF_PROBE_MODE]))
    cg.add(var.set_timing_budget_ms(config[CONF_TIMING_BUDGET]))
    cg.add(var.set_intermeasurement_ms(config[CONF_INTERMEASUREMENT_PERIOD]))
    cg.add(var.set_init_retries(config[CONF_INIT_RETRIES]))

    roi = config[CONF_ROI]
    cg.add(var.set_roi(roi[CONF_WIDTH], roi[CONF_HEIGHT], roi[CONF_CENTER]))

    for pin_config in config[CONF_XSHUT_PINS]:
        pin = await cg.gpio_pin_expression(pin_config)
        cg.add(var.add_xshut_pin(pin, pin_config[CONF_NUMBER]))
