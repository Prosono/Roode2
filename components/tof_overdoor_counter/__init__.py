import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_FREQUENCY, CONF_ID, CONF_NUMBER, CONF_SCL, CONF_SDA, CONF_TIMEOUT
import esphome.pins as pins

tof_overdoor_counter_ns = cg.esphome_ns.namespace("tof_overdoor_counter")
TofOverdoorCounter = tof_overdoor_counter_ns.class_("TofOverdoorCounter", cg.PollingComponent)

CONF_BASE_ADDRESS = "base_address"
CONF_COOLDOWN = "cooldown"
CONF_INIT_RETRIES = "init_retries"
CONF_INVERT_DIRECTION = "invert_direction"
CONF_POST_ADDRESS_DELAY = "post_address_delay"
CONF_RELEASE_DELTA = "release_delta"
CONF_SEQUENCE_TIMEOUT = "sequence_timeout"
CONF_TRIGGER_DELTA = "trigger_delta"
CONF_WAKE_DELAY = "wake_delay"
CONF_XSHUT_PINS = "xshut_pins"


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
            cv.Optional(CONF_INIT_RETRIES, default=3): cv.int_range(min=1, max=5),
            cv.Optional(CONF_TRIGGER_DELTA, default="350mm"): cv.distance,
            cv.Optional(CONF_RELEASE_DELTA, default="220mm"): cv.distance,
            cv.Optional(CONF_SEQUENCE_TIMEOUT, default="2s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_COOLDOWN, default="600ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_INVERT_DIRECTION, default=False): cv.boolean,
            cv.Required(CONF_XSHUT_PINS): validate_xshut_pins,
        }
    )
    .extend(cv.polling_component_schema("120ms"))
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
    cg.add(var.set_init_retries(config[CONF_INIT_RETRIES]))
    cg.add(var.set_trigger_delta_mm(config[CONF_TRIGGER_DELTA]))
    cg.add(var.set_release_delta_mm(config[CONF_RELEASE_DELTA]))
    cg.add(var.set_sequence_timeout_ms(config[CONF_SEQUENCE_TIMEOUT]))
    cg.add(var.set_cooldown_ms(config[CONF_COOLDOWN]))
    cg.add(var.set_invert_direction(config[CONF_INVERT_DIRECTION]))

    for pin_config in config[CONF_XSHUT_PINS]:
        pin = await cg.gpio_pin_expression(pin_config)
        cg.add(var.add_xshut_pin(pin, pin_config[CONF_NUMBER]))
