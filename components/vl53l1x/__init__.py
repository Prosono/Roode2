import logging
from typing import Dict, Any
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_FREQUENCY,
    CONF_ID,
    CONF_INTERRUPT,
    CONF_OFFSET,
    CONF_PINS,
    CONF_TIMEOUT,
)
import esphome.pins as pins

_LOGGER = logging.getLogger(__name__)

MULTI_CONF = True

vl53l1x_ns = cg.esphome_ns.namespace("vl53l1x")
VL53L1X = vl53l1x_ns.class_("VL53L1X", cg.Component)

CONF_AUTO = "auto"
CONF_CALIBRATION = "calibration"
CONF_RANGING_MODE = "ranging"
CONF_SCL_PIN = "scl_pin"
CONF_SDA_PIN = "sda_pin"
CONF_XSHUT = "xshut"
CONF_XTALK = "crosstalk"

Ranging = vl53l1x_ns.namespace("Ranging")
RANGING_MODES = {
    CONF_AUTO: CONF_AUTO,
    "shortest": Ranging.Shortest,
    "short": Ranging.Short,
    "medium": Ranging.Medium,
    "long": Ranging.Long,
    "longer": Ranging.Longer,
    "longest": Ranging.Longest,
}

int16_t = cv.int_range(min=-32768, max=32768)  # signed


def distance_as_mm(value):
    meters = cv.distance(value)
    return int(meters * 1000)


def frequency_as_hz(value):
    return int(cv.frequency(value))


def int_with_unit(*args, **kwargs):
    validator = cv.float_with_unit(*args, **kwargs)

    def int_validator(val):
        return int(validator(val))

    return int_validator


def NullableSchema(*args, default: Any = None, **kwargs):
    """
    Same as Schema but will convert nulls to empty objects. Useful when all the schema keys are optional.
    Allows YAML lines to be commented out leaving an "empty dict" which is mistakenly parsed as None.
    """

    def none_to_empty(value):
        if value is None:
            return {} if default is None else default
        raise cv.Invalid("Expected none")

    return cv.Any(cv.Schema(*args, **kwargs), none_to_empty)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(VL53L1X),
            cv.Optional(CONF_ADDRESS, default=0x29): cv.int_range(min=0x08, max=0x77),
            cv.Optional(CONF_SDA_PIN, default=21): cv.int_range(min=0, max=39),
            cv.Optional(CONF_SCL_PIN, default=22): cv.int_range(min=0, max=39),
            cv.Optional(CONF_FREQUENCY, default="400kHz"): frequency_as_hz,
            cv.Optional(
                CONF_TIMEOUT, default="2s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_PINS, default={}): NullableSchema(
                {
                    cv.Optional(CONF_XSHUT): pins.gpio_output_pin_schema,
                    cv.Optional(CONF_INTERRUPT): pins.internal_gpio_input_pin_schema,
                }
            ),
            cv.Optional(CONF_CALIBRATION, default={}): NullableSchema(
                {
                    cv.Optional(CONF_RANGING_MODE, default=CONF_AUTO): cv.enum(
                        RANGING_MODES
                    ),
                    cv.Optional(CONF_XTALK): cv.All(
                        int_with_unit(
                            "corrected photon count as cps (counts per second)", "(cps)"
                        ),
                        cv.uint16_t,
                    ),
                    cv.Optional(CONF_OFFSET): cv.All(distance_as_mm, int16_t),
                }
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config: Dict):
    cg.add_library("Wire", None)  # Required for VL53L1X_ULD to find Wire.h
    cg.add_library("rneurink", "1.2.3", "VL53L1X_ULD")

    vl53l1x = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(vl53l1x, config)

    frequency = config[CONF_FREQUENCY]
    if frequency < 400000:
        _LOGGER.warning(
            "Recommended I2C frequency for VL53L1X is 400kHz. Currently: %dkHz",
            frequency / 1000,
        )
    cg.add(vl53l1x.set_address(config[CONF_ADDRESS]))
    cg.add(vl53l1x.set_sda_pin(config[CONF_SDA_PIN]))
    cg.add(vl53l1x.set_scl_pin(config[CONF_SCL_PIN]))
    cg.add(vl53l1x.set_i2c_frequency(frequency))

    cg.add(vl53l1x.set_timeout(config[CONF_TIMEOUT]))
    await setup_hardware(vl53l1x, config)
    await setup_calibration(vl53l1x, config[CONF_CALIBRATION])


async def setup_hardware(vl53l1x: cg.Pvariable, config: Dict):
    pins = config[CONF_PINS]
    if CONF_INTERRUPT in pins:
        interrupt = await cg.gpio_pin_expression(pins[CONF_INTERRUPT])
        cg.add(vl53l1x.set_interrupt_pin(interrupt))
    if CONF_XSHUT in pins:
        xshut = await cg.gpio_pin_expression(pins[CONF_XSHUT])
        cg.add(vl53l1x.set_xshut_pin(xshut))


async def setup_calibration(vl53l1x: cg.Pvariable, config: Dict):
    if config.get(CONF_RANGING_MODE, CONF_AUTO) != CONF_AUTO:
        cg.add(vl53l1x.set_ranging_mode_override(config[CONF_RANGING_MODE]))
    if CONF_XTALK in config:
        cg.add(vl53l1x.set_xtalk(config[CONF_XTALK]))
    if CONF_OFFSET in config:
        cg.add(vl53l1x.set_offset(config[CONF_OFFSET]))
