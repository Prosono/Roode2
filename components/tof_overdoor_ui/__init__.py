import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

from ..tof_overdoor_counter import TofOverdoorCounter

AUTO_LOAD = ["json", "web_server_base"]
DEPENDENCIES = ["tof_overdoor_counter", "web_server"]

tof_overdoor_ui_ns = cg.esphome_ns.namespace("tof_overdoor_ui")
TofOverdoorUi = tof_overdoor_ui_ns.class_("TofOverdoorUi", cg.Component)

CONF_COUNTER_ID = "counter_id"
CONF_LABEL = "label"
CONF_TITLE = "title"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TofOverdoorUi),
        cv.Optional(CONF_TITLE): cv.string,
        cv.Optional(CONF_LABEL, default="Doorway Counter"): cv.string,
        cv.Required(CONF_COUNTER_ID): cv.use_id(TofOverdoorCounter),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_TITLE in config:
        cg.add(var.set_title(config[CONF_TITLE]))
    cg.add(var.set_label(config[CONF_LABEL]))

    counter = await cg.get_variable(config[CONF_COUNTER_ID])
    cg.add(var.set_counter(counter))
