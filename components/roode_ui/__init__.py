import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

from ..roode import Roode, CONF_ROODE_ID

AUTO_LOAD = ["json", "web_server_base"]
DEPENDENCIES = ["roode", "web_server"]

roode_ui_ns = cg.esphome_ns.namespace("roode_ui")
RoodeUi = roode_ui_ns.class_("RoodeUi", cg.Component)

CONF_NODES = "nodes"
CONF_LABEL = "label"
CONF_TITLE = "title"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RoodeUi),
        cv.Optional(CONF_TITLE): cv.string,
        cv.Required(CONF_NODES): cv.ensure_list(
            cv.Schema(
                {
                    cv.Required(CONF_LABEL): cv.string,
                    cv.Required(CONF_ROODE_ID): cv.use_id(Roode),
                }
            )
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_TITLE in config:
        cg.add(var.set_title(config[CONF_TITLE]))

    for node in config[CONF_NODES]:
        roode = await cg.get_variable(node[CONF_ROODE_ID])
        cg.add(var.add_node(roode, node[CONF_LABEL]))
