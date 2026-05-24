import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, switch, text_sensor
from esphome.const import CONF_ID

AUTO_LOAD = ["json", "switch", "web_server_base"]
DEPENDENCIES = ["web_server"]

room_counter_ui_ns = cg.esphome_ns.namespace("room_counter_ui")
RoomCounterUi = room_counter_ui_ns.class_("RoomCounterUi", cg.Component)

CONF_CONFIRMED_IN_SENSOR = "confirmed_in_sensor"
CONF_CONFIRMED_OUT_SENSOR = "confirmed_out_sensor"
CONF_AGGREGATE_INVERT_SWITCH = "aggregate_invert_switch"
CONF_LABEL = "label"
CONF_LAST_EVENT = "last_event"
CONF_LAST_REASON = "last_reason"
CONF_PEOPLE_SENSOR = "people_sensor"
CONF_REJECTED_EVENTS_SENSOR = "rejected_events_sensor"
CONF_TITLE = "title"
CONF_VOTE_WINDOW = "vote_window"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RoomCounterUi),
        cv.Optional(CONF_TITLE): cv.string,
        cv.Optional(CONF_LABEL, default="People in room"): cv.string,
        cv.Required(CONF_PEOPLE_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_CONFIRMED_IN_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_CONFIRMED_OUT_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_AGGREGATE_INVERT_SWITCH): cv.use_id(switch.Switch),
        cv.Optional(CONF_REJECTED_EVENTS_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_LAST_EVENT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_LAST_REASON): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_VOTE_WINDOW): cv.use_id(text_sensor.TextSensor),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_TITLE in config:
        cg.add(var.set_title(config[CONF_TITLE]))
    cg.add(var.set_label(config[CONF_LABEL]))

    people_sensor = await cg.get_variable(config[CONF_PEOPLE_SENSOR])
    cg.add(var.set_people_sensor(people_sensor))

    if CONF_CONFIRMED_IN_SENSOR in config:
        confirmed_in = await cg.get_variable(config[CONF_CONFIRMED_IN_SENSOR])
        cg.add(var.set_confirmed_in_sensor(confirmed_in))
    if CONF_CONFIRMED_OUT_SENSOR in config:
        confirmed_out = await cg.get_variable(config[CONF_CONFIRMED_OUT_SENSOR])
        cg.add(var.set_confirmed_out_sensor(confirmed_out))
    if CONF_AGGREGATE_INVERT_SWITCH in config:
        aggregate_invert = await cg.get_variable(config[CONF_AGGREGATE_INVERT_SWITCH])
        cg.add(var.set_aggregate_invert_switch(aggregate_invert))
    if CONF_REJECTED_EVENTS_SENSOR in config:
        rejected = await cg.get_variable(config[CONF_REJECTED_EVENTS_SENSOR])
        cg.add(var.set_rejected_events_sensor(rejected))
    if CONF_LAST_EVENT in config:
        last_event = await cg.get_variable(config[CONF_LAST_EVENT])
        cg.add(var.set_last_event(last_event))
    if CONF_LAST_REASON in config:
        last_reason = await cg.get_variable(config[CONF_LAST_REASON])
        cg.add(var.set_last_reason(last_reason))
    if CONF_VOTE_WINDOW in config:
        vote_window = await cg.get_variable(config[CONF_VOTE_WINDOW])
        cg.add(var.set_vote_window(vote_window))
