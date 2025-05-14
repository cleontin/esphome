import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, modbus
from esphome.const import *

AUTO_LOAD = ["sensor"]

hayward_ns = cg.esphome_ns.namespace('hayward')
Hayward = hayward_ns.class_('Hayward', cg.Component, modbus.ModbusServer)

CONF_SUCTION_TEMPERATURE = "suction_temperature"
CONF_INLET_TEMPERATURE = "inlet_temperature"
CONF_OUTLET_TEMPERATURE = "outlet_temperature"
CONF_COIL1_TEMPERATURE = "coil1_temperature"
CONF_COIL2_TEMPERATURE = "coil2_temperature"
CONF_AMBIENT_TEMPERATURE = "ambient_temperature"
CONF_EXHAUST_TEMPERATURE = "exhaust_temperature"
CONF_COMPRESSOR_CURRENT_DETECTION = "compressor_current_detection"
CONF_AC_FAN_OUTPUT = "ac_fan_output"
CONF_FLOW_RATE_INPUT = "flow_rate_input"
CONF_PRESSURE_SENSOR = "pressure_sensor"
CONF_SUPER_HEAT = "super_heat"
CONF_TARGET_SPEED_FAN_MOTOR = "target_speed_fan_motor"
CONF_OVER_HEAT_AFTER_COMMPEN = "over_heat_after_commpen"
CONF_INVERTER_PLATE_AC_VOLTAGE = "inverter_plate_ac_voltage"
CONF_ANTI_FREEZE_TEMP = "anti_freeze_temp"
CONF_EC_FAN_MOTOR_SPEED = "ec_fan_motor_speed"
CONF_SPEED_FAN_MOTOR_1 = "speed_fan_motor_1"
CONF_SPEED_FAN_MOTOR_2 = "speed_fan_motor_2"
CONF_BUSES_VOLTAGE = "buses_voltage"
CONF_LIMITED_FREQUENCY_PROTECT_STATE = "limited_frequency_protect_state"
CONF_REDUCTION_PROTECT_STATE = "reduction_protect_state"
CONF_DRIVER_BOARD_RUNNING_STATE_1 = "driver_board_running_state_1"
CONF_DRIVER_BOARD_RUNNING_STATE_2 = "driver_board_running_state_2"
CONF_DRIVER_BOARD_RUNNING_STATE_3 = "driver_board_running_state_3"
CONF_DRIVER_BOARD_RUNNING_STATE_4 = "driver_board_running_state_4"
CONF_DRIVER_BOARD_RUNNING_STATE_5 = "driver_board_running_state_5"

# Define all sensors here
SENSORS = [
    {
        "key": "suction_temperature",
        "name": "Suction Temperature",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_CELSIUS,
        "icon": ICON_THERMOMETER,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_TEMPERATURE,
        "setter": "set_suction_temperature",
    },
    {
        "key": "inlet_temperature",
        "name": "Inlet water",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_CELSIUS,
        "icon": ICON_THERMOMETER,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_TEMPERATURE,
        "setter": "set_inlet_temperature",
    },
    {
        "key": "outlet_temperature",
        "name": "Outlet Water",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_CELSIUS,
        "icon": ICON_THERMOMETER,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_TEMPERATURE,
        "setter": "set_outlet_temperature",
    },
    {
        "key": "coil1_temperature",
        "name": "Coil 1",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_CELSIUS,
        "icon": ICON_THERMOMETER,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_TEMPERATURE,
        "setter": "set_coil1_temperature",
    },
    {
        "key": "ambient_temperature",
        "name": "Ambient",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_CELSIUS,
        "icon": ICON_THERMOMETER,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_TEMPERATURE,
        "setter": "set_ambient_temperature",
    },
    {
        "key": "exhaust_temperature",
        "name": "Exhaust",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_CELSIUS,
        "icon": ICON_THERMOMETER,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_TEMPERATURE,
        "setter": "set_exhaust_temperature",
    },
    {
        "key": "compressor_current",
        "name": "Compressor Current",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_AMPERE,
        "icon": ICON_CURRENT_AC,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_CURRENT,
        "setter": "set_compressor_current",
    },
    {
        "key": "ac_fan_output",
        "name": "AC Fan Output",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_PERCENT,
        "icon": ICON_PERCENT,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_POWER_FACTOR,
        "setter": "set_ac_fan_output",
    },
    {
        "key": "target_speed_fan_motor",
        "name": "Target Speed Fan Motor",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_REVOLUTIONS_PER_MINUTE,
        "icon": ICON_FAN,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": "",
        "setter": "set_target_speed_fan_motor",
    },
    {
        "key": "inverter_plate_ac_voltage",
        "name": "Inverter Plate AC Voltage",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_VOLT,
        "icon": ICON_POWER,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "setter": "set_inverter_plate_ac_voltage",
    },
    {
        "key": "speed_fan_motor_1",
        "name": "Speed Fan Motor 1",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_REVOLUTIONS_PER_MINUTE,
        "icon": ICON_FAN,
        "accuracy_decimals": 0,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": "",
        "setter": "set_speed_fan_motor_1",
    },
    {
        "key": "super_heat",
        "name": "Super Heat",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_CELSIUS,
        "icon": ICON_THERMOMETER,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_TEMPERATURE,
        "setter": "set_super_heat_temperature",
    },
    {
        "key": "over_heat_after_commpen",
        "name": "Overheat After Compensation",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_CELSIUS,
        "icon": ICON_THERMOMETER,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_TEMPERATURE,
        "setter": "set_overheat_after_commpen",
    },
    {
        "key": "anti_freeze_temp",
        "name": "Anti-Freeze Temperature",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_CELSIUS,
        "icon": ICON_THERMOMETER,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_TEMPERATURE,
        "setter": "set_anti_freeze_temp",
    },
    {
        "key": "coil2_temperature",
        "name": "Coil 2",
        "disabled_by_default": False,
        "unit_of_measurement": UNIT_CELSIUS,
        "icon": ICON_THERMOMETER,
        "accuracy_decimals": 1,
        "state_class": STATE_CLASS_MEASUREMENT,
        "device_class": DEVICE_CLASS_TEMPERATURE,
        "setter": "set_coil2_temperature",
    },
]


CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Hayward),
}).extend({
    cv.Optional(s["key"], default={}): sensor.sensor_schema(
        unit_of_measurement=s["unit_of_measurement"],
        icon=s["icon"],
        accuracy_decimals=s["accuracy_decimals"],
        state_class=s["state_class"],
        device_class=s["device_class"],
    ).extend({
        cv.Optional("name", default=s["name"]): cv.string,
        cv.Optional("disabled_by_default", default=s["disabled_by_default"]): cv.boolean,
    })
    for s in SENSORS
}).extend(cv.COMPONENT_SCHEMA).extend(modbus.modbus_server_schema())

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield modbus.register_modbus_server(var, config)

    # Register all sensors with their configurations
    for s in SENSORS:
        conf = config[s["key"]]
        sens = yield sensor.new_sensor(conf)
        cg.add(getattr(var, s["setter"])(sens))

