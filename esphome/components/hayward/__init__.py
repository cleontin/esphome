import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, modbus, switch, number, climate, homeassistant
from esphome.const import *

DEPENDENCIES = ["modbus", "time"]
AUTO_LOAD = ["sensor", "switch", "number", "climate", "homeassistant"]

hayward_ns = cg.esphome_ns.namespace('hayward')
homeassistant_ns = cg.esphome_ns.namespace('homeassistant')
HomeassistantTime = homeassistant_ns.class_('HomeassistantTime')

Hayward = hayward_ns.class_('Hayward', cg.Component, modbus.ModbusServer)
HaywardSwitch = hayward_ns.class_('HaywardSwitch', switch.Switch, cg.Component)
HaywardHour = hayward_ns.class_('HaywardHour', number.Number, cg.Component)
HaywardClimate = hayward_ns.class_('HaywardClimate', climate.Climate, cg.Component)


CONF_SUCTION_TEMPERATURE = "suction_temperature"
CONF_INLET_TEMPERATURE = "inlet_temperature"
CONF_OUTLET_TEMPERATURE = "outlet_temperature"
CONF_COIL1_TEMPERATURE = "coil1_temperature"
CONF_COIL2_TEMPERATURE = "coil2_temperature"
CONF_AMBIENT_TEMPERATURE = "ambient_temperature"
CONF_EXHAUST_TEMPERATURE = "exhaust_temperature"
CONF_COMPRESSOR_CURRENT = "compressor_current"
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

CONF_SILENT_ACTIVE = "silent_active"

CONF_SCHEDULE_SILENT_ACTIVE = "schedule_silent_active"
CONF_SCHEDULE_SILENT_START_HOUR = "schedule_silent_start_hour"
CONF_SCHEDULE_SILENT_STOP_HOUR = "schedule_silent_stop_hour"

CONF_SCHEDULE_ON_ACTIVE = "schedule_on_active"
CONF_SCHEDULE_ON_HOUR = "schedule_on_hour"
CONF_SCHEDULE_OFF_ACTIVE = "schedule_off_active"
CONF_SCHEDULE_OFF_HOUR = "schedule_off_hour"

CONF_HAYWARD_CLIMATE = "hayward_climate"

# Define all sensors here
SENSORS = [
    {
        CONF_KEY: CONF_SUCTION_TEMPERATURE,
        CONF_NAME: "Compressor Suction",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_ICON: ICON_THERMOMETER,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
    },
    {
        CONF_KEY: CONF_INLET_TEMPERATURE,
        CONF_NAME: "Water Inlet",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_ICON: ICON_THERMOMETER,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
    },
    {
        CONF_KEY: CONF_OUTLET_TEMPERATURE,
        CONF_NAME: "Water Outlet",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_ICON: ICON_THERMOMETER,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
    },
    {
        CONF_KEY: CONF_COIL1_TEMPERATURE,
        CONF_NAME: "Coil 1",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_ICON: ICON_THERMOMETER,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
    },
    {
        CONF_KEY: CONF_AMBIENT_TEMPERATURE,
        CONF_NAME: "Ambient",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_ICON: ICON_THERMOMETER,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
    },
    {
        CONF_KEY: CONF_EXHAUST_TEMPERATURE,
        CONF_NAME: "Compressor Exhaust",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_ICON: ICON_THERMOMETER,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
    },
    {
        CONF_KEY: CONF_COMPRESSOR_CURRENT,
        CONF_NAME: "Compressor Current",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_AMPERE,
        CONF_ICON: ICON_CURRENT_AC,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_CURRENT,
    },
    {
        CONF_KEY: CONF_AC_FAN_OUTPUT,
        CONF_NAME: "AC Fan Output",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_PERCENT,
        CONF_ICON: ICON_PERCENT,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_POWER_FACTOR,
    },
    {
        CONF_KEY: CONF_TARGET_SPEED_FAN_MOTOR,
        CONF_NAME: "Fan Motor Target Speed",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_REVOLUTIONS_PER_MINUTE,
        CONF_ICON: ICON_FAN,
        CONF_ACCURACY_DECIMALS: 0,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: "",
    },
    {
        CONF_KEY: CONF_INVERTER_PLATE_AC_VOLTAGE,
        CONF_NAME: "Inverter Plate AC Voltage",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_VOLT,
        CONF_ICON: ICON_POWER,
        CONF_ACCURACY_DECIMALS: 0,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        CONF_DEVICE_CLASS: DEVICE_CLASS_VOLTAGE,
    },
    {
        CONF_KEY: CONF_SPEED_FAN_MOTOR_1,
        CONF_NAME: "Fan Motor 1 Speed",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_REVOLUTIONS_PER_MINUTE,
        CONF_ICON: ICON_FAN,
        CONF_ACCURACY_DECIMALS: 0,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        CONF_DEVICE_CLASS: "",
    },
    {
        CONF_KEY: CONF_SUPER_HEAT,
        CONF_NAME: "Super Heat",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_ICON: ICON_THERMOMETER,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
    },
    {
        CONF_KEY: CONF_OVER_HEAT_AFTER_COMMPEN,
        CONF_NAME: "Overheat After Compensation",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_ICON: ICON_THERMOMETER,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
    },
    {
        CONF_KEY: CONF_ANTI_FREEZE_TEMP,
        CONF_NAME: "Anti-Freeze Temperature",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_ICON: ICON_THERMOMETER,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
        CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
    },
    # {
    #     CONF_KEY: CONF_COIL2_TEMPERATURE,
    #     CONF_NAME: "Coil 2",
    #     CONF_DISABLED_BY_DEFAULT: False,
    #     CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
    #     CONF_ICON: ICON_THERMOMETER,
    #     CONF_ACCURACY_DECIMALS: 1,
    #     CONF_STATE_CLASS: STATE_CLASS_MEASUREMENT,
    #     CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
    # },
]
SWITCHES = [
    {
        CONF_KEY: CONF_SCHEDULE_SILENT_ACTIVE,
        CONF_NAME: "Silent schedule active",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_ICON: ICON_CHECK_CIRCLE_OUTLINE,
        CONF_DEVICE_CLASS: DEVICE_CLASS_SWITCH,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_CONFIG,
    },
    {
        CONF_KEY: CONF_SCHEDULE_ON_ACTIVE,
        CONF_NAME: "Power on schedule active",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_ICON: ICON_CHECK_CIRCLE_OUTLINE,
        CONF_DEVICE_CLASS: DEVICE_CLASS_SWITCH,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_CONFIG,
    },
    {
        CONF_KEY: CONF_SCHEDULE_OFF_ACTIVE,
        CONF_NAME: "Power off schedule active",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_ICON: ICON_CHECK_CIRCLE_OUTLINE,
        CONF_DEVICE_CLASS: DEVICE_CLASS_SWITCH,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_CONFIG,
    },
    {
        CONF_KEY: CONF_SILENT_ACTIVE,
        CONF_NAME: "Silent mode active",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_ICON: ICON_CHECK_CIRCLE_OUTLINE,
        CONF_DEVICE_CLASS: DEVICE_CLASS_SWITCH,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_CONFIG,
    },
]
NUMBERS = [
    {
        CONF_KEY: CONF_SCHEDULE_SILENT_START_HOUR,
        CONF_NAME: "Silent schedule start hour",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_HOUR,
        CONF_ICON: ICON_TIMER,
        CONF_MODE: "BOX",
        CONF_DEVICE_CLASS: DEVICE_CLASS_TIMESTAMP,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_CONFIG,
        CONF_MIN_VALUE: 0,
        CONF_MAX_VALUE: 23,
        CONF_STEP: 1,
    },
    {
        CONF_KEY: CONF_SCHEDULE_SILENT_STOP_HOUR,
        CONF_NAME: "Silent schedule stop hour",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_HOUR,
        CONF_ICON: ICON_TIMER,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TIMESTAMP,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_CONFIG,
        CONF_MIN_VALUE: 0,
        CONF_MAX_VALUE: 23,
        CONF_STEP: 1,
    },
    {
        CONF_KEY: CONF_SCHEDULE_ON_HOUR,
        CONF_NAME: "Power on schedule hour",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_HOUR,
        CONF_ICON: ICON_TIMER,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TIMESTAMP,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_CONFIG,
        CONF_MIN_VALUE: 0,
        CONF_MAX_VALUE: 23,
        CONF_STEP: 1,
    },
    {
        CONF_KEY: CONF_SCHEDULE_OFF_HOUR,
        CONF_NAME: "Power off schedule hour",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_UNIT_OF_MEASUREMENT: UNIT_HOUR,
        CONF_ICON: ICON_TIMER,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TIMESTAMP,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_CONFIG,
        CONF_MIN_VALUE: 0,
        CONF_MAX_VALUE: 23,
        CONF_STEP: 1,
    },
]

CLIMATE = [
    {
        CONF_KEY: CONF_HAYWARD_CLIMATE,
        CONF_NAME: "Hayward Climate",
        CONF_DISABLED_BY_DEFAULT: False,
    }
]

TIME = {
    cv.Optional(CONF_TIME_ID): cv.declare_id(HomeassistantTime)
}

# Version 1 of the schema
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Hayward),
}).extend({
    cv.Optional(s[CONF_KEY], default={}): sensor.sensor_schema(
        unit_of_measurement=s.get(CONF_UNIT_OF_MEASUREMENT, ""),
        icon=s.get(CONF_ICON, ""),
        accuracy_decimals=s.get(CONF_ACCURACY_DECIMALS, ""),
        state_class=s.get(CONF_STATE_CLASS, ""),
        device_class=s.get(CONF_DEVICE_CLASS, ""),
        entity_category=s.get(CONF_ENTITY_CATEGORY, "")
    ).extend({
        cv.Optional(CONF_NAME, default=s.get(CONF_NAME, "")): cv.string,
        cv.Optional(CONF_DISABLED_BY_DEFAULT, default=s.get(CONF_DISABLED_BY_DEFAULT, "")): cv.boolean,
    })
    for s in SENSORS
}).extend({
    cv.Optional(s[CONF_KEY], default={}): switch.switch_schema(switch.Switch,
        icon=s.get(CONF_ICON, ""),
        device_class=s.get(CONF_DEVICE_CLASS, ""),
        entity_category=s.get(CONF_ENTITY_CATEGORY, "")
    ).extend({
        cv.Optional(CONF_ID, default=s.get(CONF_KEY, "")): cv.declare_id(hayward_ns.HaywardSwitch),
        cv.Optional(CONF_NAME, default=s.get(CONF_NAME, "")): cv.string,
        cv.Optional(CONF_DISABLED_BY_DEFAULT, default=s.get(CONF_DISABLED_BY_DEFAULT, "")): cv.boolean,
    })
    for s in SWITCHES
}).extend({
    cv.Optional(s[CONF_KEY], default={}): number.number_schema(
        hayward_ns.HaywardHour,
    ).extend({
        cv.Optional(CONF_ID, default=s.get(CONF_KEY, "")): cv.declare_id(hayward_ns.HaywardHour),
        cv.Optional(CONF_NAME, default=s.get(CONF_NAME, "")): cv.string,
        cv.Optional(CONF_DISABLED_BY_DEFAULT, default=s.get(CONF_DISABLED_BY_DEFAULT, "")): cv.boolean,
    })
    for s in NUMBERS
}).extend({
    cv.Optional(s[CONF_KEY], default={}): climate.CLIMATE_SCHEMA.extend(cv.COMPONENT_SCHEMA).extend(
        {
        cv.Optional(CONF_ID, default=s.get(CONF_KEY, "")): cv.declare_id(hayward_ns.HaywardClimate),
        cv.Optional(CONF_NAME, default=s.get(CONF_NAME, "")): cv.string,
    })
    for s in CLIMATE
}).extend(TIME).extend(cv.COMPONENT_SCHEMA).extend(modbus.modbus_server_schema())


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await modbus.register_modbus_server(var, config)

    # Register all sensors with their configurations
    for sn in SENSORS:
        print("Adding sensor", sn[CONF_KEY])
        sn_conf = config[sn[CONF_KEY]]
        sn_inst = await sensor.new_sensor(sn_conf)
        cg.add(getattr(var, "set_" + sn[CONF_KEY])(sn_inst))

    # Register all switches with their configurations
    for sw in SWITCHES:
        sw_conf = config[sw[CONF_KEY]]
        print ("Adding switch", sw[CONF_KEY])
        sw_inst = await switch.new_switch(sw_conf)
        cg.add(getattr(var, "set_" + sw[CONF_KEY])(sw_inst))

    # Register all numbers with their configurations
    for nu in NUMBERS:
        print("Adding number", nu[CONF_KEY])
        nu_conf = config[nu[CONF_KEY]]
        nu_inst = await number.new_number(nu_conf, min_value=0, max_value=23, step=1)
        cg.add(getattr(var, "set_" + nu[CONF_KEY])(nu_inst))

    for cl in CLIMATE:
      cl_conf = config[cl[CONF_KEY]]
      print("Adding climate", cl[CONF_KEY])
      cl_inst = cg.new_Pvariable(cl_conf[CONF_ID])
      #await cg.register_component(cl_inst, cl_conf)
      await climate.register_climate(cl_inst, cl_conf)
      cg.add(getattr(var, "set_" + cl[CONF_KEY])(cl_inst))

    # Register the time component
    if CONF_TIME_ID in config:
        time_conf = config[CONF_TIME_ID]
        time_inst = cg.new_Pvariable(time_conf[CONF_ID])
        await homeassistant.register_time(time_inst, time_conf)
        cg.add(var.set_time(time_inst))
