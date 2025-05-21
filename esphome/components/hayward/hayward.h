#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/modbus/modbus.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/homeassistant/time/homeassistant_time.h"
#include <vector>

namespace esphome {
namespace hayward {

// Modbus register addresses ans sizes
const uint16_t HAYWARD_DIAGNOSTIC_START_ADDRESS = 2001;
const uint16_t HAYWARD_DIAGNOSTIC_END_ADDRESS = 2090;
const uint16_t HAYWARD_DIAGNOSTIC_SIZE = 90;
const uint16_t HAYWARD_SETTINGS_START_ADDRESS = 1001;
const uint16_t HAYWARD_SETTINGS_END_ADDRESS = 1090;
const uint16_t HAYWARD_SETTINGS_SIZE = 90;
const uint16_t HAYWARD_EXTRA_SETTINGS_START_ADDRESS = 1091;
const uint16_t HAYWARD_EXTRA_SETTINGS_END_ADDRESS = 1180;
const uint16_t HAYWARD_EXTRA_SETTINGS_SIZE = 90;
const uint16_t HAYWARD_STATUS_START_ADDRESS = 3001;
const uint16_t HAYWARD_STATUS_END_ADDRESS = 3030;
const uint16_t HAYWARD_STATUS_SIZE = 30;
const uint16_t HAYWARD_SIGNATURE_SIZE = 10; //10 registers,

// Basic settings addresses
const uint16_t HAYWARD_POWERON_REGISTER = 1011;
const uint16_t HAYWARD_POWERON2_REGISTER = 1014;
const uint16_t HAYWARD_MODE_REGISTER = 1012;
const uint16_t HAYWARD_SET_TEMPERATURE_REGISTER = 1013;
const uint16_t HAYWARD_SILENT_SCHEDULE_START_HOUR_REGISTER = 1068;
const uint16_t HAYWARD_SILENT_SCHEDULE_STOP_HOUR_REGISTER = 1069;
const uint16_t HAYWARD_SILENT_SCHEDULE_ACTIVE_REGISTER = 1072;
const uint16_t HAYWARD_SILENT_ACTIVE = 1076;

// Extra settings addresses
const uint16_t HAYWARD_POWERON_SCHEDULE_HOUR_REGISTER = 1150;
const uint16_t HAYWARD_POWERON_SCHEDULE_ACTIVE_REGISTER = 1158;
const uint16_t HAYWARD_POWEROFF_SCHEDULE_HOUR_REGISTER = 1152;
const uint16_t HAYWARD_POWEROFF_SCHEDULE_ACTIVE_REGISTER = 1159;
const uint16_t HAYWARD_SAVED_COOL_TEMP_REGISTER = 1135;
const uint16_t HAYWARD_SAVED_HEAT_TEMP_REGISTER = 1136;
const uint16_t HAYWARD_SAVED_AUTO_TEMP_REGISTER = 1137;

// Diagnostic registers
const uint16_t HAYWARD_SUCTION_TEMPERATURE_REGISTER = 2045; // T01 on the display
const uint16_t HAYWARD_INLET_TEMPERATURE_REGISTER = 2046; // T02
const uint16_t HAYWARD_OUTLET_TEMPERATURE_REGISTER = 2047; // T03
const uint16_t HAYWARD_COIL1_TEMPERATURE_REGISTER = 2048; // T04
const uint16_t HAYWARD_AMBIENT_TEMPERATURE_REGISTER = 2049; // T05
const uint16_t HAYWARD_EXHAUST_TEMPERATURE_REGISTER = 2050; // T06
const uint16_t HAYWARD_COMPRESSOR_CURRENT_REGISTER = 2051; // T07
const uint16_t HAYWARD_AC_FAN_OUTPUT_REGISTER = 2052; // T08

const uint16_t HAYWARD_SUPER_HEAT_TEMPERATURE_REGISTER = 2060; // T11
const uint16_t HAYWARD_TARGET_SPEED_FAN_MOTOR_REGISTER = 2061; // T12
const uint16_t HAYWARD_OVERHEAT_AFTER_COMMPEN_REGISTER = 2062; // T13
const uint16_t HAYWARD_INVERTER_PLATE_AC_VOLTAGE_REGISTER = 2063; // T14
const uint16_t HAYWARD_ANTI_FREEZE_TEMP_REGISTER = 2064; // T15
const uint16_t HAWYARD_SPEED_FAN_MOTOR1_REGISTER = 2067; // T17

// Status registers
const uint16_t HAYWARD_STATUS_ADDRESS_REGISTER = 3010;
const uint16_t HAYWARD_UPDATE_FLAGS_REGISTER = 3011;
const uint16_t HAYWARD_CLOCK_HOUR_REGISTER = 3015;
const uint16_t HAYWARD_CLOCK_MINUTE_REGISTER = 3016;
const uint16_t HAYWARD_CLOCK_SECOND_REGISTER = 3017;

const uint16_t HAYWARD_HAS_NO_UPDATES = 0x00;
const uint16_t HAYWARD_HAS_SETTINGS_UPDATES = 0x04;
const uint16_t HAYWARD_HAS_EXTRA_SETTINGS_UPDATES = 0x10;
const uint16_t HAYWARD_NEEDS_UPDATES = 0x8000;
//const uint16_t HAYWARD_NEEDS_UPDATES = 0x01;

const uint8_t HAYWARD_MODE_COOL = 0;
const uint8_t HAYWARD_MODE_HEAT = 1;
const uint8_t HAYWARD_MODE_AUTO = 2;

const float HAYWARD_TEMPERATURE_MIN = 20;
const float HAYWARD_TEMPERATURE_MAX = 35;
const float HAYWARD_TEMPERATURE_STEP = 0.5;

const uint8_t HAYWARD_DISABLED = 0;
const uint8_t HAYWARD_ENABLED = 1;

const uint8_t MODBUS_STATUS_EXCEPTION = 0x04;

class Hayward;

class HaywardSwitch : public switch_::Switch, public Component {
  public:
    void set_register(uint16_t register_address) { this->register_address_ = register_address; }
    void set_parent(hayward::Hayward *parent) { this->parent_ = parent; }
    uint16_t get_register() { return this->register_address_; }

  protected:
    void write_state(bool state) override;
    uint16_t register_address_;
    hayward::Hayward *parent_;
};

class HaywardHour : public number::Number, public Component {
  public:
    void setup() override {}
    void set_parent(hayward::Hayward *parent) { this->parent_ = parent; }
    void set_register(uint16_t register_address) { this->register_address_ = register_address; }
    uint16_t get_register() { return this->register_address_; }

  protected:
    uint16_t register_address_;
    hayward::Hayward *parent_;
    void control(float value) override;
};

class HaywardClimate : public climate::Climate, public Component {
  public:
    void set_parent(hayward::Hayward *parent) { this->parent_ = parent; }

  protected:
    hayward::Hayward *parent_;
    void control(const climate::ClimateCall &call) override;
    climate::ClimateTraits traits() override;
  };

class Hayward : public modbus::ModbusServer, public Component {

  void setup() override;
  void dump_config() override;

  void on_write_registers(uint16_t start_address, uint16_t bytes, const uint8_t *data, bool is_broadcast) override;
  void on_read_registers(uint16_t start_address, uint16_t count, bool is_broadcast) override;
  uint16_t encode_uint16_t(uint8_t msb, uint8_t lsb) { return ((static_cast<uint16_t>(msb) << 8) | lsb); }

  uint16_t digits_to_uint(int16_t value);

  void save_signature();
  void update_diagnostic_sensors(const uint8_t *data, size_t length);
  void update_diagnostic_entities();
  void update_settings_entities();
  void update_extra_settings_entities();
  void publish_if_changed(sensor::Sensor *sensor, float new_value, float threshold);

  std::array<uint8_t, 180> diagnostics_registers_;
  std::array<uint8_t, 180> settings_registers_;
  std::array<uint8_t, 180> extra_settings_registers_;
  std::array<uint8_t, 60> status_registers_;
  std::vector<uint8_t> status_error_response_ = {0x83, 0x04}; // Modbus exception response

  bool signature_valid_;
  bool has_settings_aquired_;
  bool has_extra_settings_aquired_;
  bool has_settings_update_;
  bool has_extra_settings_update;

  sensor::Sensor *suction_temperature_sensor_{nullptr};
  sensor::Sensor *inlet_temperature_sensor_{nullptr};
  sensor::Sensor *outlet_temperature_sensor_{nullptr};
  sensor::Sensor *coil1_temperature_sensor_{nullptr};
  sensor::Sensor *ambient_temperature_sensor_{nullptr};
  sensor::Sensor *exhaust_temperature_sensor_{nullptr};
  sensor::Sensor *compressor_current_sensor_{nullptr};
  sensor::Sensor *ac_fan_output_sensor_{nullptr};
  sensor::Sensor *target_speed_fan_motor_sensor_{nullptr};
  sensor::Sensor *inverter_plate_ac_voltage_sensor_{nullptr};
  sensor::Sensor *speed_fan_motor_1_sensor_{nullptr};
  sensor::Sensor *super_heat_temperature_sensor_{nullptr};
  sensor::Sensor *overheat_after_commpen_sensor_{nullptr};
  sensor::Sensor *anti_freeze_temp_sensor_{nullptr};
  sensor::Sensor *coil2_temperature_sensor_{nullptr};

  hayward::HaywardSwitch *schedule_silent_active_switch_{nullptr};
  hayward::HaywardSwitch *schedule_on_active_switch_{nullptr};
  hayward::HaywardSwitch *schedule_off_active_switch_{nullptr};

  hayward::HaywardHour *schedule_silent_start_hour_{nullptr};
  hayward::HaywardHour *schedule_silent_stop_hour_{nullptr};
  hayward::HaywardHour *schedule_on_hour_{nullptr};
  hayward::HaywardHour *schedule_off_hour_{nullptr};

  hayward::HaywardClimate *hayward_climate_{nullptr};

  homeassistant::HomeassistantTime *time_{nullptr};

  public:
    bool has_settings_aquired() { return this->has_settings_aquired_; }
    bool has_extra_settings_aquired() { return this->has_extra_settings_aquired_; }
    void set_register(uint16_t address, uint16_t value);
    int16_t get_register(uint16_t address);
    
    void set_suction_temperature(sensor::Sensor *suction_temperature_sensor);
    void set_inlet_temperature(sensor::Sensor *inlet_temperature_sensor);
    void set_outlet_temperature(sensor::Sensor *outlet_temperature_sensor);
    void set_coil1_temperature(sensor::Sensor *coil1_temperature_sensor);
    void set_ambient_temperature(sensor::Sensor *ambient_temperature_sensor);
    void set_exhaust_temperature(sensor::Sensor *exhaust_temperature_sensor);
    void set_compressor_current(sensor::Sensor *compressor_current_sensor);
    void set_ac_fan_output(sensor::Sensor *ac_fan_output_sensor);
    void set_target_speed_fan_motor(sensor::Sensor *target_speed_fan_motor_sensor);
    void set_inverter_plate_ac_voltage(sensor::Sensor *inverter_plate_ac_voltage_sensor);
    void set_speed_fan_motor_1(sensor::Sensor *speed_fan_motor_1_sensor);
    void set_super_heat(sensor::Sensor *super_heat_temperature_sensor);
    void set_over_heat_after_commpen(sensor::Sensor *overheat_after_commpen_sensor);
    void set_anti_freeze_temp(sensor::Sensor *anti_freeze_temp_sensor);

    void set_schedule_silent_active(hayward::HaywardSwitch *schedule_silent_active_switch);
    void set_schedule_on_active(hayward::HaywardSwitch *schedule_on_active_switch);
    void set_schedule_off_active(hayward::HaywardSwitch *schedule_off_active_switch);

    void set_silent_active(hayward::HaywardSwitch *silent_active_switch);

    void set_schedule_silent_start_hour(hayward::HaywardHour *schedule_silent_start_hour);
    void set_schedule_silent_stop_hour(hayward::HaywardHour *schedule_silent_stop_hour);
    void set_schedule_on_hour(hayward::HaywardHour *schedule_on_hour);
    void set_schedule_off_hour(hayward::HaywardHour *schedule_off_hour);

    void set_hayward_climate(hayward::HaywardClimate *hayward_climate);

    void compare_bytes(const uint8_t* data1, const uint8_t* data2, size_t length);
};



} //namespace hayward
} //namespace esphome
