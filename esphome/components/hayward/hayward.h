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

class Hayward;

class HaywardSwitch : public switch_::Switch, public Component {
      
  protected:
    void write_state(bool state) override;
    uint16_t register_address_;
    hayward::Hayward *parent_;
  
  public:
    void set_register(uint16_t register_address) { this->register_address_ = register_address; }
    uint16_t get_register() { return this->register_address_; }
};

class HaywardHour : public number::Number, public Component {
    public:
     void setup() override {
        this->publish_state(0);
     }
     void set_register(uint16_t register_address) { this->register_address_ = register_address; }
     uint16_t get_register() { return this->register_address_; }

    protected:
     uint16_t register_address_;
      hayward::Hayward *parent_;

     void control(float value) override;

   
};

  
class HaywardClimate : public climate::Climate, public Component {
  public:
    void setup() override;
    void set_parent(hayward::Hayward *parent) { this->parent_ = parent; }
    void on_state_change(climate::Climate& climate);
  
  protected:
    hayward::Hayward *parent_;
    void control(const climate::ClimateCall &call) override;
    climate::ClimateTraits traits() override {
      climate::ClimateTraits traits{};
          traits.set_supports_current_temperature(true);
          traits.set_supported_modes({
              climate::CLIMATE_MODE_OFF,
              climate::CLIMATE_MODE_HEAT,
              climate::CLIMATE_MODE_COOL,
							climate::CLIMATE_MODE_AUTO,
          });
          // traits.set_supports_action(true);
          traits.set_visual_temperature_step(0.5);
          traits.set_visual_min_temperature(20.0);
          traits.set_visual_max_temperature(35.0);

      return traits;
    }
  
  };

class Hayward : public modbus::ModbusServer, public Component {

  void setup() override;
  void loop() override;
  void dump_config() override;

  void on_write_registers(uint16_t start_address, uint16_t bytes, const uint8_t *data, bool is_broadcast) override;
  void on_read_registers(uint16_t start_address, uint16_t count, bool is_broadcast) override;
  uint16_t encode_uint16_t(uint8_t msb, uint8_t lsb) { return ((static_cast<uint16_t>(msb) << 8) | lsb); }
  
  float bytesToInt(const std::array<uint8_t, 180>& buffer, size_t index);
  float bytesToTenths(const std::array<uint8_t, 180>& buffer, size_t index);
  uint16_t digitsToUint(int16_t value);

  void save_signature();
  //bool prepare_status_();
  void update_diagnostic_sensors(const uint8_t *data, size_t length);
  void update_diagnostic_entities();
  void update_settings_entities();
  void update_extra_settings_entities();
  void publish_if_changed(sensor::Sensor *sensor, float new_value, float threshold);

  std::array<uint8_t, 180> registers_2001_; // diagnostic updates
  std::array<uint8_t, 180> registers_1001_; // general settings
  std::array<uint8_t, 180> registers_1091_; // additional settings
  std::array<uint8_t, 60> registers_3001_; // status

  bool signature_valid_;
  bool has_1001_aquired_;
  bool has_1091_aquired_;
  bool has_1001_update_;
  bool has_1091_update_;

  void check_1001_update();
  void check_1091_update();

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
    bool has_1001_aquired() { return this->has_1001_aquired_; }
    bool has_1091_aquired() { return this->has_1091_aquired_; }
    void set_register(uint16_t address, uint16_t value);
    uint16_t get_register(uint16_t address);
    void set_suction_temperature(sensor::Sensor *suction_temperature_sensor) { suction_temperature_sensor_ = suction_temperature_sensor; }
    void set_inlet_temperature(sensor::Sensor *inlet_temperature_sensor) { inlet_temperature_sensor_ = inlet_temperature_sensor; }
    void set_outlet_temperature(sensor::Sensor *outlet_temperature_sensor) { outlet_temperature_sensor_ = outlet_temperature_sensor; }
    void set_coil1_temperature(sensor::Sensor *coil1_temperature_sensor) { coil1_temperature_sensor_ = coil1_temperature_sensor; }
    void set_ambient_temperature(sensor::Sensor *ambient_temperature_sensor) { ambient_temperature_sensor_ = ambient_temperature_sensor; }
    void set_exhaust_temperature(sensor::Sensor *exhaust_temperature_sensor) { exhaust_temperature_sensor_ = exhaust_temperature_sensor; }
    void set_compressor_current(sensor::Sensor *compressor_current_sensor) { compressor_current_sensor_ = compressor_current_sensor; }
    void set_ac_fan_output(sensor::Sensor *ac_fan_output_sensor) { ac_fan_output_sensor_ = ac_fan_output_sensor; }
    void set_target_speed_fan_motor(sensor::Sensor *target_speed_fan_motor_sensor) { target_speed_fan_motor_sensor_ = target_speed_fan_motor_sensor; }    
    void set_inverter_plate_ac_voltage(sensor::Sensor *inverter_plate_ac_voltage_sensor) { inverter_plate_ac_voltage_sensor_ = inverter_plate_ac_voltage_sensor; }
    void set_speed_fan_motor_1(sensor::Sensor *speed_fan_motor_1_sensor) { speed_fan_motor_1_sensor_ = speed_fan_motor_1_sensor; }
    void set_super_heat(sensor::Sensor *super_heat_temperature_sensor) { super_heat_temperature_sensor_ = super_heat_temperature_sensor; }
    void set_over_heat_after_commpen(sensor::Sensor *overheat_after_commpen_sensor) { overheat_after_commpen_sensor_ = overheat_after_commpen_sensor; }
    void set_anti_freeze_temp(sensor::Sensor *anti_freeze_temp_sensor) { anti_freeze_temp_sensor_ = anti_freeze_temp_sensor; }

    void set_schedule_silent_active(hayward::HaywardSwitch *schedule_silent_active_switch) {
      schedule_silent_active_switch_ = schedule_silent_active_switch;
      this->schedule_silent_active_switch_->set_register(1072);
    }
    void set_schedule_on_active(hayward::HaywardSwitch *schedule_on_active_switch) {
      schedule_on_active_switch_ = schedule_on_active_switch; 
      this->schedule_on_active_switch_->set_register(1158);
    }
    void set_schedule_off_active(hayward::HaywardSwitch *schedule_off_active_switch) { 
      schedule_off_active_switch_ = schedule_off_active_switch; 
      this->schedule_off_active_switch_->set_register(1159);
    }

    void set_schedule_silent_start_hour(hayward::HaywardHour *schedule_silent_start_hour) { 
      schedule_silent_start_hour_ = schedule_silent_start_hour; 
      this->schedule_silent_start_hour_->set_register(1068);
    }
    void set_schedule_silent_stop_hour(hayward::HaywardHour *schedule_silent_stop_hour) { 
      schedule_silent_stop_hour_ = schedule_silent_stop_hour; 
      this->schedule_silent_stop_hour_->set_register(1069);
    }
    void set_schedule_on_hour(hayward::HaywardHour *schedule_on_hour) { 
      schedule_on_hour_ = schedule_on_hour; 
      this->schedule_on_hour_->set_register(1150);
    }
    void set_schedule_off_hour(hayward::HaywardHour *schedule_off_hour) { 
      schedule_off_hour_ = schedule_off_hour; 
      this->schedule_off_hour_->set_register(1152);
    }

    void set_hayward_climate(hayward::HaywardClimate *hayward_climate) {
      hayward_climate_ = hayward_climate;
      this->hayward_climate_->set_parent(this);
    }

    void set_time(homeassistant::HomeassistantTime *time) {
      this->time_ = time;
    }
};



} //namespace hayward
} //namespace esphome