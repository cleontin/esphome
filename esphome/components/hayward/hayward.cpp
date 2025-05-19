#include "esphome/core/log.h"
#include "hayward.h"

namespace esphome {
namespace hayward {

static const char *TAG = "hayward";



void Hayward::setup() {
  this->signature_valid_ = false;
  this->has_1001_aquired_ = false;
  this->has_1091_aquired_ = false;
  this->has_1001_update_ = false;
  this->has_1091_update_ = false;

  //this->schedule_silent_active_switch_->set_register(1072);
  // this->schedule_on_active_switch_->set_register(1158);
  // this->schedule_off_active_switch_->set_register(1159);

  // this->schedule_silent_start_hour_->set_register(1068);
  // this->schedule_silent_stop_hour_->set_register(1069);
  // this->schedule_on_hour_->set_register(1150);
  // this->schedule_off_hour_->set_register(1152);
}
void Hayward::loop() {}

void Hayward::dump_config() {
  ESP_LOGCONFIG(TAG, "Hayward sensor");
  ESP_LOGCONFIG(TAG, "  Address: %d", this->address_);
  ESP_LOGCONFIG(TAG, "  Register start: %d", this->register_start_);
  ESP_LOGCONFIG(TAG, "  Register count: %d", this->register_count_);
}

float Hayward::bytesToInt(const std::array<uint8_t, 180>& buffer, size_t index) {
  return static_cast<int16_t>((buffer[index] << 8) | buffer[index + 1]);
}
float Hayward::bytesToTenths(const std::array<uint8_t, 180>& buffer, size_t index) {
  int16_t value = static_cast<int16_t>((buffer[index] << 8) | buffer[index + 1]);
  return value / 10.0f;
}
uint16_t Hayward::digitsToUint(int16_t value) {
  uint8_t high_digit = value / 10;
  uint8_t low_digit = value % 10;
  return (high_digit << 4) | low_digit;
}

void Hayward::save_signature() {
  memcpy(this->registers_3001_.data(), this->registers_2001_.data(), 20);
  this->registers_3001_[18] = 0x0B;
  this->registers_3001_[19] = 0xB9;
  this->signature_valid_ = true;
}

void Hayward::publish_if_changed(sensor::Sensor *sensor, float new_value, float threshold = 0.1) {
  if (sensor != nullptr && (!sensor->has_state() ||
      std::abs(sensor->get_state() - new_value) >= threshold)) {
    sensor->publish_state(new_value);
  }
}

void Hayward::on_write_registers(uint16_t start_address, uint16_t bytes, const uint8_t *data, bool is_broadcast) {

  uint16_t received_value = 0;
  if (!is_broadcast) {
    ESP_LOGVV(TAG, "Write register %d %d %s", start_address, bytes, format_hex_pretty(data, bytes).c_str());
  }

  if (start_address == 2001 && bytes == 180) {
    // This is the periodic update from the device with sensor values
    memcpy(this->registers_2001_.data(), data, bytes);
    this->save_signature();
    this->update_diagnostic_entities();
  }
  else if (start_address == 1001 && bytes == 180) {
    // This is an update of the current settings of the device
    memcpy(this->registers_1001_.data(), data, bytes);
    this->has_1001_aquired_ = true;
    this->has_1001_update_ = false;
    this->update_settings_entities();
  }
  else if (start_address == 1091 && bytes == 180) {
    // This are set temperatures for each mode and other settings
    memcpy(this->registers_1091_.data(), data, bytes);
    this->has_1091_aquired_ = true;
    this->has_1091_update_ = false;
    this->update_extra_settings_entities();
  }
  else {
    ESP_LOGW(TAG, "Unknown register write: %d %d", start_address, bytes);
  }

  
}

void Hayward::on_read_registers(uint16_t start_address, uint16_t count, bool is_broadcast) {
  if (is_broadcast) {
    ESP_LOGW(TAG, "Broadcast read request; ignoring");
    return;
  }
  else {
    ESP_LOGVV(TAG, "Read register %d %d", start_address, count);
  }
  ESPTime now = this->time_->now();
  //ESP_LOGI(TAG, "Current time: %02d:%02d:%02d", now.hour, now.minute, now.second);
  this->set_register(3015, digitsToUint(now.hour));
  this->set_register(3016, digitsToUint(now.minute));
  this->set_register(3017, digitsToUint(now.second));
  


  if (start_address == 3001 && count == 30) {
    ESP_LOGVV(TAG, "Read status requested");
    if (this->signature_valid_) {
      if (this->has_1001_update_) {
        this->set_register(3011, 4);
      }
      else if (this->has_1091_update_) {
        this->set_register(3011, 16);
      }
      else {
        this->set_register(3011, 0);
      }
      this->send(0x03, 3001, 30, 60, this->registers_3001_.data());
    }
  }
  else if (start_address == 1001 && count == 90) {
    ESP_LOGI(TAG, "Read update requested");
    this->send(0x03, 1001, 90, 180, this->registers_1001_.data());
    this->has_1001_update_ = false;
  }
  else if (start_address == 1091 && count == 90) {
    ESP_LOGI(TAG, "Read update requested");
    this->send(0x03, 1091, 90, 180, this->registers_1091_.data());
    this->has_1091_update_ = false;
  }
  else {
    ESP_LOGW(TAG, "Unknown register read: %d %d", start_address, count);
  }
}

void Hayward::set_register(uint16_t address, uint16_t value) {
  //ESP_LOGI(TAG, "Set local register %d to %d", address, value);
  if (address >= 1001 && address <= 1090) {
    if (!this->has_1001_aquired_) {
      ESP_LOGW(TAG, "Cannot set local register %d to %d, 1001 not acquired", address, value);
      return;
    }
    if (value == this->get_register(address)) {
      ESP_LOGW(TAG, "Register %d already set to %d", address, value);
      return;
    }
    int index = (address - 1001) * 2;
    this->registers_1001_[index] = value >> 8;
    this->registers_1001_[index + 1] = value & 0xFF;
    this->has_1001_update_ = true;
  }
  else if (address >= 1091 && address <= 1180) {
    if (!this->has_1091_aquired_) {
      ESP_LOGW(TAG, "Cannot set local register %d to %d, 1091 not acquired", address, value);
      return;
    }
    if (value == this->get_register(address)) {
      ESP_LOGW(TAG, "Register %d already set to %d", address, value);
      return;
    }
    int index = (address - 1091) * 2;
    this->registers_1091_[index] = value >> 8;
    this->registers_1091_[index + 1] = value & 0xFF;
    this->has_1091_update_ = true;
  }
  else if (address >= 3001 && address <= 3030) {
    int index = (address - 3001) * 2;
    this->registers_3001_[index] = value >> 8;
    this->registers_3001_[index + 1] = value & 0xFF;
  }
  else {
    ESP_LOGW(TAG, "Cannot set register %d to %d", address, value);
    return;
  }
}

uint16_t Hayward::get_register(uint16_t address) {
  uint16_t value = 0;
  if (address >= 1001 && address <= 1090) {
    value = (this->registers_1001_[address - 1001] << 8) | this->registers_1001_[address - 1001 + 1];
  }
  else if (address >= 1091 && address <= 1180) {
    value = (this->registers_1091_[address - 1091] << 8) | this->registers_1091_[address - 1091 + 1];
  }
  else if (address >= 3001 && address <= 3030) {
    value = (this->registers_3001_[address - 3001] << 8) | this->registers_3001_[address - 3001 + 1];
  }
  else {
    ESP_LOGW(TAG, "Cannot get register %d", address);
    return 0;
  }
  return value;
}

void Hayward::update_diagnostic_entities() {
  publish_if_changed(suction_temperature_sensor_, bytesToTenths(this->registers_2001_, 88)); //T01
  publish_if_changed(inlet_temperature_sensor_, bytesToTenths(this->registers_2001_, 90)); //T02
  this->hayward_climate_->current_temperature = bytesToTenths(this->registers_2001_, 90); //T02
  publish_if_changed(outlet_temperature_sensor_, bytesToTenths(this->registers_2001_, 92)); //T03
  publish_if_changed(coil1_temperature_sensor_, bytesToTenths(this->registers_2001_, 94)); //T04
  publish_if_changed(ambient_temperature_sensor_, bytesToTenths(this->registers_2001_, 96)); //T05
  publish_if_changed(exhaust_temperature_sensor_, bytesToTenths(this->registers_2001_, 98)); //T06
  publish_if_changed(super_heat_temperature_sensor_, bytesToTenths(this->registers_2001_, 118)); //T11
  publish_if_changed(overheat_after_commpen_sensor_, bytesToTenths(this->registers_2001_, 122)); //T13
  publish_if_changed(anti_freeze_temp_sensor_, bytesToTenths(this->registers_2001_, 126)); //T15
  publish_if_changed(compressor_current_sensor_, bytesToTenths(this->registers_2001_, 100)); //T07
  publish_if_changed(ac_fan_output_sensor_, bytesToTenths(this->registers_2001_, 102)); //T08
  publish_if_changed(target_speed_fan_motor_sensor_, bytesToInt(this->registers_2001_, 120)); //T12
  publish_if_changed(inverter_plate_ac_voltage_sensor_, bytesToInt(this->registers_2001_, 124)); //T14
  publish_if_changed(speed_fan_motor_1_sensor_, bytesToInt(this->registers_2001_, 132)); //T17


}

void Hayward::update_settings_entities() {
  
  climate::ClimateCall climate_call = this->hayward_climate_->make_call();

  for (int reg = 1001; reg < 1091; reg++) {
    int index = (reg - 1001) * 2;
    uint16_t received_value = this->encode_uint16_t(this->registers_1001_[index], this->registers_1001_[index+1]);

    switch (reg) {
      case 1011:
      case 1014:
        if (received_value == 0) {
          climate_call.set_mode(climate::CLIMATE_MODE_OFF);
        }
        break;
      case 1012:
        // Mode
        if (received_value == 0) {
          climate_call.set_mode(climate::CLIMATE_MODE_COOL);
        }
        else if (received_value == 1) {
          climate_call.set_mode(climate::CLIMATE_MODE_HEAT);
        }
        else if (received_value == 2) {
          climate_call.set_mode(climate::CLIMATE_MODE_AUTO);
        }
        break;
      case 1013:
        // Target temperature
        climate_call.set_target_temperature(received_value / 10.0);
        break;
      case 1068:
        // Schedule silent start hour
        this->schedule_silent_start_hour_->publish_state(received_value);
        break;
      case 1069:
        // Schedule silent stop hour
        this->schedule_silent_stop_hour_->publish_state(received_value);
        break;
      case 1072:
        // Schedule silent active
        this->schedule_silent_active_switch_->publish_state(received_value);
        break;
      default:
        // Unhandled register
        ESP_LOGVV(TAG, "Unhandled register %d value %d", reg, received_value);
        break;
    }
  }
  climate_call.perform();
}
    
void Hayward::update_extra_settings_entities() {
  for (int reg = 1091; reg < 1181; reg++) {
    int index = (reg - 1091) * 2;
    uint16_t received_value = this->encode_uint16_t(this->registers_1091_[index], this->registers_1091_[index+1]);
    switch (reg) {
      case 1150:
        // Schedule on hour
        this->schedule_on_hour_->publish_state(received_value);
        break;
      case 1152:
        // Schedule off hour
        this->schedule_off_hour_->publish_state(received_value);
        break;
      default:
        // Unhandled register
        ESP_LOGVV(TAG, "Unhandled register %d value %d", reg, received_value);
        break;
    }
  }
}

void HaywardSwitch::write_state(bool state) {
  // this->parent_->set_register(this->register_address_, state ? 1 : 0);
  // this->publish_state(state);
  return;
}

void HaywardClimate::setup() {
  this->current_temperature = 20.0;
  this->target_temperature = 21.0;
  this->mode = climate::CLIMATE_MODE_HEAT;
  this->action = climate::CLIMATE_ACTION_HEATING;
  this->publish_state();
  this->add_on_state_callback(std::bind(&HaywardClimate::on_state_change, this, std::placeholders::_1));
}

void HaywardClimate::on_state_change(climate::Climate& climate) {

  ESP_LOGI(TAG, "Climate state changed");

  if (!this->parent_->has_1001_aquired()) {
    return;
  }


}

void HaywardClimate::control(const climate::ClimateCall &call) {

  ESP_LOGI(TAG, "Climate control called");
  if (!(this->parent_->has_1001_aquired() && this->parent_->has_1091_aquired())) {
    ESP_LOGW(TAG, "Cannot control climate, 1001 and 1091 not acquired");
    return;
  }

  if (call.get_mode().has_value() && call.get_mode() != this->mode) {
    this->mode = *call.get_mode();
    uint16_t target_big_temp = 0;
    if (this->mode == climate::CLIMATE_MODE_COOL) {
      target_big_temp = this->parent_->get_register(1135);
      this->parent_->set_register(1013, target_big_temp);
      this->target_temperature = target_big_temp / 10.0;
      this->parent_->set_register(1012, 0);
      this->parent_->set_register(1011, 1);
      this->parent_->set_register(1014, 1);
    }
    else if (this->mode == climate::CLIMATE_MODE_HEAT) {
      target_big_temp = this->parent_->get_register(1136);
      this->parent_->set_register(1013, target_big_temp);
      this->target_temperature = target_big_temp / 10.0;
      this->parent_->set_register(1012, 1);
      this->parent_->set_register(1011, 1);
      this->parent_->set_register(1014, 1);
    }
    else if (this->mode == climate::CLIMATE_MODE_AUTO) {
      target_big_temp = this->parent_->get_register(1137);
      this->parent_->set_register(1013, target_big_temp);
      this->target_temperature = target_big_temp / 10.0;
      this->parent_->set_register(1012, 2);
      this->parent_->set_register(1011, 1);
      this->parent_->set_register(1014, 1);
    }
    else if (this->mode == climate::CLIMATE_MODE_OFF) {
      this->parent_->set_register(1011, 0);
      this->parent_->set_register(1014, 0);
    }
    else {
      ESP_LOGW(TAG, "Unsupported mode %d", this->mode);
    }
  }
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
    int target_temp = static_cast<uint16_t>(this->target_temperature * 10);
    this->parent_->set_register(1013, target_temp);
    if (this->mode == climate::CLIMATE_MODE_COOL) {
      this->parent_->set_register(1135, target_temp);
    }
    else if (this->mode == climate::CLIMATE_MODE_HEAT) {
      this->parent_->set_register(1136, target_temp);
    }
    else if (this->mode == climate::CLIMATE_MODE_AUTO) {
      this->parent_->set_register(1137, target_temp);
    }
  }
  this->publish_state();
}

void HaywardHour::control(float value) {
  this->parent_->set_register(this->register_address_, static_cast<uint16_t>(value));
  this->publish_state(value);

}

} //namespace hayward
} //namespace esphome