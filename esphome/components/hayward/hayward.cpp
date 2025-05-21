#include "esphome/core/log.h"
#include "hayward.h"

namespace esphome {
namespace hayward {

static const char *TAG = "hayward";


void HaywardSwitch::write_state(bool state) {
  ESP_LOGD(TAG, "Setting register %d to %d", this->register_address_, state ? HAYWARD_ENABLED : HAYWARD_DISABLED);
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent is null");
    return;
  }
  this->parent_->set_register(this->register_address_, state ? HAYWARD_ENABLED : HAYWARD_DISABLED);
  this->publish_state(state);
}

void HaywardHour::control(float value) {
  uint16_t val = static_cast<uint16_t>(value);
  ESP_LOGD(TAG, "Setting register %d to %d", this->register_address_, val);
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent is null");
    return;
  }
  this->parent_->set_register(this->register_address_, val);
  this->publish_state(val);
}

climate::ClimateTraits HaywardClimate::traits() {
  climate::ClimateTraits traits{};
  traits.set_supports_current_temperature(true);
  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_AUTO,
  });
  // traits.set_supports_action(true);
  traits.set_visual_temperature_step(HAYWARD_TEMPERATURE_STEP);
  traits.set_visual_min_temperature(HAYWARD_TEMPERATURE_MIN);
  traits.set_visual_max_temperature(HAYWARD_TEMPERATURE_MAX);
  return traits;
}

void HaywardClimate::control(const climate::ClimateCall &call) {

  ESP_LOGV(TAG, "Climate control called");
  if (!(this->parent_->has_settings_aquired() && this->parent_->has_extra_settings_aquired())) {
    ESP_LOGW(TAG, "Cannot control climate, some registers not acquired");
    return;
  }

  if (call.get_mode().has_value() && call.get_mode() != this->mode) {
    this->mode = *call.get_mode();
    uint16_t target_big_temp = 0;
    if (this->mode == climate::CLIMATE_MODE_COOL) {
      ESP_LOGD(TAG, "Setting mode to COOL");
      target_big_temp = this->parent_->get_register(HAYWARD_SAVED_COOL_TEMP_REGISTER);
      this->parent_->set_register(HAYWARD_SET_TEMPERATURE_REGISTER, target_big_temp);
      this->target_temperature = target_big_temp / 10.0;
      this->parent_->set_register(HAYWARD_MODE_REGISTER, 0);
      this->parent_->set_register(HAYWARD_POWERON_REGISTER, 1);
      this->parent_->set_register(HAYWARD_POWERON2_REGISTER, 1);
    }
    else if (this->mode == climate::CLIMATE_MODE_HEAT) {
      ESP_LOGD(TAG, "Setting mode to HEAT");
      target_big_temp = this->parent_->get_register(HAYWARD_SAVED_HEAT_TEMP_REGISTER);
      this->parent_->set_register(HAYWARD_SET_TEMPERATURE_REGISTER, target_big_temp);
      this->target_temperature = target_big_temp / 10.0;
      this->parent_->set_register(HAYWARD_MODE_REGISTER, 1);
      this->parent_->set_register(HAYWARD_POWERON_REGISTER, 1);
      this->parent_->set_register(HAYWARD_POWERON2_REGISTER, 1);
    }
    else if (this->mode == climate::CLIMATE_MODE_AUTO) {
      ESP_LOGD(TAG, "Setting mode to AUTO");
      target_big_temp = this->parent_->get_register(HAYWARD_SAVED_AUTO_TEMP_REGISTER);
      this->parent_->set_register(HAYWARD_SET_TEMPERATURE_REGISTER, target_big_temp);
      this->target_temperature = target_big_temp / 10.0;
      this->parent_->set_register(HAYWARD_MODE_REGISTER, 2);
      this->parent_->set_register(HAYWARD_POWERON_REGISTER, 1);
      this->parent_->set_register(HAYWARD_POWERON2_REGISTER, 1);
    }
    else if (this->mode == climate::CLIMATE_MODE_OFF) {
      ESP_LOGD(TAG, "Setting mode to OFF");
      this->parent_->set_register(HAYWARD_POWERON_REGISTER, 0);
      this->parent_->set_register(HAYWARD_POWERON2_REGISTER, 0);
    }
    else {
      ESP_LOGW(TAG, "Unsupported mode %d", this->mode);
    }
  }

  if (call.get_target_temperature().has_value() && (call.get_target_temperature() != this->target_temperature)) {
    ESP_LOGD(TAG, "Setting target temperature to %f", *call.get_target_temperature());
    this->target_temperature = *call.get_target_temperature();
    int target_temp = static_cast<uint16_t>(this->target_temperature * 10);
    this->parent_->set_register(HAYWARD_SET_TEMPERATURE_REGISTER, target_temp);
    if (this->mode == climate::CLIMATE_MODE_COOL) {
      this->parent_->set_register(HAYWARD_SAVED_COOL_TEMP_REGISTER, target_temp);
    }
    else if (this->mode == climate::CLIMATE_MODE_HEAT) {
      this->parent_->set_register(HAYWARD_SAVED_HEAT_TEMP_REGISTER, target_temp);
    }
    else if (this->mode == climate::CLIMATE_MODE_AUTO) {
      this->parent_->set_register(HAYWARD_SAVED_AUTO_TEMP_REGISTER, target_temp);
    }
  }

  this->publish_state();
}

void Hayward::setup() {
  this->signature_valid_ = false;
  this->has_settings_aquired_ = false;
  this->has_extra_settings_aquired_ = false;
  this->has_settings_update_ = false;
  this->has_extra_settings_update = false;

  // this->status_error_response_[0] = 0x03;
  // this->status_error_response_[1] = MODBUS_STATUS_EXCEPTION;
}

void Hayward::dump_config() {
  ESP_LOGCONFIG(TAG, "Hayward sensor");
  ESP_LOGCONFIG(TAG, "  Acting as address: %d", this->address_);
  ESP_LOGCONFIG(TAG, "  Accepting broadcast messages: %s", this->accept_broadcast_ ? "YES" : "NO");
}

uint16_t Hayward::digits_to_uint(int16_t value) {
  uint8_t high_digit = value / 10;
  uint8_t low_digit = value % 10;
  return (high_digit << 4) | low_digit;
}

void Hayward::save_signature() {
  memcpy(this->status_registers_.data(), this->diagnostics_registers_.data(), HAYWARD_SIGNATURE_SIZE * 2);
  // memcpy(this->settings_registers_.data(), this->diagnostics_registers_.data(), HAYWARD_SIGNATURE_SIZE * 2);
  // memcpy(this->extra_settings_registers_.data(), this->diagnostics_registers_.data(), HAYWARD_SIGNATURE_SIZE * 2);
  this->set_register(HAYWARD_STATUS_ADDRESS_REGISTER, HAYWARD_STATUS_START_ADDRESS);
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
  ESP_LOGVV(TAG, "Write register strating from %d (%d bytes) %s",
    start_address, bytes, format_hex_pretty(data, bytes).c_str());
  
  if (start_address == HAYWARD_DIAGNOSTIC_START_ADDRESS && bytes == (HAYWARD_DIAGNOSTIC_SIZE * 2 )) {
    // this->compare_bytes(this->diagnostics_registers_.data(), data, bytes);
    memcpy(this->diagnostics_registers_.data(), data, bytes);
    if (!this->signature_valid_) {
      this->save_signature();
    }
    this->update_diagnostic_entities();
  }
  else if ((start_address == HAYWARD_SETTINGS_START_ADDRESS) &&
      (bytes == (HAYWARD_SETTINGS_SIZE * 2))) {
    this->compare_bytes(this->settings_registers_.data(), data, bytes);
    memcpy(this->settings_registers_.data(), data, bytes);
    ESP_LOGI(TAG, "Settings registers acquired");
    ESP_LOGD(TAG, "Settings registers: %s",
      format_hex_pretty(this->settings_registers_.data(), bytes).c_str());
    this->has_settings_aquired_ = true;
    this->has_settings_update_ = false;
    this->update_settings_entities();
  }
  else if ((start_address == HAYWARD_EXTRA_SETTINGS_START_ADDRESS) &&
      (bytes == (HAYWARD_EXTRA_SETTINGS_SIZE * 2))) {
    this->compare_bytes(this->extra_settings_registers_.data(), data, bytes);
    memcpy(this->extra_settings_registers_.data(), data, bytes);
    ESP_LOGI(TAG, "Extra settings registers acquired");
    ESP_LOGD(TAG, "Extra settings registers: %s",
      format_hex_pretty(this->extra_settings_registers_.data(), bytes).c_str());
    this->has_extra_settings_aquired_ = true;
    this->has_extra_settings_update = false;
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
    ESP_LOGV(TAG, "Read request register %d count %d", start_address, count);
  }

  if (start_address == HAYWARD_STATUS_START_ADDRESS && count == HAYWARD_STATUS_SIZE) {
    ESP_LOGVV(TAG, "Read status requested");

    if (this->has_settings_update_ && this->signature_valid_) {
      this->set_register(HAYWARD_UPDATE_FLAGS_REGISTER, HAYWARD_HAS_SETTINGS_UPDATES);
    }
    else if (this->has_extra_settings_update && this->signature_valid_) {
      this->set_register(HAYWARD_UPDATE_FLAGS_REGISTER, HAYWARD_HAS_EXTRA_SETTINGS_UPDATES);
    }
    else if (!this->has_settings_aquired_ || 
              !this->has_extra_settings_aquired_ ||
              !this->signature_valid_) {
      this->set_register(HAYWARD_UPDATE_FLAGS_REGISTER, HAYWARD_NEEDS_UPDATES);
      ESP_LOGW(TAG, "Sending status requesting settings and extra settings");
    }
    else {
      this->set_register(HAYWARD_UPDATE_FLAGS_REGISTER, HAYWARD_HAS_NO_UPDATES);
    }
    ESPTime now = this->time_->now();
    this->set_register(HAYWARD_CLOCK_HOUR_REGISTER, digits_to_uint(now.hour));
    this->set_register(HAYWARD_CLOCK_MINUTE_REGISTER, digits_to_uint(now.minute));
    this->set_register(HAYWARD_CLOCK_SECOND_REGISTER, digits_to_uint(now.second));
    this->send(0x03, HAYWARD_STATUS_START_ADDRESS, HAYWARD_STATUS_SIZE,
      HAYWARD_STATUS_SIZE * 2, this->status_registers_.data());
  }
  else if (start_address == HAYWARD_SETTINGS_START_ADDRESS && count == HAYWARD_SETTINGS_SIZE) {
    if (this->has_settings_aquired_) {
      ESP_LOGI(TAG, "Sending settings update");
      ESP_LOGD(TAG, "Settings registers: %s",
        format_hex_pretty(this->settings_registers_.data(), HAYWARD_SETTINGS_SIZE * 2).c_str());
      this->send(0x03, HAYWARD_SETTINGS_START_ADDRESS, HAYWARD_SETTINGS_SIZE,
        HAYWARD_SETTINGS_SIZE * 2, this->settings_registers_.data());
      this->has_settings_update_ = false;
    }
    else {
      ESP_LOGW(TAG, "Cannot send settings update, not acquired");
      // this->send_raw(this->status_error_response_);
      return;
    }
  }
  else if (start_address == HAYWARD_EXTRA_SETTINGS_START_ADDRESS && count == HAYWARD_EXTRA_SETTINGS_SIZE) {
    if (this->has_extra_settings_aquired_) {
      ESP_LOGI(TAG, "Sending extra settings update");
      ESP_LOGD(TAG, "Extra settings registers: %s",
        format_hex_pretty(this->extra_settings_registers_.data(), HAYWARD_EXTRA_SETTINGS_SIZE * 2).c_str());
      this->send(0x03, HAYWARD_EXTRA_SETTINGS_START_ADDRESS, HAYWARD_EXTRA_SETTINGS_SIZE,
        HAYWARD_EXTRA_SETTINGS_SIZE * 2, this->extra_settings_registers_.data());
      this->has_extra_settings_update = false;
    }
    else {
      ESP_LOGW(TAG, "Cannot send extra settings update, not acquired");
      // this->send_raw(this->status_error_response_);
      return;      
    }
  }
  else {
    ESP_LOGW(TAG, "Unknown register read: %d %d", start_address, count);
  }
}

void Hayward::set_register(uint16_t address, uint16_t value) {
  ESP_LOGV(TAG, "Set register %d to %d", address, value);
  if (address >= HAYWARD_SETTINGS_START_ADDRESS && address <= HAYWARD_SETTINGS_END_ADDRESS) {
    if (!this->has_settings_aquired_) {
      ESP_LOGW(TAG, "Cannot set register %d to %d, settings register set not acquired", address, value);
      return;
    }
    if (value == this->get_register(address)) {
      ESP_LOGW(TAG, "Register %d already set to %d", address, value);
      return;
    }
    int index = (address - HAYWARD_SETTINGS_START_ADDRESS) * 2;
    this->settings_registers_[index] = value >> 8;
    this->settings_registers_[index + 1] = value & 0xFF;
    this->has_settings_update_ = true;
  }
  else if (address >= HAYWARD_EXTRA_SETTINGS_START_ADDRESS && address <= HAYWARD_EXTRA_SETTINGS_END_ADDRESS) {
    if (!this->has_extra_settings_aquired_) {
      ESP_LOGW(TAG, "Cannot set register %d to %d, extra settings register set not acquired", address, value);
      return;
    }
    if (value == this->get_register(address)) {
      ESP_LOGW(TAG, "Register %d already set to %d", address, value);
      return;
    }
    int index = (address - HAYWARD_EXTRA_SETTINGS_START_ADDRESS) * 2;
    this->extra_settings_registers_[index] = value >> 8;
    this->extra_settings_registers_[index + 1] = value & 0xFF;
    this->has_extra_settings_update = true;
  }
  else if (address >= HAYWARD_STATUS_START_ADDRESS && address <= HAYWARD_STATUS_END_ADDRESS) {
    int index = (address - HAYWARD_STATUS_START_ADDRESS) * 2;
    this->status_registers_[index] = value >> 8;
    this->status_registers_[index + 1] = value & 0xFF;
  }
  else {
    ESP_LOGW(TAG, "Cannot set register %d to %d", address, value);
    return;
  }
}

int16_t Hayward::get_register(uint16_t address) {
  int16_t value = 0;
  uint8_t lsb = 0;
  uint8_t msb = 0;
  uint8_t index = 0;
  if (address >= HAYWARD_SETTINGS_START_ADDRESS && address <= HAYWARD_SETTINGS_END_ADDRESS) {
    index = (address - HAYWARD_SETTINGS_START_ADDRESS) * 2;
    msb = this->settings_registers_[index];
    lsb = this->settings_registers_[index + 1];
  }
  else if (address >= HAYWARD_EXTRA_SETTINGS_START_ADDRESS && address <= HAYWARD_EXTRA_SETTINGS_END_ADDRESS) {
    index = (address - HAYWARD_EXTRA_SETTINGS_START_ADDRESS) * 2;
    msb = this->extra_settings_registers_[index];
    lsb = this->extra_settings_registers_[index + 1];
  }
  else if (address >= HAYWARD_STATUS_START_ADDRESS && address <= HAYWARD_STATUS_END_ADDRESS) {
    index = (address - HAYWARD_STATUS_START_ADDRESS) * 2;
    msb = this->status_registers_[index];
    lsb = this->status_registers_[index + 1];
  }
  else if (address >= HAYWARD_DIAGNOSTIC_START_ADDRESS && address <= HAYWARD_DIAGNOSTIC_END_ADDRESS) {
    index = (address - HAYWARD_DIAGNOSTIC_START_ADDRESS) * 2;
    msb = this->diagnostics_registers_[index];
    lsb = this->diagnostics_registers_[index + 1];
  }
  else {
    ESP_LOGW(TAG, "Cannot get register %d", address);
    return 0;
  }
  value = static_cast<int16_t>((msb << 8) | lsb);
  ESP_LOGVV(TAG, "Get register %d: msb=%d, lsb=%d, value %d", address, msb, lsb, value);
  return value;
}

void Hayward::update_diagnostic_entities() {
  ESP_LOGV(TAG, "Update diagnostic entities");
  // These values are received every second, throttle down publishing
  publish_if_changed(this->suction_temperature_sensor_,
    this->get_register(HAYWARD_SUCTION_TEMPERATURE_REGISTER) / 10.0);
  publish_if_changed(this->inlet_temperature_sensor_,
    this->get_register(HAYWARD_INLET_TEMPERATURE_REGISTER) / 10.0);
  publish_if_changed(this->outlet_temperature_sensor_,
    this->get_register(HAYWARD_OUTLET_TEMPERATURE_REGISTER) / 10.0);
  publish_if_changed(this->coil1_temperature_sensor_,
    this->get_register(HAYWARD_COIL1_TEMPERATURE_REGISTER) / 10.0);
  publish_if_changed(this->ambient_temperature_sensor_,
    this->get_register(HAYWARD_AMBIENT_TEMPERATURE_REGISTER) / 10.0);
  publish_if_changed(this->exhaust_temperature_sensor_,
    this->get_register(HAYWARD_EXHAUST_TEMPERATURE_REGISTER) / 10.0);
  publish_if_changed(this->compressor_current_sensor_,
    this->get_register(HAYWARD_COMPRESSOR_CURRENT_REGISTER) / 10.0);
  publish_if_changed(this->ac_fan_output_sensor_,
    this->get_register(HAYWARD_AC_FAN_OUTPUT_REGISTER) / 10.0);
  publish_if_changed(this->super_heat_temperature_sensor_,
    this->get_register(HAYWARD_SUPER_HEAT_TEMPERATURE_REGISTER) / 10.0);
  publish_if_changed(this->target_speed_fan_motor_sensor_,
    this->get_register(HAYWARD_TARGET_SPEED_FAN_MOTOR_REGISTER));
  publish_if_changed(this->overheat_after_commpen_sensor_,
    this->get_register(HAYWARD_OVERHEAT_AFTER_COMMPEN_REGISTER) / 10.0);
  publish_if_changed(this->inverter_plate_ac_voltage_sensor_,
    this->get_register(HAYWARD_INVERTER_PLATE_AC_VOLTAGE_REGISTER));
  publish_if_changed(this->anti_freeze_temp_sensor_,
    this->get_register(HAYWARD_ANTI_FREEZE_TEMP_REGISTER) / 10.0);
  publish_if_changed(this->speed_fan_motor_1_sensor_,
    this->get_register(HAWYARD_SPEED_FAN_MOTOR1_REGISTER));

  this->hayward_climate_->current_temperature = this->get_register(HAYWARD_INLET_TEMPERATURE_REGISTER) / 10.0;
}

void Hayward::update_settings_entities() {

  // climate::ClimateCall climate_call = this->hayward_climate_->make_call();
  bool climate_updated = false;

  for (int reg = HAYWARD_SETTINGS_START_ADDRESS; reg <= HAYWARD_SETTINGS_END_ADDRESS; reg++) {
    uint16_t received_value = this->get_register(reg);

    switch (reg) {
      case HAYWARD_POWERON_REGISTER:
      case HAYWARD_POWERON2_REGISTER:
        ESP_LOGI(TAG, "Power on register %d value %d", reg, received_value);
        if ((received_value == 0) &&
            (this->hayward_climate_->mode != climate::CLIMATE_MODE_OFF)) {
          //climate_call.set_mode(climate::CLIMATE_MODE_OFF);
          this->hayward_climate_->mode = climate::CLIMATE_MODE_OFF;
          climate_updated = true;
        }
        break;
      case HAYWARD_MODE_REGISTER:
        ESP_LOGI(TAG, "Mode register %d value %d", reg, received_value);
        if ((received_value == HAYWARD_MODE_COOL) && 
            (this->hayward_climate_->mode != climate::CLIMATE_MODE_COOL)) {
          //climate_call.set_mode(climate::CLIMATE_MODE_COOL);
          this->hayward_climate_->mode = climate::CLIMATE_MODE_COOL;
          climate_updated = true;
        }
        else if ((received_value == HAYWARD_MODE_HEAT) &&
            (this->hayward_climate_->mode != climate::CLIMATE_MODE_HEAT)) {
          //climate_call.set_mode(climate::CLIMATE_MODE_HEAT);
          this->hayward_climate_->mode = climate::CLIMATE_MODE_HEAT;
          climate_updated = true;
        }
        else if ((received_value == HAYWARD_MODE_AUTO) &&
            (this->hayward_climate_->mode != climate::CLIMATE_MODE_AUTO)) {
          //climate_call.set_mode(climate::CLIMATE_MODE_AUTO);
          this->hayward_climate_->mode = climate::CLIMATE_MODE_AUTO;
          climate_updated = true;
        }
        break;
      case HAYWARD_SET_TEMPERATURE_REGISTER:
        if (this->hayward_climate_->target_temperature != (received_value / 10.0)) {
          climate_updated = true;
          ESP_LOGI(TAG, "Set temperature register %d value %d", reg, received_value);
          // climate_call.set_target_temperature(received_value / 10.0);
          this->hayward_climate_->target_temperature = received_value / 10.0;
          climate_updated = true;
        }
        break;
      case HAYWARD_SILENT_SCHEDULE_START_HOUR_REGISTER:
        ESP_LOGI(TAG, "Silent schedule start hour register %d value %d", reg, received_value);
        if (this->schedule_silent_start_hour_->state != received_value) {
          this->schedule_silent_start_hour_->publish_state(received_value);
        }
        break;
      case HAYWARD_SILENT_SCHEDULE_STOP_HOUR_REGISTER:
        ESP_LOGI(TAG, "Silent schedule stop hour register %d value %d", reg, received_value);
        if (this->schedule_silent_stop_hour_->state != received_value) {
          this->schedule_silent_stop_hour_->publish_state(received_value);
        }
        break;
      case HAYWARD_SILENT_SCHEDULE_ACTIVE_REGISTER:
        ESP_LOGI(TAG, "Silent schedule active register %d value %d", reg, received_value);
        if (this->schedule_silent_active_switch_->state != received_value) {
          this->schedule_silent_active_switch_->publish_state(received_value);
        }
        break;
      default:
        ESP_LOGVV(TAG, "Unhandled register %d value %d", reg, received_value);
        break;
    }
  }
  // climate_call.perform();
  if (climate_updated) {
    this->hayward_climate_->publish_state();
  }
}

void Hayward::update_extra_settings_entities() {
  for (int reg = HAYWARD_EXTRA_SETTINGS_START_ADDRESS; reg <= HAYWARD_EXTRA_SETTINGS_END_ADDRESS; reg++) {
    uint16_t received_value = this->get_register(reg);
    switch (reg) {
      case HAYWARD_POWERON_SCHEDULE_HOUR_REGISTER:
        ESP_LOGV(TAG, "Power on schedule hour register %d value %d", reg, received_value);
        this->schedule_on_hour_->publish_state(received_value);
        break;
      case HAYWARD_POWEROFF_SCHEDULE_HOUR_REGISTER:
        ESP_LOGV(TAG, "Power off schedule hour register %d value %d", reg, received_value);
        this->schedule_off_hour_->publish_state(received_value);
        break;
      case HAYWARD_POWERON_SCHEDULE_ACTIVE_REGISTER:
        ESP_LOGV(TAG, "Power on schedule active register %d value %d", reg, received_value);
        this->schedule_on_active_switch_->publish_state(received_value);
        break;
      case HAYWARD_POWEROFF_SCHEDULE_ACTIVE_REGISTER:
        ESP_LOGV(TAG, "Power off schedule active register %d value %d", reg, received_value);
        this->schedule_off_active_switch_->publish_state(received_value);
        break;
      default:
        ESP_LOGVV(TAG, "Unhandled register %d value %d", reg, received_value);
        break;
    }
  }
}

void Hayward::set_suction_temperature(sensor::Sensor *suction_temperature_sensor) {
  this->suction_temperature_sensor_ = suction_temperature_sensor;
}
void Hayward::set_inlet_temperature(sensor::Sensor *inlet_temperature_sensor) {
  this->inlet_temperature_sensor_ = inlet_temperature_sensor;
}
void Hayward::set_outlet_temperature(sensor::Sensor *outlet_temperature_sensor) {
  this->outlet_temperature_sensor_ = outlet_temperature_sensor;
}
void Hayward::set_coil1_temperature(sensor::Sensor *coil1_temperature_sensor) {
  this->coil1_temperature_sensor_ = coil1_temperature_sensor;
}
void Hayward::set_ambient_temperature(sensor::Sensor *ambient_temperature_sensor) {
  this->ambient_temperature_sensor_ = ambient_temperature_sensor;
}
void Hayward::set_exhaust_temperature(sensor::Sensor *exhaust_temperature_sensor) {
  this->exhaust_temperature_sensor_ = exhaust_temperature_sensor;
}
void Hayward::set_compressor_current(sensor::Sensor *compressor_current_sensor) {
  this->compressor_current_sensor_ = compressor_current_sensor;
}
void Hayward::set_ac_fan_output(sensor::Sensor *ac_fan_output_sensor) {
  this->ac_fan_output_sensor_ = ac_fan_output_sensor;
}
void Hayward::set_target_speed_fan_motor(sensor::Sensor *target_speed_fan_motor_sensor) {
  this->target_speed_fan_motor_sensor_ = target_speed_fan_motor_sensor;
}
void Hayward::set_inverter_plate_ac_voltage(sensor::Sensor *inverter_plate_ac_voltage_sensor) {
  this->inverter_plate_ac_voltage_sensor_ = inverter_plate_ac_voltage_sensor;
}
void Hayward::set_speed_fan_motor_1(sensor::Sensor *speed_fan_motor_1_sensor) {
  this->speed_fan_motor_1_sensor_ = speed_fan_motor_1_sensor;
}
void Hayward::set_super_heat(sensor::Sensor *super_heat_temperature_sensor) {
  this->super_heat_temperature_sensor_ = super_heat_temperature_sensor;
}
void Hayward::set_over_heat_after_commpen(sensor::Sensor *overheat_after_commpen_sensor) {
  this->overheat_after_commpen_sensor_ = overheat_after_commpen_sensor;
}
void Hayward::set_anti_freeze_temp(sensor::Sensor *anti_freeze_temp_sensor) {
  this->anti_freeze_temp_sensor_ = anti_freeze_temp_sensor;
}

void Hayward::set_schedule_silent_active(hayward::HaywardSwitch *schedule_silent_active_switch) {
  this->schedule_silent_active_switch_ = schedule_silent_active_switch;
  this->schedule_silent_active_switch_->set_parent(this);
  this->schedule_silent_active_switch_->set_register(HAYWARD_SILENT_SCHEDULE_ACTIVE_REGISTER);
}
void Hayward::set_schedule_silent_start_hour(hayward::HaywardHour *schedule_silent_start_hour) {
  this->schedule_silent_start_hour_ = schedule_silent_start_hour;
  this->schedule_silent_start_hour_->set_parent(this);
  this->schedule_silent_start_hour_->set_register(HAYWARD_SILENT_SCHEDULE_START_HOUR_REGISTER);
}
void Hayward::set_schedule_silent_stop_hour(hayward::HaywardHour *schedule_silent_stop_hour) {
  this->schedule_silent_stop_hour_ = schedule_silent_stop_hour;
  this->schedule_silent_stop_hour_->set_parent(this);
  this->schedule_silent_stop_hour_->set_register(HAYWARD_SILENT_SCHEDULE_STOP_HOUR_REGISTER);
}
void Hayward::set_silent_active(hayward::HaywardSwitch *silent_active_switch) {
  this->schedule_silent_active_switch_ = silent_active_switch;
  this->schedule_silent_active_switch_->set_parent(this);
  this->schedule_silent_active_switch_->set_register(HAYWARD_SILENT_ACTIVE);
}
void Hayward::set_schedule_on_active(hayward::HaywardSwitch *schedule_on_active_switch) {
  this->schedule_on_active_switch_ = schedule_on_active_switch;
  this->schedule_on_active_switch_->set_parent(this);
  this->schedule_on_active_switch_->set_register(HAYWARD_POWERON_SCHEDULE_ACTIVE_REGISTER);
}
void Hayward::set_schedule_off_active(hayward::HaywardSwitch *schedule_off_active_switch) {
  this->schedule_off_active_switch_ = schedule_off_active_switch;
  this->schedule_off_active_switch_->set_parent(this);
  this->schedule_off_active_switch_->set_register(HAYWARD_POWEROFF_SCHEDULE_ACTIVE_REGISTER);
}
void Hayward::set_schedule_on_hour(hayward::HaywardHour *schedule_on_hour) {
  this->schedule_on_hour_ = schedule_on_hour;
  this->schedule_on_hour_->set_parent(this);
  this->schedule_on_hour_->set_register(HAYWARD_POWERON_SCHEDULE_HOUR_REGISTER);
}
void Hayward::set_schedule_off_hour(hayward::HaywardHour *schedule_off_hour) {
  this->schedule_off_hour_ = schedule_off_hour;
  this->schedule_off_hour_->set_parent(this);
  this->schedule_off_hour_->set_register(HAYWARD_POWEROFF_SCHEDULE_HOUR_REGISTER);
}
void Hayward::set_hayward_climate(hayward::HaywardClimate *hayward_climate) {
  this->hayward_climate_ = hayward_climate;
  this->hayward_climate_->set_parent(this);
}

void Hayward::compare_bytes(const uint8_t* data1, const uint8_t* data2, size_t length) {
  bool found_difference = false;
  for (size_t i = 0; i < length; ++i) {
    if (data1[i] != data2[i]) {
      found_difference = true;
      ESP_LOGD(TAG, "Byte %zu differs: %02X -> %02X", i, data1[i], data2[i]);
    }
  }
  if (!found_difference) {
    ESP_LOGD(TAG, "No differences found");
  }
}

} //namespace hayward
} //namespace esphome
