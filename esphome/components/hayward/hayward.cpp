#include "esphome/core/log.h"
#include "hayward.h"

namespace esphome {
namespace hayward {

static const char *TAG = "hayward";

void Hayward::setup() {}
void Hayward::loop() {}
void Hayward::update() {}

void Hayward::dump_config() {
    ESP_LOGCONFIG(TAG, "Hayward sensor");
    ESP_LOGCONFIG(TAG, "  Address: %d", this->address_);
    ESP_LOGCONFIG(TAG, "  Register start: %d", this->register_start_);
    ESP_LOGCONFIG(TAG, "  Register count: %d", this->register_count_);
}

void Hayward::on_register_update(const uint8_t *data, size_t length) {

  if (length < 180) {
    ESP_LOGE(TAG, "Data buffer too short: %d bytes, need at least 180", length);
    return;
  }

  auto publish_if_changed = [](esphome::sensor::Sensor *sensor, float new_value, float threshold = 0.1) {
    if ( sensor != nullptr && (!sensor->has_state() || std::abs(sensor->get_state() - new_value) >= threshold)) {
      sensor->publish_state(new_value);
    }
  };

  publish_if_changed(suction_temperature_sensor_, bytesToTenths(&data[88]), 0.1); //T01
  publish_if_changed(inlet_temperature_sensor_, bytesToTenths(&data[90]), 0.1); //T02
  publish_if_changed(outlet_temperature_sensor_, bytesToTenths(&data[92]), 0.1); //T03
  publish_if_changed(coil1_temperature_sensor_, bytesToTenths(&data[94]), 0.1); //T04
  publish_if_changed(ambient_temperature_sensor_, bytesToTenths(&data[96]), 0.1); //T05
  publish_if_changed(exhaust_temperature_sensor_, bytesToTenths(&data[98]), 0.1); //T06
  publish_if_changed(super_heat_temperature_sensor_, bytesToTenths(&data[118]), 0.1); //T11
  publish_if_changed(overheat_after_commpen_sensor_, bytesToTenths(&data[122]), 0.1); //T13
  publish_if_changed(anti_freeze_temp_sensor_, bytesToTenths(&data[126]), 0.1); //T15

  publish_if_changed(compressor_current_sensor_, bytesToTenths(&data[100]), 0.1); //T07
  publish_if_changed(ac_fan_output_sensor_, bytesToTenths(&data[102]), 0.1); //T08
  publish_if_changed(target_speed_fan_motor_sensor_, bytesToInt(&data[120]), 0.1); //T12
  publish_if_changed(inverter_plate_ac_voltage_sensor_, bytesToInt(&data[124]), 0.1); //T14
  publish_if_changed(speed_fan_motor_1_sensor_, bytesToInt(&data[132]), 0.1); //T17
  
  //this->anti_freeze_temp_sensor_->publish_state(data[120]

  //this->coil2_temperature_sensor_->publish_state((data[96]<<8 | data[97])/10.0);

}

} //namespace hayward
} //namespace esphome