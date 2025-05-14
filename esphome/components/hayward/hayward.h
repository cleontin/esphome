#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/modbus/modbus.h"

namespace esphome {
namespace hayward {

class Hayward : public sensor::Sensor, public PollingComponent, public modbus::ModbusServer {
    void setup() override;
    void loop() override;
    void update() override;
    void dump_config() override;

    void on_register_update(const uint8_t *data, size_t size) override;

    float bytesToInt(const uint8_t* buffer) {
        return ((static_cast<int16_t>(buffer[0]) << 8) | buffer[1]);
    }
    float bytesToTenths(const uint8_t* buffer) {
        int16_t value = (buffer[0] << 8) | buffer[1];
        return value / 10.0f;
    }

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
    

    public:
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

        void set_super_heat_temperature(sensor::Sensor *super_heat_temperature_sensor) { super_heat_temperature_sensor_ = super_heat_temperature_sensor; }
        void set_overheat_after_commpen(sensor::Sensor *overheat_after_commpen_sensor) { overheat_after_commpen_sensor_ = overheat_after_commpen_sensor; }
        void set_anti_freeze_temp(sensor::Sensor *anti_freeze_temp_sensor) { anti_freeze_temp_sensor_ = anti_freeze_temp_sensor; }

        void set_coil2_temperature(sensor::Sensor *coil2_temperature_sensor) { coil2_temperature_sensor_ = coil2_temperature_sensor; }


};

} //namespace hayward
} //namespace esphome