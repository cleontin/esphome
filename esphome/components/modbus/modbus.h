#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#include <vector>

namespace esphome {
namespace modbus {

enum ModbusRole {
  CLIENT,
  SERVER,
};

enum class ModbusFunctionCode : uint8_t {
  ReadCoils = 0x01,
  ReadDiscreteInputs = 0x02,
  ReadHoldingRegisters = 0x03,
  ReadInputRegisters = 0x04,
  WriteSingleCoil = 0x05,
  WriteSingleRegister = 0x06,
  WriteMultipleCoils = 0x0F,
  WriteMultipleRegisters = 0x10
  // Add other Modbus function codes as needed
};

const char* get_function_code_name(uint8_t code);

class ModbusDevice;
class ModbusServer;

class Modbus : public uart::UARTDevice, public Component {
 public:
  Modbus() = default;

  void setup() override;

  void loop() override;

  void dump_config() override;

  void register_device(ModbusDevice *device) { this->devices_.push_back(device); }
  void register_server(ModbusServer *server) { this->servers_.push_back(server); }

  float get_setup_priority() const override;

  void send(uint8_t address, uint8_t function_code, uint16_t start_address, uint16_t number_of_entities,
            uint8_t payload_len = 0, const uint8_t *payload = nullptr);
  void send_raw(const std::vector<uint8_t> &payload);
  void set_role(ModbusRole role) { this->role = role; }
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  uint8_t waiting_for_response{0};
  void set_send_wait_time(uint16_t time_in_ms) { send_wait_time_ = time_in_ms; }
  void set_disable_crc(bool disable_crc) { disable_crc_ = disable_crc; }
  void set_accept_broadcast(bool accept_broadcast) { this->accept_broadcast_ = accept_broadcast; }

  void handle_request();
  void handle_response();

  ModbusRole role;

 protected:
  GPIOPin *flow_control_pin_{nullptr};

  bool parse_modbus_byte_(uint8_t byte);
  uint16_t send_wait_time_{250};
  bool disable_crc_;
  bool accept_broadcast_;
  std::vector<uint8_t> rx_buffer_;
  uint32_t last_modbus_byte_{0};
  uint32_t last_send_{0};
  std::vector<ModbusDevice *> devices_;
  std::vector<ModbusServer *> servers_;
};

class ModbusDevice {
 public:
  void set_parent(Modbus *parent) { parent_ = parent; }
  void set_address(uint8_t address) { address_ = address; }
  virtual void on_modbus_data(const std::vector<uint8_t> &data) = 0;
  virtual void on_modbus_error(uint8_t function_code, uint8_t exception_code) {}
  virtual void on_modbus_read_registers(uint8_t function_code, uint16_t start_address, uint16_t number_of_registers){};
  void send(uint8_t function, uint16_t start_address, uint16_t number_of_entities, uint8_t payload_len = 0,
            const uint8_t *payload = nullptr) {
    this->parent_->send(this->address_, function, start_address, number_of_entities, payload_len, payload);
  }
  void send_raw(const std::vector<uint8_t> &payload) { this->parent_->send_raw(payload); }
  // If more than one device is connected block sending a new command before a response is received
  bool waiting_for_response() { return parent_->waiting_for_response != 0; }

 protected:
  friend Modbus;

  Modbus *parent_;
  uint8_t address_;
};

class ModbusServer {
  public:
   void set_parent(Modbus *parent) { parent_ = parent; }
   void set_address(uint8_t address) { address_ = address; }
   void set_accept_broadcast(bool accept_broadcast) { accept_broadcast_ = accept_broadcast; }
   //virtual void on_modbus_request(const std::vector<uint8_t> &data) = 0;
   //virtual void on_modbus_error(uint8_t function_code, uint8_t exception_code) {}
   //virtual void on_modbus_read_registers(uint8_t function_code, uint16_t start_address, uint16_t number_of_registers){};
   virtual void on_write_registers(uint16_t start_address, uint16_t bytes, const uint8_t *data) {};
   void send(uint8_t function, uint16_t start_address, uint16_t number_of_entities, uint8_t payload_len = 0,
             const uint8_t *payload = nullptr) {
     this->parent_->send(this->address_, function, start_address, number_of_entities, payload_len, payload);
   }
   void send_raw(const std::vector<uint8_t> &payload) { this->parent_->send_raw(payload); }
 
  protected:
   friend Modbus;
 
   Modbus *parent_;
   uint8_t address_;
   uint16_t register_start_;
   uint16_t register_count_;
   bool accept_broadcast_{false};
 };

}  // namespace modbus
}  // namespace esphome
