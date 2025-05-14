#include "modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace modbus {

static const char *const TAG = "modbus";

void Modbus::setup() {
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }
}
void Modbus::loop() {
  const uint32_t now = millis();

  if (now - this->last_modbus_byte_ > 50) {
    this->rx_buffer_.clear();
    this->last_modbus_byte_ = now;
  }
  // stop blocking new send commands after send_wait_time_ ms regardless if a response has been received since then
  if (now - this->last_send_ > send_wait_time_) {
    waiting_for_response = 0;
  }

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    if (this->parse_modbus_byte_(byte)) {
      this->last_modbus_byte_ = now;
    } else {
      this->rx_buffer_.clear();
    }
  }
}

const char* get_function_code_name(uint8_t code) {
  switch (code) {
    case static_cast<uint8_t>(ModbusFunctionCode::ReadCoils):
      return "Read Coils";
    case static_cast<uint8_t>(ModbusFunctionCode::ReadDiscreteInputs):
      return "Read Discrete Inputs";
    case static_cast<uint8_t>(ModbusFunctionCode::ReadHoldingRegisters):
      return "Read Holding Registers";
    case static_cast<uint8_t>(ModbusFunctionCode::ReadInputRegisters):
      return "Read Input Registers";
    case static_cast<uint8_t>(ModbusFunctionCode::WriteSingleCoil):
      return "Write Single Coil";
    case static_cast<uint8_t>(ModbusFunctionCode::WriteSingleRegister):
      return "Write Single Register";
    case static_cast<uint8_t>(ModbusFunctionCode::WriteMultipleCoils):
      return "Write Multiple Coils";
    case static_cast<uint8_t>(ModbusFunctionCode::WriteMultipleRegisters):
      return "Write Multiple Registers";
    default:
      return "Unknown Function Code";
  }
}

bool Modbus::parse_modbus_byte_(uint8_t byte) {
  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  const uint8_t *raw = &this->rx_buffer_[0];
  ESP_LOGV(TAG, "Modbus received Byte  %d (0X%x)", byte, byte);

  if (at == 0)
    // Byte 0: modbus address, valid so far
    return true;

  uint8_t address = raw[0];
  uint8_t function_code = raw[1];
  
  
  // We need at least byte 3  (2 and 3 being CRC) for a valid message
  if (at < 3)
    return true;

  uint8_t data_offset = 2;
  uint8_t data_len = 1;
  uint8_t rs_data_len = 1;


  // Data length and offset depends on the function code and whether the frame is a request or a response
  ModbusFunctionCode code = static_cast<ModbusFunctionCode>(function_code);
  if (code == ModbusFunctionCode::ReadCoils ||
      code == ModbusFunctionCode::ReadDiscreteInputs ||
      code == ModbusFunctionCode::ReadHoldingRegisters ||
      code == ModbusFunctionCode::ReadInputRegisters) {
    data_len = 4;
    rs_data_len = (at > 1) ? uint8_t(raw[2]) + 1 : 2;
  }
  else if (code == ModbusFunctionCode::WriteSingleCoil ||
             code == ModbusFunctionCode::WriteSingleRegister) {
    data_len = 4;
    rs_data_len = 4;
  }
  else if (function_code == 0x7) {
    data_len = 0;
    rs_data_len = 1;
  }
  else if (function_code == 0x9) {
    data_len = 0;
    rs_data_len = 4;
  }
  else if (code == ModbusFunctionCode::WriteMultipleCoils  ||
             code == ModbusFunctionCode::WriteMultipleRegisters) {
    data_len = (at > 5) ? uint8_t(raw[6]) + 5 : 6;
    rs_data_len = 4;
  }
  else if ((function_code & 0x80) == 0x80) {
    data_len = 1;
  }

  // We need the CRC bytes for either a request or a response 
  if ((at < data_offset + data_len + 1) && (at < data_offset + rs_data_len + 1))
    return true;


  std::vector<uint8_t> data(this->rx_buffer_.begin(), this->rx_buffer_.begin() + at + 1);

  // Are we at the right length for a request or response?
  if (at == data_offset + data_len + 1) {
    // We might have a request here
    uint16_t computed_crc = crc16(raw, data_offset + data_len);
    uint16_t remote_crc = uint16_t(raw[data_offset + data_len]) | (uint16_t(raw[data_offset + data_len + 1]) << 8);

    if (computed_crc == remote_crc) {
      //ESP_LOGV(TAG, "Modbus CRC Check matches! %02X==%02X", computed_crc, remote_crc);
      //ESP_LOGV(TAG, "  Function: %02X request, Len: %02X", function_code, data_len);
      //ESP_LOGV(TAG, "  Frame: %s", format_hex_pretty(raw,at+1).c_str());
      this->handle_request();
      return false; // Start a new frame
    }
  } else if (at == data_offset + rs_data_len + 1) {
    // Check for a response
    uint16_t computed_crc = crc16(raw, data_offset + rs_data_len);
    uint16_t remote_crc = uint16_t(raw[data_offset + rs_data_len]) | (uint16_t(raw[data_offset + rs_data_len + 1]) << 8);

    if (computed_crc == remote_crc) {
      //ESP_LOGW(TAG, "Modbus CRC Check matches! %02X==%02X", computed_crc, remote_crc);
      //ESP_LOGW(TAG, "  Function: %02X response, Len: %02X", function_code, rs_data_len);
      //ESP_LOGW(TAG, "  Frame: %s", format_hex_pretty(raw,at+1).c_str());
      this->handle_response();
      return false; // Start a new frame
    }
  } 
  
  if ((at > data_offset + data_len + 1) && (at > data_offset + rs_data_len + 1)) {
    ESP_LOGW(TAG, "Frame did not match: %s", format_hex_pretty(raw,at+1).c_str());
    ESP_LOGW(TAG, "  Frame: %s", format_hex_pretty(raw, data_offset + data_len).c_str());
    ESP_LOGW(TAG, "  Offset: %d", data_offset);
    ESP_LOGW(TAG, "  RqLen: %d", data_len);
    ESP_LOGW(TAG, "  RsLen: %d", rs_data_len);
    ESP_LOGW(TAG, "  At: %d", uint8_t(at));
    return false; //Start again
  }

  return true;

}

void Modbus::handle_request() {

  const uint8_t *data = &this->rx_buffer_[0];

  uint8_t frame_address = data[0];
  uint8_t function_code = data[1];
  ModbusFunctionCode code = static_cast<ModbusFunctionCode>(function_code);
  uint16_t start_address = 0;
  uint16_t quantity = 0;
  uint8_t byte_count = 0;

  if (code == ModbusFunctionCode::ReadCoils ||
      code == ModbusFunctionCode::ReadDiscreteInputs ||
      code == ModbusFunctionCode::ReadHoldingRegisters ||
      code == ModbusFunctionCode::ReadInputRegisters) {
    start_address = uint16_t(data[2] << 8 | data[3]);
    quantity = uint16_t(data[4] << 8 | data[5]);
    ESP_LOGW(TAG, "Device 0x%02X Requesting %s(0x%02X) %d elements starting at address %d.",
             frame_address, get_function_code_name(function_code), function_code, quantity, start_address);
  }
  else if (code == ModbusFunctionCode::WriteMultipleCoils ||
           code == ModbusFunctionCode::WriteMultipleRegisters) {
    start_address = uint16_t(data[2] << 8 | data[3]);
    quantity = uint16_t(data[4] << 8 | data[5]);
    byte_count = data[6];
    ESP_LOGW(TAG, "Device 0x%02X Request %s(0x%02X): %d elements (%d bytes) starting at address %d.",
             frame_address, get_function_code_name(function_code), function_code, quantity, byte_count, start_address);
    //ESP_LOGW(TAG, "  %s", format_hex_pretty(&data[7], byte_count).c_str());
    if (code == ModbusFunctionCode::WriteMultipleRegisters) {
      char buffer[2048];
      int offset = 0;
      for (int byte_index = 0; byte_index < byte_count; byte_index += 2) {
        int value = data[7 + byte_index] << 8 | data[8 + byte_index];
        offset += sprintf(buffer + offset, "%d, ", value);
        //offset += sprintf(buffer + offset, "%d::%d, ", byte_index, byte_count);
      }
      ESP_LOGI(TAG, " Values: %s", buffer);

    }
  
    for (auto *server : this->servers_) {
      if (server->address_ == frame_address || (server->accept_broadcast_ && frame_address == 0)) {
        ESP_LOGW(TAG, "Found matching server with address %d", server->address_);
        if (server->register_start_ >= start_address && (server->register_start_ + server->register_count_) <= (start_address + quantity)) {
          int listener_offset = server->register_start_ - start_address;
          server->on_register_update(&data[7+listener_offset], byte_count);
        }
      }
    }
  }
}

void Modbus::handle_response() {

  const uint8_t *data = &this->rx_buffer_[0];

  uint8_t frame_address = data[0];
  uint8_t function_code = data[1];
  ModbusFunctionCode code = static_cast<ModbusFunctionCode>(function_code);
  uint16_t start_address = 0;
  uint16_t quantity = 0;
  uint8_t byte_count = 0;

  if (code == ModbusFunctionCode::ReadCoils ||
      code == ModbusFunctionCode::ReadDiscreteInputs ||
      code == ModbusFunctionCode::ReadHoldingRegisters ||
      code == ModbusFunctionCode::ReadInputRegisters) {
    byte_count = data[2];
    ESP_LOGW(TAG, "Device 0x%02X Responding %s(0x%02X) with %d bytes of data:", frame_address, get_function_code_name(function_code), function_code, byte_count);
    if (function_code == 0x3) {
      char buffer[1024];
      int offset = 0;
      for (uint8_t i = 3; i < byte_count+3; i+=2) {
        offset += sprintf(buffer + offset, "%d, ", (int) data[i]<<8 | data[i+1]); 
      }
      ESP_LOGI(TAG, " Values: %s", buffer);
    }
    //ESP_LOGW(TAG, "  %s", format_hex_pretty(&data[3], byte_count).c_str());
  } else if (code == ModbusFunctionCode::WriteMultipleCoils ||
             code == ModbusFunctionCode::WriteMultipleRegisters) {
    start_address = uint16_t(data[2] << 8 | data[3]);
    quantity = uint16_t(data[4] << 8 | data[5]);
    ESP_LOGW(TAG, "Device 0x%02X Responding %s(0x%02X):  Written %d elements starting at address %d.", frame_address, get_function_code_name(function_code), function_code, quantity, start_address);
  } else if ((function_code & 0x80) == 0x80) {
    uint8_t error_code = data[2];
    function_code &= 0x7F;
    ESP_LOGW(TAG, "Device 0x%02X function 0x%02X returned error code 0x%02X.", frame_address, function_code, error_code);
  }
}

void Modbus::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  Send Wait Time: %d ms", this->send_wait_time_);
  ESP_LOGCONFIG(TAG, "  CRC Disabled: %s", YESNO(this->disable_crc_));
  ESP_LOGCONFIG(TAG, "  Accept Broadcast: %s", YESNO(this->accept_broadcast_));
}
float Modbus::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void Modbus::send(uint8_t address, uint8_t function_code, uint16_t start_address, uint16_t number_of_entities,
                  uint8_t payload_len, const uint8_t *payload) {
  static const size_t MAX_VALUES = 128;

  // Only check max number of registers for standard function codes
  // Some devices use non standard codes like 0x43
  if (number_of_entities > MAX_VALUES && function_code <= 0x10) {
    ESP_LOGE(TAG, "send too many values %d max=%zu", number_of_entities, MAX_VALUES);
    return;
  }

  std::vector<uint8_t> data;
  data.push_back(address);
  data.push_back(function_code);
  if (this->role == ModbusRole::CLIENT) {
    data.push_back(start_address >> 8);
    data.push_back(start_address >> 0);
    if (function_code != 0x5 && function_code != 0x6) {
      data.push_back(number_of_entities >> 8);
      data.push_back(number_of_entities >> 0);
    }
  }

  if (payload != nullptr) {
    if (this->role == ModbusRole::SERVER || function_code == 0xF || function_code == 0x10) {  // Write multiple
      data.push_back(payload_len);  // Byte count is required for write
    } else {
      payload_len = 2;  // Write single register or coil
    }
    for (int i = 0; i < payload_len; i++) {
      data.push_back(payload[i]);
    }
  }

  auto crc = crc16(data.data(), data.size());
  data.push_back(crc >> 0);
  data.push_back(crc >> 8);

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  this->write_array(data);
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
  waiting_for_response = address;
  last_send_ = millis();
  ESP_LOGV(TAG, "Modbus write: %s", format_hex_pretty(data).c_str());
}

// Helper function for lambdas
// Send raw command. Except CRC everything must be contained in payload
void Modbus::send_raw(const std::vector<uint8_t> &payload) {
  if (payload.empty()) {
    return;
  }

  // disable transmitting for now
  //if (this->flow_control_pin_ != nullptr)
  //  this->flow_control_pin_->digital_write(true);

  auto crc = crc16(payload.data(), payload.size());
  this->write_array(payload);
  this->write_byte(crc & 0xFF);
  this->write_byte((crc >> 8) & 0xFF);
  this->flush();
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
  waiting_for_response = payload[0];
  ESP_LOGV(TAG, "Modbus write raw: %s", format_hex_pretty(payload).c_str());
  last_send_ = millis();
}

}  // namespace modbus
}  // namespace esphome
