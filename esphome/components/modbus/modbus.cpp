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
  if (function_code == 0x1 || function_code == 0x2 || function_code == 0x3 || function_code == 0x4) {
    data_len = 4;
    if (at > 1) {
	rs_data_len = uint8_t(raw[2]) + 1;
    } else {
	rs_data_len = 2;
    }
  } else if (function_code == 0x5 || function_code == 0x6) {
    data_len = 4;
    rs_data_len = 4;
  } else if (function_code == 0x7) {
    data_len = 0;
    rs_data_len = 1;
  } else if (function_code == 0x9) {
    data_len = 0;
    rs_data_len = 4;
  } else if (function_code == 0xF || function_code == 0x10) {
    if (at > 5) {
      data_len = uint8_t(raw[6]) + 5;
    } else {
      data_len = 6;
    } 
    rs_data_len = 4;
  } else if ((function_code & 0x80) == 0x80) {
    data_len = 1;
  }

  // We need the CRC bytes for either a request or a response 
  if ((at < data_offset + data_len + 1) && (at < data_offset + rs_data_len + 1))
    return true;

  // Are we at the right length for a request or response?
  if (at == data_offset + data_len + 1) {
    // We might have a request here
    uint16_t computed_crc = crc16(raw, data_offset + data_len);
    uint16_t remote_crc = uint16_t(raw[data_offset + data_len]) | (uint16_t(raw[data_offset + data_len + 1]) << 8);

    if (computed_crc == remote_crc) {
      ESP_LOGW(TAG, "Modbus CRC Check matches! %02X==%02X", computed_crc, remote_crc);
      ESP_LOGW(TAG, "  Function: %02X request, Len: %02X", function_code, data_len);
      ESP_LOGW(TAG, "  Frame: %s", format_hex_pretty(raw,at+1).c_str());
      return false; // Start a new frame
    }
  } else if (at == data_offset + rs_data_len + 1) {
    // Check for a response
    uint16_t computed_crc = crc16(raw, data_offset + rs_data_len);
    uint16_t remote_crc = uint16_t(raw[data_offset + rs_data_len]) | (uint16_t(raw[data_offset + rs_data_len + 1]) << 8);

    if (computed_crc == remote_crc) {
      ESP_LOGW(TAG, "Modbus CRC Check matches! %02X==%02X", computed_crc, remote_crc);
      ESP_LOGW(TAG, "  Function: %02X response, Len: %02X", function_code, rs_data_len);
      ESP_LOGW(TAG, "  Frame: %s", format_hex_pretty(raw,at+1).c_str());
      return false; // Start a new frame
    }
  } 
  
  if ((at > data_offset + data_len + 1) && (at > data_offset + rs_data_len + 1)) {
    ESP_LOGW(TAG, "Frame did not match: %s", format_hex_pretty(raw,at+1).c_str());
    return false; //Start again
  }


  //ESP_LOGW(TAG, "  Frame: %s", format_hex_pretty(raw, data_offset + data_len).c_str());
  //ESP_LOGW(TAG, "  Offset: %d", data_offset);
  //ESP_LOGW(TAG, "  RqLen: %d", data_len);
  //ESP_LOGW(TAG, "  RsLen: %d", rs_data_len);
  //ESP_LOGW(TAG, "  At: %d", uint8_t(at));
  return true;

}

void Modbus::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  Send Wait Time: %d ms", this->send_wait_time_);
  ESP_LOGCONFIG(TAG, "  CRC Disabled: %s", YESNO(this->disable_crc_));
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

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

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
