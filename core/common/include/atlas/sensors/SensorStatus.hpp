#pragma once

#include <cstdint>
#include <string>

namespace atlas::sensors {

enum class ConnectionState : uint8_t {
  Disconnected,
  Connecting,
  Streaming,
  Error,
};

struct SensorStatus {
  uint32_t sensor_id = 0;
  ConnectionState state = ConnectionState::Disconnected;
  std::string driver;
  std::string serial_number;
  std::string ip_address;
  std::string message;
};

}  // namespace atlas::sensors
