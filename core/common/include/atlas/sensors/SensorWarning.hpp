#pragma once

#include <atlas/timesync/SensorTimestamp.hpp>
#include <cstdint>

namespace atlas::sensors {

enum class SensorWarningCode : uint16_t {
  AbruptAcceleration = 1,
  AbruptRotation = 2,
  ExcessiveSpeed = 3,
};

enum class SensorWarningSeverity : uint8_t {
  Information = 0,
  Caution = 1,
  Critical = 2,
};

struct SensorWarning {
  uint32_t sensor_id = 0;
  atlas::timesync::SensorTimestamp timestamp;
  SensorWarningCode code = SensorWarningCode::AbruptAcceleration;
  SensorWarningSeverity severity = SensorWarningSeverity::Caution;
  float measured_value = 0.0F;
  float threshold = 0.0F;
};

}  // namespace atlas::sensors
