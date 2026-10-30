#pragma once

#include <atlas/timesync/SensorTimestamp.hpp>
#include <cstdint>
#include <array>
#include <vector>

namespace atlas::sensors {

struct LidarPoint {
  float x_m = 0.0F;
  float y_m = 0.0F;
  float z_m = 0.0F;
  uint32_t offset_ns = 0;
  uint8_t reflectivity = 0;
  uint8_t tag = 0;
  std::array<uint8_t, 3> rgb{};
  bool has_color = false;
  uint64_t color_timestamp_ns = 0; // original camera exposure, for independent evidence
};

struct LidarFrame {
  uint32_t sensor_id = 0;
  uint16_t packet_sequence = 0;
  uint8_t frame_index = 0;
  atlas::timesync::SensorTimestamp timestamp;
  uint32_t duration_ns = 0;
  std::vector<LidarPoint> points;
};

}  // namespace atlas::sensors
