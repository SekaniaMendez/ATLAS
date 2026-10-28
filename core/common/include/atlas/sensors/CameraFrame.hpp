#pragma once
#include <atlas/timesync/SensorTimestamp.hpp>
#include <array>
#include <string>

namespace atlas::sensors {
/// Keyframe metadata; image bytes live in an external session directory.
/// Intrinsics and world_from_camera are row-major, in native unrotated image
/// coordinates and ARKit's local frame. This pose is not a LiDAR/project pose.
struct CameraFrame {
  uint32_t sensor_id = 0;
  atlas::timesync::SensorTimestamp timestamp;
  uint32_t width = 0, height = 0;
  std::array<double, 9> intrinsics{};
  std::array<double, 16> world_from_camera{};
  std::string image_path;
  std::string calibration_json;
};
}
