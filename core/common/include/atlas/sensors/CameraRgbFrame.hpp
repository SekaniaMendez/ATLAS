#pragma once
#include <array>
#include <cstdint>
#include <vector>

namespace atlas::sensors {
/// Native unrotated sRGB image; ARKit camera axes (+X right, +Y up, -Z forward).
/// The rigid transform maps LiDAR coordinates to those camera axes.
struct CameraRgbFrame {
  uint64_t timestamp_ns = 0; // phone monotonic clock, never a LiDAR timestamp
  uint32_t width = 0, height = 0;
  std::array<double, 4> intrinsics{}; // fx, fy, cx, cy at this image resolution
  std::array<double, 9> camera_from_lidar_rotation{};
  std::array<double, 3> camera_from_lidar_translation{};
  bool calibration_verified = false;
  bool tracking_normal = false;
  std::vector<uint8_t> rgb; // tightly packed RGB, top row first
};
}
