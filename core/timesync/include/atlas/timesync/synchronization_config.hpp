#pragma once

#include <cstddef>
#include <cstdint>

namespace atlas::timesync {

struct SynchronizationConfig {
  size_t lidar_capacity = 128, imu_capacity = 4096, camera_capacity = 32;
  size_t max_points_per_packet = 4096;
  size_t max_camera_metadata_bytes = 64 * 1024;
  uint64_t max_imu_gap_ns = 20'000'000;
  uint64_t camera_tolerance_ns = 100'000'000;
  uint64_t max_wait_ns = 200'000'000;
  uint64_t max_clock_uncertainty_ns = 5'000'000;
  bool require_camera = false;
};

}  // namespace atlas::timesync
