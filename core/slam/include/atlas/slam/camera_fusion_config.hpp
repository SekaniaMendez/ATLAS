#pragma once
#include <atlas/timesync/SensorTimestamp.hpp>
#include <cstdint>
#include <cstddef>

namespace atlas::slam {
struct CameraFusionConfig {
  bool clock_alignment_verified = false;
  int64_t phone_to_lidar_offset_ns = 0;
  uint64_t clock_uncertainty_ns = UINT64_MAX;
  timesync::ClockSource lidar_clock = timesync::ClockSource::Unknown;
  uint64_t maximum_time_error_ns = 15'000'000;
  uint64_t maximum_wait_ns = 80'000'000;
  size_t maximum_frames = 6;
  uint32_t maximum_image_pixels = 640 * 480;
  double occlusion_tolerance_m = 0.05;
  uint32_t depth_cell_pixels = 4;
};
}
