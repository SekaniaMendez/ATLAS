#pragma once

#include <chrono>

namespace atlas::preprocessing {

struct SensorPreprocessorConfig {
  float minimum_lidar_distance_m = 0.60F;
  float abrupt_jerk_threshold_m_s3 = 30.0F;
  float abrupt_angular_speed_threshold_rad_s = 3.0F;
  float excessive_speed_threshold_m_s = 2.5F;
  std::chrono::milliseconds warning_cooldown{1'000};
};

}  // namespace atlas::preprocessing
