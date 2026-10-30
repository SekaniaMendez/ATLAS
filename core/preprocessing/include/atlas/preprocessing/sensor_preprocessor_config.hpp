#pragma once

#include <chrono>

namespace atlas::preprocessing {

struct SensorPreprocessorConfig {
  float minimum_lidar_distance_m = 0.60F;
  float abrupt_jerk_threshold_m_s3 = 30.0F;
  float abrupt_angular_speed_threshold_rad_s = 3.0F;
  float excessive_speed_threshold_m_s = 2.5F;
  std::chrono::milliseconds warning_cooldown{1'000};
  /// Enable only for MID-360 tags. Reject value 2 (low confidence) in any
  /// of bits 0-1, 2-3 or 4-5; retain medium/unknown values and ignore bits 6-7.
  /// Generic sources default to no tag interpretation. Raw recording is unchanged.
  bool filter_mid360_low_confidence_tags = false;
};

}  // namespace atlas::preprocessing
