#pragma once

#include <atlas/preprocessing/sensor_preprocessor_config.hpp>
#include <chrono>

namespace atlas::preprocessing {

// Share the default thresholds with the public preprocessing configuration.
struct ImuMotionMonitorConfig {
  float abrupt_jerk_threshold_m_s3 = SensorPreprocessorConfig{}.abrupt_jerk_threshold_m_s3;
  float abrupt_angular_speed_threshold_rad_s =
      SensorPreprocessorConfig{}.abrupt_angular_speed_threshold_rad_s;
  float excessive_speed_threshold_m_s = SensorPreprocessorConfig{}.excessive_speed_threshold_m_s;
  std::chrono::milliseconds warning_cooldown = SensorPreprocessorConfig{}.warning_cooldown;
};

}  // namespace atlas::preprocessing
