#pragma once

#include "imu_motion_monitor_config.hpp"

#include <atlas/imu/ImuSample.hpp>
#include <atlas/sensors/SensorWarning.hpp>
#include <chrono>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace atlas::preprocessing {

class ImuMotionMonitor {
 public:
  explicit ImuMotionMonitor(ImuMotionMonitorConfig config);

  [[nodiscard]] std::vector<atlas::sensors::SensorWarning> inspect(
      const atlas::imu::ImuSample& sample);
  [[nodiscard]] std::optional<atlas::sensors::SensorWarning> inspect_estimated_speed(
      uint32_t sensor_id, atlas::timesync::SensorTimestamp timestamp, float speed_m_s);

 private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  struct SensorState {
    bool has_acceleration = false;
    uint64_t previous_timestamp_ns = 0;
    float previous_accel_x = 0.0F;
    float previous_accel_y = 0.0F;
    float previous_accel_z = 0.0F;
    SteadyTime last_acceleration_warning{};
    SteadyTime last_rotation_warning{};
    SteadyTime last_speed_warning{};
  };

  [[nodiscard]] bool cooldown_elapsed(SteadyTime last_warning, SteadyTime now) const;

  ImuMotionMonitorConfig config_;
  std::unordered_map<uint32_t, SensorState> states_;
};

}  // namespace atlas::preprocessing
