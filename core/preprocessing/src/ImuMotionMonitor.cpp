#include "ImuMotionMonitor.hpp"

#include <cmath>

namespace atlas::preprocessing {
namespace {

[[nodiscard]] bool finite_acceleration(const atlas::imu::ImuSample& sample) noexcept {
  return std::isfinite(sample.accel_x_m_s2) && std::isfinite(sample.accel_y_m_s2) &&
         std::isfinite(sample.accel_z_m_s2);
}

[[nodiscard]] float angular_speed(const atlas::imu::ImuSample& sample) noexcept {
  return std::sqrt(sample.gyro_x_rad_s * sample.gyro_x_rad_s +
                   sample.gyro_y_rad_s * sample.gyro_y_rad_s +
                   sample.gyro_z_rad_s * sample.gyro_z_rad_s);
}

}  // namespace

ImuMotionMonitor::ImuMotionMonitor(ImuMotionMonitorConfig config) : config_(config) {}

std::vector<atlas::sensors::SensorWarning> ImuMotionMonitor::inspect(
    const atlas::imu::ImuSample& sample) {
  std::vector<atlas::sensors::SensorWarning> warnings;
  auto& state = states_[sample.sensor_id];
  const auto now = std::chrono::steady_clock::now();

  if (finite_acceleration(sample)) {
    if (state.has_acceleration && sample.timestamp.nanoseconds > state.previous_timestamp_ns) {
      constexpr double kNanosecondsToSeconds = 1.0e-9;
      const double elapsed_seconds =
          static_cast<double>(sample.timestamp.nanoseconds - state.previous_timestamp_ns) *
          kNanosecondsToSeconds;
      if (elapsed_seconds >= 0.0001 && elapsed_seconds <= 0.5) {
        const float delta_x = sample.accel_x_m_s2 - state.previous_accel_x;
        const float delta_y = sample.accel_y_m_s2 - state.previous_accel_y;
        const float delta_z = sample.accel_z_m_s2 - state.previous_accel_z;
        const float acceleration_delta =
            std::sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
        const float jerk = acceleration_delta / static_cast<float>(elapsed_seconds);
        if (jerk > config_.abrupt_jerk_threshold_m_s3 &&
            cooldown_elapsed(state.last_acceleration_warning, now)) {
          warnings.push_back({
              .sensor_id = sample.sensor_id,
              .timestamp = sample.timestamp,
              .code = atlas::sensors::SensorWarningCode::AbruptAcceleration,
              .severity = atlas::sensors::SensorWarningSeverity::Caution,
              .measured_value = jerk,
              .threshold = config_.abrupt_jerk_threshold_m_s3,
          });
          state.last_acceleration_warning = now;
        }
      }
    }
    state.has_acceleration = true;
    state.previous_timestamp_ns = sample.timestamp.nanoseconds;
    state.previous_accel_x = sample.accel_x_m_s2;
    state.previous_accel_y = sample.accel_y_m_s2;
    state.previous_accel_z = sample.accel_z_m_s2;
  }

  const float rotation_speed = angular_speed(sample);
  if (std::isfinite(rotation_speed) &&
      rotation_speed > config_.abrupt_angular_speed_threshold_rad_s &&
      cooldown_elapsed(state.last_rotation_warning, now)) {
    warnings.push_back({
        .sensor_id = sample.sensor_id,
        .timestamp = sample.timestamp,
        .code = atlas::sensors::SensorWarningCode::AbruptRotation,
        .severity = atlas::sensors::SensorWarningSeverity::Caution,
        .measured_value = rotation_speed,
        .threshold = config_.abrupt_angular_speed_threshold_rad_s,
    });
    state.last_rotation_warning = now;
  }

  return warnings;
}

std::optional<atlas::sensors::SensorWarning> ImuMotionMonitor::inspect_estimated_speed(
    const uint32_t sensor_id, const atlas::timesync::SensorTimestamp timestamp,
    const float speed_m_s) {
  if (!std::isfinite(speed_m_s) || speed_m_s <= config_.excessive_speed_threshold_m_s) {
    return std::nullopt;
  }

  auto& state = states_[sensor_id];
  const auto now = std::chrono::steady_clock::now();
  if (!cooldown_elapsed(state.last_speed_warning, now)) {
    return std::nullopt;
  }
  state.last_speed_warning = now;
  return atlas::sensors::SensorWarning{
      .sensor_id = sensor_id,
      .timestamp = timestamp,
      .code = atlas::sensors::SensorWarningCode::ExcessiveSpeed,
      .severity = atlas::sensors::SensorWarningSeverity::Caution,
      .measured_value = speed_m_s,
      .threshold = config_.excessive_speed_threshold_m_s,
  };
}

bool ImuMotionMonitor::cooldown_elapsed(const SteadyTime last_warning, const SteadyTime now) const {
  return last_warning == SteadyTime{} || now - last_warning >= config_.warning_cooldown;
}

}  // namespace atlas::preprocessing
