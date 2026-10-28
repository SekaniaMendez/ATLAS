#pragma once

#include "slam_demo_config.hpp"

#include <atlas/imu/ImuSample.hpp>
#include <atlas/sensors/LidarFrame.hpp>
#include <atlas/slam/VoxelSlamEngine.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <vector>

namespace atlas::demo {

struct SyntheticPose {
  std::array<double, 3> position_m{};
  /// Unit quaternion in x,y,z,w order, sensor -> world.
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
};

struct TrajectoryMetrics {
  size_t pose_count = 0;
  double position_rmse_m = 0.0;
  double maximum_position_error_m = 0.0;
  double terminal_position_error_m = 0.0;
  double maximum_orientation_error_rad = 0.0;
  double terminal_orientation_error_rad = 0.0;
};

struct MapMetrics {
  size_t point_count = 0;
  double surface_rmse_m = 0.0;
  double maximum_surface_error_m = 0.0;
};

/// Ray casts from a moving sensor against fixed, finite world surfaces. Every
/// beam gets its own acquisition time. No registration or map code generates
/// the truth: the six room faces and an offset solid box define the scene.
class SyntheticRoom {
 public:
  explicit SyntheticRoom(SlamDemoConfig config = {}) : config_(config) {
    if (config_.points_per_scan < 2 || config_.scan_duration_ns == 0 ||
        config_.imu_period_ns == 0)
      throw std::invalid_argument("Invalid synthetic sampling configuration");
    for (size_t axis = 0; axis < 3; ++axis) {
      if (!(config_.room_min_m[axis] < config_.room_max_m[axis]) ||
          !(config_.obstacle_min_m[axis] < config_.obstacle_max_m[axis]))
        throw std::invalid_argument("Invalid synthetic room bounds");
    }
  }

  [[nodiscard]] const SlamDemoConfig& config() const noexcept { return config_; }

  [[nodiscard]] uint64_t scan_start_ns(size_t index) const {
    return config_.start_timestamp_ns +
           static_cast<uint64_t>(index) * config_.scan_duration_ns;
  }

  [[nodiscard]] SyntheticPose pose_at(uint64_t timestamp_ns) const {
    const uint64_t motion_start = config_.start_timestamp_ns + config_.stationary_duration_ns;
    const double t = timestamp_ns > motion_start
                         ? static_cast<double>(timestamp_ns - motion_start) * 1e-9
                         : 0.0;
    SyntheticPose pose;
    for (size_t axis = 0; axis < 3; ++axis)
      pose.position_m[axis] = config_.velocity_m_s[axis] * t;
    const double roll = config_.initial_roll_rad * 0.5;
    const double pitch = config_.initial_pitch_rad * 0.5;
    const double yaw = config_.yaw_rate_rad_s * t * 0.5;
    // Standard Rz(yaw) Ry(pitch) Rx(roll), explicit to keep truth independent.
    pose.orientation_xyzw = {
        std::sin(roll) * std::cos(pitch) * std::cos(yaw) -
            std::cos(roll) * std::sin(pitch) * std::sin(yaw),
        std::cos(roll) * std::sin(pitch) * std::cos(yaw) +
            std::sin(roll) * std::cos(pitch) * std::sin(yaw),
        std::cos(roll) * std::cos(pitch) * std::sin(yaw) -
            std::sin(roll) * std::sin(pitch) * std::cos(yaw),
        std::cos(roll) * std::cos(pitch) * std::cos(yaw) +
            std::sin(roll) * std::sin(pitch) * std::sin(yaw)};
    return pose;
  }

  [[nodiscard]] imu::ImuSample imu_at(uint64_t timestamp_ns) const {
    const auto pose = pose_at(timestamp_ns);
    const bool moving = timestamp_ns >= config_.start_timestamp_ns + config_.stationary_duration_ns;
    // Yaw is about world Z. Express that angular velocity and stationary
    // specific force in the body frame, then add a fixed calibratable bias.
    const auto gyro = rotate(pose.orientation_xyzw, {0.0, 0.0, moving ? config_.yaw_rate_rad_s : 0.0}, true);
    const auto accel = rotate(pose.orientation_xyzw, {0.0, 0.0, config_.gravity_m_s2}, true);
    imu::ImuSample sample;
    sample.sensor_id = config_.sensor_id;
    sample.timestamp = {timestamp_ns, timesync::ClockSource::Unsynchronized};
    sample.gyro_x_rad_s = static_cast<float>(gyro[0] + config_.gyro_bias_rad_s[0]);
    sample.gyro_y_rad_s = static_cast<float>(gyro[1] + config_.gyro_bias_rad_s[1]);
    sample.gyro_z_rad_s = static_cast<float>(gyro[2] + config_.gyro_bias_rad_s[2]);
    sample.accel_x_m_s2 = static_cast<float>(accel[0]);
    sample.accel_y_m_s2 = static_cast<float>(accel[1]);
    sample.accel_z_m_s2 = static_cast<float>(accel[2]);
    return sample;
  }

  [[nodiscard]] sensors::LidarFrame frame(size_t index) const {
    sensors::LidarFrame frame;
    frame.sensor_id = config_.sensor_id;
    frame.packet_sequence = static_cast<uint16_t>(index);
    frame.frame_index = static_cast<uint8_t>(index);
    frame.timestamp = {scan_start_ns(index), timesync::ClockSource::Unsynchronized};
    frame.duration_ns = config_.scan_duration_ns;
    frame.points.reserve(config_.points_per_scan);
    std::mt19937 generator(config_.random_seed + static_cast<uint32_t>(index));
    std::normal_distribution<double> noise(0.0, config_.range_noise_stddev_m);
    constexpr double golden_angle = std::numbers::pi * (3.0 - 2.2360679774997896964);
    for (size_t i = 0; i < config_.points_per_scan; ++i) {
      const uint32_t offset = static_cast<uint32_t>(
          i * static_cast<uint64_t>(config_.scan_duration_ns - 1) /
          (config_.points_per_scan - 1));
      const uint64_t acquisition_ns = frame.timestamp.nanoseconds +
          (config_.motion_during_scan ? offset : config_.scan_duration_ns - 1);
      const auto pose = pose_at(acquisition_ns);
      // Fibonacci sphere: stable surface coverage, varying ranges and
      // non-grid-aligned samples. This deliberately uses full spherical FOV.
      const double z = 1.0 - 2.0 * (static_cast<double>(i) + 0.5) /
                                     static_cast<double>(config_.points_per_scan);
      const double radial = std::sqrt(std::max(0.0, 1.0 - z * z));
      const double azimuth = golden_angle * static_cast<double>(i);
      const std::array<double, 3> beam{radial * std::cos(azimuth), radial * std::sin(azimuth), z};
      const auto direction = rotate(pose.orientation_xyzw, beam, false);
      const double distance = raycast(pose.position_m, direction);
      if (!std::isfinite(distance)) throw std::runtime_error("Synthetic ray left the closed room");
      const double measured_distance = distance + noise(generator);
      frame.points.push_back({
          .x_m = static_cast<float>(beam[0] * measured_distance),
          .y_m = static_cast<float>(beam[1] * measured_distance),
          .z_m = static_cast<float>(beam[2] * measured_distance),
          .offset_ns = offset,
          .reflectivity = 100});
    }
    return frame;
  }

  /// IMU is enqueued with look-ahead before each scan; timestamps remain
  /// monotonic within each sensor stream. Backpressure keeps replay bounded.
  void feed(slam::VoxelSlamEngine& engine, bool with_imu = true) const {
    uint64_t next_imu_ns = config_.start_timestamp_ns;
    for (size_t index = 0; index < config_.scan_count; ++index) {
      feed_scan(engine, index, next_imu_ns, with_imu);
    }
  }

  void feed_scan(slam::VoxelSlamEngine& engine, size_t index,
                 uint64_t& next_imu_ns, bool with_imu = true) const {
    if (with_imu) {
      const auto imu_end = scan_start_ns(index) + config_.scan_duration_ns + config_.imu_period_ns;
      for (; next_imu_ns <= imu_end; next_imu_ns += config_.imu_period_ns)
        engine.on_imu_sample(imu_at(next_imu_ns));
    }
    engine.on_lidar_frame(frame(index));
    engine.wait_until_idle();
  }

  [[nodiscard]] TrajectoryMetrics evaluate(const std::vector<slam::PoseEstimate>& trajectory) const {
    TrajectoryMetrics metrics;
    double squared_error_sum = 0.0;
    for (const auto& estimate : trajectory) {
      const auto truth = pose_at(estimate.timestamp.nanoseconds);
      const std::array<double, 3> delta{estimate.x_m - truth.position_m[0],
                                      estimate.y_m - truth.position_m[1],
                                      estimate.z_m - truth.position_m[2]};
      const double squared_error = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
      const double position_error = std::sqrt(squared_error);
      const auto& q = truth.orientation_xyzw;
      const double dot = std::abs(q[0] * estimate.qx + q[1] * estimate.qy +
                                 q[2] * estimate.qz + q[3] * estimate.qw);
      const double angle_error = 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
      ++metrics.pose_count;
      squared_error_sum += squared_error;
      metrics.maximum_position_error_m = std::max(metrics.maximum_position_error_m, position_error);
      metrics.maximum_orientation_error_rad = std::max(metrics.maximum_orientation_error_rad, angle_error);
      metrics.terminal_position_error_m = position_error;
      metrics.terminal_orientation_error_rad = angle_error;
    }
    if (metrics.pose_count != 0)
      metrics.position_rmse_m = std::sqrt(squared_error_sum / static_cast<double>(metrics.pose_count));
    return metrics;
  }

  [[nodiscard]] bool within_tolerance(const TrajectoryMetrics& metrics,
                                      const slam::SlamStats& stats) const {
    const size_t stationary_scans = config_.stationary_duration_ns / config_.scan_duration_ns;
    const size_t moving_scans = config_.scan_count > stationary_scans
                                   ? config_.scan_count - stationary_scans : 1;
    return !stats.fatal_error && metrics.pose_count > 1 &&
           stats.scans_accepted >= static_cast<double>(moving_scans) * config_.minimum_accepted_scan_fraction &&
           metrics.terminal_position_error_m <= config_.maximum_terminal_error_m &&
           metrics.position_rmse_m <= config_.maximum_trajectory_rmse_m &&
           metrics.maximum_orientation_error_rad <= config_.maximum_orientation_error_rad;
  }

  [[nodiscard]] MapMetrics evaluate_map(const std::vector<slam::MapPoint>& points) const {
    MapMetrics metrics;
    double squared_sum = 0.0;
    for (const auto& point : points) {
      const std::array<double, 3> position{point.x_m, point.y_m, point.z_m};
      const double error = std::min(surface_distance(position, config_.room_min_m, config_.room_max_m),
                                    surface_distance(position, config_.obstacle_min_m, config_.obstacle_max_m));
      squared_sum += error * error;
      metrics.maximum_surface_error_m = std::max(metrics.maximum_surface_error_m, error);
      ++metrics.point_count;
    }
    if (metrics.point_count > 0)
      metrics.surface_rmse_m = std::sqrt(squared_sum / static_cast<double>(metrics.point_count));
    return metrics;
  }

 private:
  [[nodiscard]] static double surface_distance(const std::array<double, 3>& point,
                                                const std::array<double, 3>& minimum,
                                                const std::array<double, 3>& maximum) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t normal_axis = 0; normal_axis < 3; ++normal_axis) {
      for (double plane : {minimum[normal_axis], maximum[normal_axis]}) {
        double squared = 0.0;
        for (size_t axis = 0; axis < 3; ++axis) {
          const double projection = axis == normal_axis ? plane
              : std::clamp(point[axis], minimum[axis], maximum[axis]);
          const double residual = point[axis] - projection;
          squared += residual * residual;
        }
        best = std::min(best, std::sqrt(squared));
      }
    }
    return best;
  }

  [[nodiscard]] static std::array<double, 3> rotate(const std::array<double, 4>& q,
                                                  const std::array<double, 3>& v,
                                                  bool inverse) {
    const double sign = inverse ? -1.0 : 1.0;
    const std::array<double, 3> u{sign * q[0], sign * q[1], sign * q[2]};
    const std::array<double, 3> cross{u[1] * v[2] - u[2] * v[1],
                                     u[2] * v[0] - u[0] * v[2],
                                     u[0] * v[1] - u[1] * v[0]};
    const std::array<double, 3> cross2{u[1] * cross[2] - u[2] * cross[1],
                                      u[2] * cross[0] - u[0] * cross[2],
                                      u[0] * cross[1] - u[1] * cross[0]};
    return {v[0] + 2.0 * (q[3] * cross[0] + cross2[0]),
            v[1] + 2.0 * (q[3] * cross[1] + cross2[1]),
            v[2] + 2.0 * (q[3] * cross[2] + cross2[2])};
  }

  [[nodiscard]] double raycast(const std::array<double, 3>& origin,
                               const std::array<double, 3>& direction) const {
    double distance = std::numeric_limits<double>::infinity();
    // A sensor inside the room exits through exactly one of these six faces.
    for (size_t axis = 0; axis < 3; ++axis) {
      if (std::abs(direction[axis]) < 1e-12) continue;
      const double boundary = direction[axis] > 0.0
                                  ? config_.room_max_m[axis] : config_.room_min_m[axis];
      const double t = (boundary - origin[axis]) / direction[axis];
      if (t > 0.0) distance = std::min(distance, t);
    }
    // Slab intersection with the offset box handles visibility/occlusion.
    double near = 0.0;
    double far = distance;
    for (size_t axis = 0; axis < 3; ++axis) {
      if (std::abs(direction[axis]) < 1e-12) {
        if (origin[axis] < config_.obstacle_min_m[axis] ||
            origin[axis] > config_.obstacle_max_m[axis]) return distance;
        continue;
      }
      double first = (config_.obstacle_min_m[axis] - origin[axis]) / direction[axis];
      double second = (config_.obstacle_max_m[axis] - origin[axis]) / direction[axis];
      if (first > second) std::swap(first, second);
      near = std::max(near, first);
      far = std::min(far, second);
      if (near > far) return distance;
    }
    return near > 0.0 ? std::min(distance, near) : distance;
  }

  SlamDemoConfig config_;
};

}  // namespace atlas::demo
