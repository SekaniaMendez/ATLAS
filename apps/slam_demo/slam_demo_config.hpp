#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace atlas::demo {

/// Synthetic geometry and motion are deliberately independent of the estimator.
/// The first 0.7 s are stationary; the remaining trajectory has constant world
/// translation and yaw rate. This is a controlled test, not a MID-360 simulator.
struct SlamDemoConfig {
  uint32_t sensor_id = 360;
  uint64_t start_timestamp_ns = 1'000'000'000;
  uint32_t scan_duration_ns = 100'000'000;
  uint64_t imu_period_ns = 5'000'000;
  uint64_t stationary_duration_ns = 700'000'000;
  size_t scan_count = 40;
  size_t points_per_scan = 5000;
  std::array<double, 3> room_min_m{-4.0, -3.0, -1.4};
  std::array<double, 3> room_max_m{6.0, 4.0, 2.2};
  std::array<double, 3> obstacle_min_m{2.1, -1.5, -1.4};
  std::array<double, 3> obstacle_max_m{2.8, -0.6, 0.8};
  std::array<double, 3> velocity_m_s{0.25, 0.12, 0.025};
  double yaw_rate_rad_s = 0.18;
  double initial_roll_rad = 0.0;
  double initial_pitch_rad = 0.0;
  double range_noise_stddev_m = 0.003;
  std::array<double, 3> gyro_bias_rad_s{0.003, -0.002, 0.004};
  double gravity_m_s2 = 9.80665;
  uint32_t random_seed = 360;
  bool motion_during_scan = true;
  double maximum_terminal_error_m = 0.15;
  double maximum_trajectory_rmse_m = 0.10;
  double maximum_map_surface_rmse_m = 0.06;
  double maximum_orientation_error_rad = 0.05;
  double minimum_accepted_scan_fraction = 0.70;
};

}  // namespace atlas::demo
