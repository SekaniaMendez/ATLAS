#pragma once

#include <atlas/slam/sparse_voxel_map_config.hpp>
#include <atlas/slam/voxel_registration_config.hpp>
#include <array>
#include <cstddef>
#include <cstdint>

namespace atlas::slam {

/// Prototype local LiDAR odometry with gyro prediction/rotational deskew.
/// Coordinate frame: local metric frame, gravity-aligned when IMU is enabled.
/// It does not supply a surveyed/geographic reference or loop closure.
struct VoxelSlamConfig {
  SparseVoxelMapConfig canvas{.voxel_size_m = 0.05, .max_samples_per_voxel = 1};
  SparseVoxelMapConfig matching_map{.voxel_size_m = 0.20, .max_samples_per_voxel = 8};
  VoxelRegistrationConfig registration;
  uint64_t scan_duration_ns = 100'000'000;
  uint64_t max_scan_duration_ns = 250'000'000;
  size_t minimum_scan_points = 100;
  size_t maximum_scan_points = 40'000;
  double input_voxel_size_m = 0.08;
  double minimum_range_m = 0.60;
  double maximum_range_m = 40.0;
  size_t max_queue_bytes = 32 * 1024 * 1024;
  size_t max_pending_scans = 4;
  size_t max_canvas_blocks = 20'000;
  size_t max_matching_blocks = 8'000;
  size_t max_trajectory_poses = 100'000;
  bool use_imu = true;
  uint64_t imu_initialization_ns = 500'000'000;
  size_t minimum_initialization_imu_samples = 50;
  uint64_t maximum_imu_gap_ns = 30'000'000;
  uint64_t imu_history_ns = 3'000'000'000;
  size_t maximum_imu_samples = 4'096;
  double gravity_m_s2 = 9.80665;
  double stationary_gyro_limit_rad_s = 0.03;
  double stationary_accel_tolerance_m_s2 = 1.0;
  double stationary_accel_stddev_m_s2 = 0.20;
  /// Rotation of IMU vectors into the LiDAR axes, quaternion x,y,z,w.
  /// Identity assumes co-oriented axes; verify for each sensor integration.
  std::array<double, 4> imu_to_lidar_rotation_xyzw{0.0, 0.0, 0.0, 1.0};
  double maximum_prediction_interval_s = 0.30;
  double maximum_speed_m_s = 3.0;
  double velocity_smoothing = 0.5;
  /// Keep the registration reference fixed while views barely change. This
  /// avoids feeding every small registration error back into the reference.
  double matching_keyframe_translation_m = 0.15;
  double matching_keyframe_rotation_rad = 0.10;
};

}  // namespace atlas::slam
