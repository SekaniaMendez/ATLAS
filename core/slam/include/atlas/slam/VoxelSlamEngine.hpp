#pragma once

#include <atlas/slam/ISlamEngine.hpp>
#include <atlas/slam/SparseVoxelMap.hpp>
#include <atlas/slam/voxel_slam_config.hpp>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace atlas::slam {

struct SlamStats {
  uint64_t scans_processed = 0;
  uint64_t scans_accepted = 0;
  uint64_t scans_rejected = 0;
  uint64_t rejected_events = 0;
  uint64_t map_voxels = 0;
  uint64_t map_blocks = 0;
  uint64_t imu_samples = 0;
  size_t queued_bytes = 0;
  size_t peak_queued_bytes = 0;
  double last_rmse_m = 0.0;
  size_t last_correspondences = 0;
  bool fatal_error = false;
  TrackingState state = TrackingState::Uninitialized;
  std::string message = "Waiting for sensor data";
};

/// Single-sensor local mapping prototype. Callbacks only enqueue bounded copies;
/// one worker owns scan assembly, registration and the voxel maps. Public reads
/// return copies under a mutex. Caller must stop producers before finish.
/// reset() requires that ALL other calls (including readers) have stopped.
/// IMU must share sensor_id and device clock with LiDAR when use_imu is enabled.
class VoxelSlamEngine final : public ISlamEngine {
 public:
  explicit VoxelSlamEngine(VoxelSlamConfig config = {});
  ~VoxelSlamEngine() override;
  VoxelSlamEngine(const VoxelSlamEngine&) = delete;
  VoxelSlamEngine& operator=(const VoxelSlamEngine&) = delete;
  void on_lidar_frame(const sensors::LidarFrame&) override;
  void on_imu_sample(const imu::ImuSample&) override;
  void reset() override;
  [[nodiscard]] std::optional<PoseEstimate> latest_pose() const override;
  [[nodiscard]] SlamStats stats() const;
  [[nodiscard]] std::vector<MapPoint> map_points(size_t max_points) const;
  [[nodiscard]] std::vector<PoseEstimate> trajectory() const;
  /// Backpressure for unpaced replay/tests. Does not close the current scan.
  void wait_until_idle();
  /// Drain all enqueued events, process a final partial scan if valid, and stop.
  /// Idempotent. Incomplete IMU coverage is reported as a rejected scan.
  void finish();
  /// New destinations only; never overwrite a prior capture/result.
  void save_map(const std::filesystem::path& new_directory) const;
  void export_ply(const std::filesystem::path& new_file) const;
  void export_trajectory(const std::filesystem::path& new_file) const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace atlas::slam
