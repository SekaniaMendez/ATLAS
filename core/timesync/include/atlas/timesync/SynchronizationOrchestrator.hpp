#pragma once
#include <atlas/timesync/synchronization_config.hpp>
#include <atlas/timesync/StreamSynchronizer.hpp>
#include <atlas/sensors/ISensorDataSink.hpp>
#include <mutex>

namespace atlas::timesync {
struct SynchronizedPacket {
  uint64_t epoch = 0;
  uint64_t begin_ns = 0, end_ns = 0;
  sensors::LidarFrame lidar;
  std::vector<TimedSample<imu::ImuSample>> imu; // includes bracketing samples
  std::optional<TimedSample<sensors::CameraFrame>> camera;
  bool imu_complete = false, camera_matched = false, clocks_precise = false;
  bool ready_for_estimation = false;
  uint64_t clock_uncertainty_ns = 0;
};
struct SynchronizationStats {
  uint64_t accepted = 0, rejected = 0, emitted = 0, incomplete = 0;
  uint64_t reset_discarded = 0;
  SyncPushResult last_rejection = SyncPushResult::Accepted;
};

/// Thread-safe coordinator for one LiDAR, IMU and camera. It assembles time
/// intervals; it does NOT estimate clock offsets, deskew, interpolate IMU or fuse
/// poses. Missing/inaccurate data stays explicit, even at a timeout or end of file.
class SynchronizationOrchestrator final : public sensors::ISensorDataSink {
 public:
  explicit SynchronizationOrchestrator(SynchronizationConfig config = {});
  void configure_clocks(std::optional<ClockAlignment> lidar, std::optional<ClockAlignment> imu,
                        std::optional<ClockAlignment> camera = std::nullopt);
  SyncPushResult push(const sensors::LidarFrame&);
  SyncPushResult push(const imu::ImuSample&);
  SyncPushResult push(const sensors::CameraFrame&);
  void on_lidar_frame(const sensors::LidarFrame& v) override { (void)push(v); }
  void on_imu_sample(const imu::ImuSample& v) override { (void)push(v); }
  void on_camera_frame(const sensors::CameraFrame& v) override { (void)push(v); }
  /// now_ns is in the configured reference clock, NOT wall-clock arrival time.
  /// end_of_stream drains remaining LiDAR packets, marking gaps/incompleteness.
  [[nodiscard]] std::optional<SynchronizedPacket> poll(uint64_t now_ns, bool end_of_stream = false);
  [[nodiscard]] SynchronizationStats stats() const;
 private:
  SyncPushResult account(SyncPushResult result);
  SynchronizationConfig config_;
  mutable std::mutex mutex_;
  LidarSynchronizer lidar_;
  ImuSynchronizer imu_;
  CameraSynchronizer camera_;
  SynchronizationStats stats_;
  uint64_t epoch_ = 0;
};
}
