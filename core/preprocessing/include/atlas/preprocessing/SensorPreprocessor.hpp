#pragma once

#include <atlas/preprocessing/sensor_preprocessor_config.hpp>
#include <atlas/sensors/ISensorDataSink.hpp>
#include <atlas/sensors/SensorDataBus.hpp>
#include <atlas/timesync/SensorTimestamp.hpp>
#include <cstdint>
#include <memory>

namespace atlas::preprocessing {

struct SensorPreprocessorStats {
  uint64_t lidar_points_received = 0;
  uint64_t lidar_points_rejected = 0;
  /// Subset of rejected points; range/invalid-coordinate rejection takes precedence.
  uint64_t lidar_points_rejected_by_tag = 0;
  uint64_t imu_samples_received = 0;
  uint64_t warnings_published = 0;
};

class SensorPreprocessor final : public atlas::sensors::ISensorDataSink {
 public:
  explicit SensorPreprocessor(atlas::sensors::SensorDataBus& output,
                              SensorPreprocessorConfig config = {});
  ~SensorPreprocessor() override;

  SensorPreprocessor(const SensorPreprocessor&) = delete;
  SensorPreprocessor& operator=(const SensorPreprocessor&) = delete;

  void on_lidar_frame(const atlas::sensors::LidarFrame& frame) override;
  void on_camera_frame(const atlas::sensors::CameraFrame& frame) override;
  void on_imu_sample(const atlas::imu::ImuSample& sample) override;
  void on_sensor_status(const atlas::sensors::SensorStatus& status) override;
  void on_sensor_warning(const atlas::sensors::SensorWarning& warning) override;

  // SLAM can supply a drift-corrected speed here once velocity estimation exists.
  void observe_estimated_speed(uint32_t sensor_id, atlas::timesync::SensorTimestamp timestamp,
                               float speed_m_s);

  [[nodiscard]] SensorPreprocessorStats stats() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace atlas::preprocessing
