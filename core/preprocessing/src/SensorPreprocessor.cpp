#include <atlas/preprocessing/SensorPreprocessor.hpp>
#include <atomic>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "ImuMotionMonitor.hpp"
#include "LiDARFilter.hpp"

namespace atlas::preprocessing {

class SensorPreprocessor::Impl {
 public:
  Impl(atlas::sensors::SensorDataBus& output, const SensorPreprocessorConfig& config)
      : output_(output),
        lidar_filter_(config.minimum_lidar_distance_m),
        imu_monitor_({
            .abrupt_jerk_threshold_m_s3 = config.abrupt_jerk_threshold_m_s3,
            .abrupt_angular_speed_threshold_rad_s = config.abrupt_angular_speed_threshold_rad_s,
            .excessive_speed_threshold_m_s = config.excessive_speed_threshold_m_s,
            .warning_cooldown = config.warning_cooldown,
        }) {}

  void on_lidar_frame(const atlas::sensors::LidarFrame& frame) {
    lidar_points_received_.fetch_add(frame.points.size(), std::memory_order_relaxed);
    auto result = lidar_filter_.filter(frame);
    lidar_points_rejected_.fetch_add(result.rejected_points, std::memory_order_relaxed);
    output_.publish(result.frame);
  }

  void on_imu_sample(const atlas::imu::ImuSample& sample) {
    imu_samples_received_.fetch_add(1, std::memory_order_relaxed);
    std::vector<atlas::sensors::SensorWarning> warnings;
    {
      std::lock_guard lock(monitor_mutex_);
      warnings = imu_monitor_.inspect(sample);
    }
    for (const auto& warning : warnings) {
      publish_warning(warning);
    }
    output_.publish(sample);
  }

  void on_sensor_status(const atlas::sensors::SensorStatus& status) { output_.publish(status); }
  void on_camera_frame(const atlas::sensors::CameraFrame& frame) { output_.publish(frame); }

  void on_sensor_warning(const atlas::sensors::SensorWarning& warning) { output_.publish(warning); }

  void observe_estimated_speed(const uint32_t sensor_id,
                               const atlas::timesync::SensorTimestamp timestamp,
                               const float speed_m_s) {
    std::optional<atlas::sensors::SensorWarning> warning;
    {
      std::lock_guard lock(monitor_mutex_);
      warning = imu_monitor_.inspect_estimated_speed(sensor_id, timestamp, speed_m_s);
    }
    if (warning) {
      publish_warning(*warning);
    }
  }

  [[nodiscard]] SensorPreprocessorStats stats() const noexcept {
    return {
        .lidar_points_received = lidar_points_received_.load(),
        .lidar_points_rejected = lidar_points_rejected_.load(),
        .imu_samples_received = imu_samples_received_.load(),
        .warnings_published = warnings_published_.load(),
    };
  }

 private:
  void publish_warning(const atlas::sensors::SensorWarning& warning) {
    warnings_published_.fetch_add(1, std::memory_order_relaxed);
    output_.publish(warning);
  }

  atlas::sensors::SensorDataBus& output_;
  LiDARFilter lidar_filter_;
  ImuMotionMonitor imu_monitor_;
  std::mutex monitor_mutex_;
  std::atomic_uint64_t lidar_points_received_{0};
  std::atomic_uint64_t lidar_points_rejected_{0};
  std::atomic_uint64_t imu_samples_received_{0};
  std::atomic_uint64_t warnings_published_{0};
};

SensorPreprocessor::SensorPreprocessor(atlas::sensors::SensorDataBus& output,
                                       SensorPreprocessorConfig config)
    : impl_(std::make_unique<Impl>(output, config)) {}

SensorPreprocessor::~SensorPreprocessor() = default;

void SensorPreprocessor::on_camera_frame(const atlas::sensors::CameraFrame& frame) {
  impl_->on_camera_frame(frame);
}

void SensorPreprocessor::on_lidar_frame(const atlas::sensors::LidarFrame& frame) {
  impl_->on_lidar_frame(frame);
}

void SensorPreprocessor::on_imu_sample(const atlas::imu::ImuSample& sample) {
  impl_->on_imu_sample(sample);
}

void SensorPreprocessor::on_sensor_status(const atlas::sensors::SensorStatus& status) {
  impl_->on_sensor_status(status);
}

void SensorPreprocessor::on_sensor_warning(const atlas::sensors::SensorWarning& warning) {
  impl_->on_sensor_warning(warning);
}

void SensorPreprocessor::observe_estimated_speed(const uint32_t sensor_id,
                                                 const atlas::timesync::SensorTimestamp timestamp,
                                                 const float speed_m_s) {
  impl_->observe_estimated_speed(sensor_id, timestamp, speed_m_s);
}

SensorPreprocessorStats SensorPreprocessor::stats() const noexcept { return impl_->stats(); }

}  // namespace atlas::preprocessing
