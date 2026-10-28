#include <atlas/preprocessing/SensorPreprocessor.hpp>
#include <atlas/sensors/ISensorDataSink.hpp>
#include <cassert>
#include <chrono>
#include <memory>
#include <vector>

namespace {

class CapturingSink final : public atlas::sensors::ISensorDataSink {
 public:
  void on_lidar_frame(const atlas::sensors::LidarFrame& frame) override { last_frame = frame; }
  void on_imu_sample(const atlas::imu::ImuSample& /*sample*/) override { ++imu_samples; }
  void on_sensor_warning(const atlas::sensors::SensorWarning& warning) override {
    warnings.push_back(warning);
  }

  atlas::sensors::LidarFrame last_frame;
  size_t imu_samples = 0;
  std::vector<atlas::sensors::SensorWarning> warnings;
};

}  // namespace

int main() {
  atlas::sensors::SensorDataBus output;
  auto sink = std::make_shared<CapturingSink>();
  const auto subscription = output.subscribe(sink);

  atlas::preprocessing::SensorPreprocessor preprocessor(
      output, {
                  .minimum_lidar_distance_m = 0.60F,
                  .abrupt_jerk_threshold_m_s3 = 50.0F,
                  .abrupt_angular_speed_threshold_rad_s = 1.0F,
                  .excessive_speed_threshold_m_s = 2.5F,
                  .warning_cooldown = std::chrono::milliseconds{0},
              });

  atlas::sensors::LidarFrame frame;
  frame.points = {
      {.x_m = 0.59F},
      {.x_m = 0.60F},
      {.x_m = 1.0F},
  };
  preprocessor.on_lidar_frame(frame);
  assert(sink->last_frame.points.size() == 2);
  assert(sink->last_frame.points[0].x_m == 0.60F);

  atlas::imu::ImuSample first;
  first.sensor_id = 7;
  first.timestamp.nanoseconds = 1'000'000'000ULL;
  preprocessor.on_imu_sample(first);

  auto abrupt_acceleration = first;
  abrupt_acceleration.timestamp.nanoseconds += 10'000'000ULL;
  abrupt_acceleration.accel_x_m_s2 = 1.0F;
  preprocessor.on_imu_sample(abrupt_acceleration);
  assert(sink->warnings.size() == 1);
  assert(sink->warnings.back().code == atlas::sensors::SensorWarningCode::AbruptAcceleration);

  auto abrupt_rotation = abrupt_acceleration;
  abrupt_rotation.timestamp.nanoseconds += 10'000'000ULL;
  abrupt_rotation.gyro_z_rad_s = 1.5F;
  preprocessor.on_imu_sample(abrupt_rotation);
  assert(sink->warnings.size() == 2);
  assert(sink->warnings.back().code == atlas::sensors::SensorWarningCode::AbruptRotation);
  assert(sink->imu_samples == 3);

  preprocessor.observe_estimated_speed(7, abrupt_rotation.timestamp, 2.5F);
  assert(sink->warnings.size() == 2);
  preprocessor.observe_estimated_speed(7, abrupt_rotation.timestamp, 2.6F);
  assert(sink->warnings.size() == 3);
  assert(sink->warnings.back().code == atlas::sensors::SensorWarningCode::ExcessiveSpeed);

  const auto stats = preprocessor.stats();
  assert(stats.lidar_points_received == 3);
  assert(stats.lidar_points_rejected == 1);
  assert(stats.imu_samples_received == 3);
  assert(stats.warnings_published == 3);

  output.unsubscribe(subscription);
  return 0;
}
