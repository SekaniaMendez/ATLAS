#include <atlas/preprocessing/SensorPreprocessor.hpp>
#include <atlas/sensors/ISensorDataSink.hpp>
#include <cassert>
#include <chrono>
#include <limits>
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

void check_mid360_tag_filter() {
  using atlas::sensors::LidarFrame;
  atlas::sensors::SensorDataBus output;
  auto sink = std::make_shared<CapturingSink>();
  const auto subscription = output.subscribe(sink);
  atlas::preprocessing::SensorPreprocessor filtered(
      output, {.filter_mid360_low_confidence_tags = true});

  LidarFrame frame;
  frame.sensor_id = 42;
  frame.packet_sequence = 123;
  frame.frame_index = 7;
  frame.timestamp = {123456789, atlas::timesync::ClockSource::Unsynchronized};
  frame.duration_ns = 1000;
  // Normal, medium confidence (each field and combined), reserved field values,
  // and reserved high bits must survive. Do not use `tag == 0` as the filter.
  const std::vector<uint8_t> retained{0x00, 0x01, 0x04, 0x10, 0x15,
                                      0x03, 0x0c, 0x30, 0x3f, 0x40, 0x80, 0xc0};
  // Low confidence in each field, mixed with medium/reserved fields and high bits.
  const std::vector<uint8_t> rejected{0x02, 0x08, 0x20, 0x16, 0x19,
                                      0x25, 0x2a, 0xc2, 0xc8, 0xe0, 0x3e};
  for (auto tag : retained)
    frame.points.push_back({.x_m = 1.0F, .offset_ns = 900, .reflectivity = 81, .tag = tag});
  for (auto tag : rejected)
    frame.points.push_back({.x_m = 1.0F, .tag = tag});
  // Range/invalid-coordinate rejection takes precedence; no double counting.
  frame.points.push_back({.x_m = 0.59F, .tag = 0x02});
  frame.points.push_back({.x_m = std::numeric_limits<float>::quiet_NaN(), .tag = 0x08});
  filtered.on_lidar_frame(frame);
  const auto& result = sink->last_frame;
  assert(result.points.size() == retained.size());
  assert(result.sensor_id == frame.sensor_id && result.packet_sequence == frame.packet_sequence);
  assert(result.frame_index == frame.frame_index && result.duration_ns == frame.duration_ns);
  assert(result.timestamp.nanoseconds == frame.timestamp.nanoseconds &&
         result.timestamp.source == frame.timestamp.source);
  for (size_t i = 0; i < retained.size(); ++i) {
    assert(result.points[i].tag == retained[i]);
    assert(result.points[i].x_m == 1.0F && result.points[i].offset_ns == 900);
    assert(result.points[i].reflectivity == 81);
  }
  const auto stats = filtered.stats();
  assert(stats.lidar_points_received == frame.points.size());
  assert(stats.lidar_points_rejected == rejected.size() + 2);
  assert(stats.lidar_points_rejected_by_tag == rejected.size());
  assert(frame.points.size() == retained.size() + rejected.size() + 2);

  // Generic sources, including non-Livox tags, preserve the previous behavior.
  atlas::preprocessing::SensorPreprocessor unfiltered(output);
  unfiltered.on_lidar_frame(frame);
  assert(sink->last_frame.points.size() == retained.size() + rejected.size());
  assert(unfiltered.stats().lidar_points_rejected_by_tag == 0);
  assert(unfiltered.stats().lidar_points_rejected == 2);

  frame.points = {{.x_m = 2.0F, .tag = 0x02}};
  filtered.on_lidar_frame(frame);
  assert(sink->last_frame.points.empty());
  assert(sink->last_frame.duration_ns == frame.duration_ns);
  assert(filtered.stats().lidar_points_rejected_by_tag == rejected.size() + 1);
  frame.points.clear();
  filtered.on_lidar_frame(frame);
  assert(sink->last_frame.points.empty());
  assert(filtered.stats().lidar_points_rejected_by_tag == rejected.size() + 1);
  output.unsubscribe(subscription);
}

}  // namespace

int main() {
  check_mid360_tag_filter();
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
