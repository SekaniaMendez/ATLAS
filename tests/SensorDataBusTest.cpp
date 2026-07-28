#include <atlas/sensors/SensorDataBus.hpp>
#include <cassert>
#include <memory>

namespace {

class CountingSink final : public atlas::sensors::ISensorDataSink {
 public:
  void on_lidar_frame(const atlas::sensors::LidarFrame& frame) override {
    ++lidar_frames;
    last_point_count = frame.points.size();
  }

  void on_imu_sample(const atlas::imu::ImuSample& /*sample*/) override { ++imu_samples; }
  void on_sensor_status(const atlas::sensors::SensorStatus& /*status*/) override { ++statuses; }

  int lidar_frames = 0;
  int imu_samples = 0;
  int statuses = 0;
  size_t last_point_count = 0;
};

}  // namespace

int main() {
  atlas::sensors::SensorDataBus bus;
  auto first = std::make_shared<CountingSink>();
  auto second = std::make_shared<CountingSink>();
  const auto first_id = bus.subscribe(first);
  const auto second_id = bus.subscribe(second);
  assert(first_id != 0);
  assert(second_id != 0);
  assert(first_id != second_id);

  atlas::sensors::LidarFrame frame;
  frame.points.resize(2);
  bus.publish(frame);
  bus.publish(atlas::imu::ImuSample{});
  bus.publish(atlas::sensors::SensorStatus{});

  assert(first->lidar_frames == 1);
  assert(first->imu_samples == 1);
  assert(first->statuses == 1);
  assert(first->last_point_count == 2);
  assert(second->lidar_frames == 1);

  bus.unsubscribe(first_id);
  bus.publish(frame);
  assert(first->lidar_frames == 1);
  assert(second->lidar_frames == 2);
  return 0;
}
