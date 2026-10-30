#include <atlas/storage/Capture.hpp>
#include <filesystem>
#include <limits>
#include <string>

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  constexpr uint64_t origin = 9'007'199'254'740'993ULL;  // Must survive without float rounding.
  const std::string mode = argv[2];
  atlas::storage::CaptureRecorder writer(argv[1], "fixture\ncalibration=unverified\n");
  atlas::imu::ImuSample imu;
  imu.sensor_id = 42;
  imu.timestamp = {origin - 10'000'000, atlas::timesync::ClockSource::Unsynchronized};
  imu.gyro_x_rad_s = 0.1F;
  imu.accel_z_m_s2 = 9.81F;
  writer.on_imu_sample(imu);
  atlas::sensors::LidarFrame frame;
  frame.sensor_id = 42;
  frame.timestamp = {origin, atlas::timesync::ClockSource::Unsynchronized};
  frame.duration_ns = 600'000'000;
  frame.points = {
      {.x_m = 1.0F, .offset_ns = 0},
      {.x_m = 2.0F, .offset_ns = 100'000'000, .reflectivity = 99},
      {.x_m = 3.0F, .offset_ns = 500'000'000},
      {.x_m = 4.0F, .offset_ns = 100'000'000, .tag = 2},
      {.x_m = 0.1F, .offset_ns = 100'000'000},
      {.x_m = std::numeric_limits<float>::quiet_NaN(), .offset_ns = 100'000'000}};
  writer.on_lidar_frame(frame);
  imu.timestamp.nanoseconds = origin + 100'000'000;
  if (mode == "imu_clock") imu.timestamp.source = atlas::timesync::ClockSource::Ptp;
  writer.on_imu_sample(imu);
  imu.timestamp.nanoseconds = origin + 500'000'000;
  writer.on_imu_sample(imu);
  frame.timestamp.nanoseconds = origin + 1'000'000'000;
  if (mode == "lidar_clock") frame.timestamp.source = atlas::timesync::ClockSource::Ptp;
  frame.points = {{.x_m = 5.0F}};
  writer.on_lidar_frame(frame);
  if (mode == "regression") {
    frame.timestamp.nanoseconds = origin + 750'000'000;
    writer.on_lidar_frame(frame);
  }
  frame.sensor_id = 77;
  frame.timestamp = {100, atlas::timesync::ClockSource::Gps};
  writer.on_lidar_frame(frame);
  writer.close();
}
