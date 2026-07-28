#include <atlas/livox/LivoxSource.hpp>
#include <atlas/sensors/ISensorDataSink.hpp>
#include <atlas/sensors/SensorDataBus.hpp>

#ifdef ATLAS_HAS_UI_STREAM
#include <atlas/ui/UiStreamServer.hpp>
#endif
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

namespace {

std::atomic_bool g_running{true};

void on_signal(int) { g_running.store(false); }

class ConsoleSink final : public atlas::sensors::ISensorDataSink {
 public:
  void on_lidar_frame(const atlas::sensors::LidarFrame& frame) override {
    const auto packet_number = point_packets_.fetch_add(1) + 1;
    if (packet_number % 50 == 0) {
      std::cout << "[MID-360] point packets=" << packet_number
                << " latest_points=" << frame.points.size()
                << " timestamp_ns=" << frame.timestamp.nanoseconds << '\n';
    }
  }

  void on_imu_sample(const atlas::imu::ImuSample& sample) override {
    const auto sample_number = imu_samples_.fetch_add(1) + 1;
    if (sample_number % 100 == 0) {
      std::cout << "[MID-360] IMU samples=" << sample_number
                << " gyro_z_rad_s=" << sample.gyro_z_rad_s
                << " accel_z_m_s2=" << sample.accel_z_m_s2 << '\n';
    }
  }

  void on_sensor_status(const atlas::sensors::SensorStatus& status) override {
    std::cout << "[MID-360] status=" << static_cast<int>(status.state)
              << " handle=" << status.sensor_id;
    if (!status.serial_number.empty()) {
      std::cout << " serial=" << status.serial_number;
    }
    if (!status.ip_address.empty()) {
      std::cout << " ip=" << status.ip_address;
    }
    std::cout << " message=" << status.message << '\n';
  }

 private:
  std::atomic_uint64_t point_packets_{0};
  std::atomic_uint64_t imu_samples_{0};
};

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, on_signal);

  atlas::sensors::SensorDataBus data_bus;
  auto console = std::make_shared<ConsoleSink>();
  const auto subscription = data_bus.subscribe(console);

#ifdef ATLAS_HAS_UI_STREAM
  auto ui_stream = std::make_shared<atlas::ui::UiStreamServer>();
  if (!ui_stream->start()) {
    std::cerr << "[ATLAS UI] stream unavailable: " << ui_stream->last_error() << '\n';
  } else {
    std::cout << "[ATLAS UI] point stream listening on TCP 47777 (_atlas._tcp)\n";
  }
  const auto ui_subscription = data_bus.subscribe(ui_stream);
#endif

  atlas::livox::LivoxSourceConfig config;
  if (argc >= 2) {
    config.config_path = argv[1];
  }

  atlas::livox::LivoxSource source(data_bus, std::move(config));
  if (!source.start()) {
    std::cerr << "[MID-360] start failed: " << source.last_error() << '\n';
    return 1;
  }

  std::cout << "[MID-360] source initialized; waiting for data (Ctrl+C to exit)\n";
  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  source.stop();
  const auto stats = source.stats();
  std::cout << "[MID-360] stopped: point_packets=" << stats.point_packets
            << " points=" << stats.points << " imu_samples=" << stats.imu_samples
            << " rejected_packets=" << stats.rejected_packets << '\n';
#ifdef ATLAS_HAS_UI_STREAM
  data_bus.unsubscribe(ui_subscription);
  ui_stream->stop();
  const auto ui_stats = ui_stream->stats();
  std::cout << "[ATLAS UI] frames_sent=" << ui_stats.frames_sent
            << " points_sent=" << ui_stats.points_sent
            << " dropped_points=" << ui_stats.dropped_points << '\n';
#endif
  data_bus.unsubscribe(subscription);
  return 0;
}
