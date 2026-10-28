#include <atlas/livox/LivoxSource.hpp>
#include <atlas/preprocessing/SensorPreprocessor.hpp>
#include <atlas/sensors/ISensorDataSink.hpp>
#include <atlas/sensors/SensorDataBus.hpp>
#include <atlas/storage/Capture.hpp>
#include "../common/SlamAppSupport.hpp"

#ifdef ATLAS_HAS_UI_STREAM
#include <atlas/ui/UiStreamServer.hpp>
#endif
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <memory>
#include <fstream>
#include <sstream>
#include <thread>

namespace {

std::atomic_bool g_running{true};

void on_signal(int) { g_running.store(false); }

class ConsoleSink final : public atlas::sensors::ISensorDataSink {
 public:
  explicit ConsoleSink(bool report_packets) : report_packets_(report_packets) {}
  void on_lidar_frame(const atlas::sensors::LidarFrame& frame) override {
    if (!report_packets_) return;
    const auto packet_number = point_packets_.fetch_add(1) + 1;
    if (packet_number % 50 == 0) {
      std::cout << "[MID-360] point packets=" << packet_number
                << " latest_points=" << frame.points.size()
                << " timestamp_ns=" << frame.timestamp.nanoseconds << '\n';
    }
  }

  void on_imu_sample(const atlas::imu::ImuSample& sample) override {
    if (!report_packets_) return;
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

  void on_sensor_warning(const atlas::sensors::SensorWarning& warning) override {
    std::cout << "[ATLAS WARNING] code=" << static_cast<int>(warning.code)
              << " measured=" << warning.measured_value << " threshold=" << warning.threshold
              << '\n';
  }

 private:
  bool report_packets_;
  std::atomic_uint64_t point_packets_{0};
  std::atomic_uint64_t imu_samples_{0};
};

}  // namespace

int run_probe(int argc, char** argv) {
  std::signal(SIGINT, on_signal);

  atlas::livox::LivoxSourceConfig config;
  const atlas::apps::SlamAppConfig app_config;
  atlas::apps::SlamOutputPaths outputs;
  std::string record_directory, metadata_path;
  bool has_config = false, use_slam = false;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if ((argument == "--record" || argument == "--metadata") && i + 1 < argc) {
      (argument == "--record" ? record_directory : metadata_path) = argv[++i];
    } else if (argument == "--slam") {
      use_slam = true;
    } else if ((argument == "--map" || argument == "--ply" || argument == "--trajectory") && i + 1 < argc) {
      const std::filesystem::path path = argv[++i];
      if (argument == "--map") outputs.map = path;
      else if (argument == "--ply") outputs.ply = path;
      else outputs.trajectory = path;
    } else if (argument == "--help") {
      std::cout << "atlas_livox_probe [config.json] [--record NEW_DIRECTORY] [--metadata CALIBRATION_FILE]\n"
                   "                  [--slam] [--map NEW_DIRECTORY] [--ply NEW_FILE] [--trajectory NEW_FILE]\n"
                   "SLAM starts in a local metric frame. Keep the sensor still during IMU initialization.\n"
                   "Map/PLY/trajectory outputs require --slam and are written on Ctrl+C.\n";
      return 0;
    } else if (!argument.starts_with("--") && !has_config) {
      config.config_path = argument; has_config = true;
    } else throw std::runtime_error("Unknown/incomplete option: " + argument);
  }
  if (!metadata_path.empty() && record_directory.empty()) throw std::runtime_error("--metadata requires --record");
  outputs.validate(use_slam);

  atlas::sensors::SensorDataBus raw_data_bus;
  atlas::sensors::SensorDataBus processed_data_bus;
  std::shared_ptr<atlas::storage::CaptureRecorder> recorder;
  atlas::sensors::SensorDataBus::SubscriptionId recording_subscription = 0;
  if (!record_directory.empty()) {
    const auto read_text = [](const std::string& path) {
      std::ifstream file(path, std::ios::binary);
      if (!file) throw std::runtime_error("Cannot read capture metadata: " + path);
      std::ostringstream text; text << file.rdbuf();
      if (file.bad()) throw std::runtime_error("Metadata read failed: " + path);
      return text.str();
    };
    const auto utc_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    std::string metadata = "source=MID-360 raw bus\ncreated_unix_ns=" + std::to_string(utc_ns) +
        "\nclock_policy=preserve source plus Mac steady arrival offsets\ncamera_images=record separately on iPhone\n[livox_config]\n" +
        (config.config_path.empty() ? "SDK default; no configuration file supplied" : read_text(config.config_path)) +
        "\n[external_calibration]\n" +
        (metadata_path.empty() ? "not supplied; no verified transform assumed" : read_text(metadata_path));
    recorder = std::make_shared<atlas::storage::CaptureRecorder>(record_directory, metadata);
    recording_subscription = raw_data_bus.subscribe(recorder);
    std::cout << "[ATLAS CAPTURE] recording raw LiDAR/IMU to " << record_directory << '\n';
  }
  auto preprocessor =
      std::make_shared<atlas::preprocessing::SensorPreprocessor>(processed_data_bus);
  const auto preprocessing_subscription = raw_data_bus.subscribe(preprocessor);

  auto console = std::make_shared<ConsoleSink>(!use_slam);
  const auto subscription = processed_data_bus.subscribe(console);
  std::shared_ptr<atlas::slam::VoxelSlamEngine> slam;
  atlas::sensors::SensorDataBus::SubscriptionId slam_subscription = 0;
  if (use_slam) {
    slam = std::make_shared<atlas::slam::VoxelSlamEngine>();
    slam_subscription = processed_data_bus.subscribe(slam);
    std::cout << "[ATLAS SLAM] local voxel map enabled; keep MID-360 stationary for initialization\n";
  }

#ifdef ATLAS_HAS_UI_STREAM
  atlas::ui::UiStreamServerConfig ui_config;
  ui_config.world_frame = use_slam;
  if (use_slam) {
    ui_config.max_points_per_frame = app_config.maximum_snapshot_points;
    ui_config.max_pending_points = app_config.maximum_snapshot_points;
  }
  auto ui_stream = std::make_shared<atlas::ui::UiStreamServer>(ui_config);
  if (!ui_stream->start()) {
    std::cerr << "[ATLAS UI] stream unavailable: " << ui_stream->last_error() << '\n';
  } else {
    std::cout << "[ATLAS UI] " << (use_slam ? "SLAM map" : "point")
              << " stream listening on TCP " << ui_config.port << " (_atlas._tcp)\n";
  }
  std::shared_ptr<atlas::sensors::ISensorDataSink> ui_sink = ui_stream;
  if (use_slam) ui_sink = std::make_shared<atlas::apps::WarningRelay>(ui_stream);
  const auto ui_subscription = processed_data_bus.subscribe(ui_sink);
#endif

  atlas::livox::LivoxSource source(raw_data_bus, std::move(config));
  if (!source.start()) {
    std::cerr << "[MID-360] start failed: " << source.last_error() << '\n';
    return 1;
  }

  std::cout << "[MID-360] source initialized; waiting for data (Ctrl+C to exit)\n";
  auto next_status = std::chrono::steady_clock::now();
#ifdef ATLAS_HAS_UI_STREAM
  auto next_map = next_status;
#endif
  bool failed = false;
  while (g_running.load()) {
    if (recorder && recorder->stats().failed) {
      std::cerr << "[ATLAS CAPTURE] " << recorder->stats().error << '\n';
      failed = true;
      break;
    }
    const auto now = std::chrono::steady_clock::now();
    if (slam) {
      if (now >= next_status) {
        atlas::apps::print_slam_status(*slam);
        next_status = now + app_config.status_interval;
      }
      if (slam->stats().fatal_error) {
        std::cerr << "[ATLAS SLAM] stopping after fatal error: " << slam->stats().message << '\n';
        failed = true;
        break;
      }
#ifdef ATLAS_HAS_UI_STREAM
      if (now >= next_map) {
        atlas::apps::publish_slam_snapshot(*slam, *ui_stream, app_config.maximum_snapshot_points);
        next_map = now + app_config.map_snapshot_interval;
      }
#endif
    }
    std::this_thread::sleep_for(app_config.live_poll_interval);
  }

  source.stop();
  if (slam) {
    processed_data_bus.unsubscribe(slam_subscription);
    slam->finish();
    atlas::apps::print_slam_status(*slam);
    const auto slam_stats = slam->stats();
    failed = failed || slam_stats.fatal_error || slam_stats.scans_accepted == 0 ||
             slam_stats.state == atlas::slam::TrackingState::Lost;
#ifdef ATLAS_HAS_UI_STREAM
    atlas::apps::publish_slam_snapshot(*slam, *ui_stream, app_config.maximum_snapshot_points);
#endif
    outputs.save(*slam);
#ifdef ATLAS_HAS_UI_STREAM
    if (ui_stream->stats().client_connected)
      std::this_thread::sleep_for(app_config.final_snapshot_grace);
#endif
  }
  if (recorder) {
    raw_data_bus.unsubscribe(recording_subscription);
    recorder->close();
    std::cout << "[ATLAS CAPTURE] complete: events=" << recorder->stats().written_events << '\n';
  }
  const auto stats = source.stats();
  const auto preprocessing_stats = preprocessor->stats();
  std::cout << "[MID-360] stopped: point_packets=" << stats.point_packets
            << " points=" << stats.points << " imu_samples=" << stats.imu_samples
            << " rejected_packets=" << stats.rejected_packets << '\n';
  std::cout << "[ATLAS PREPROCESSING] lidar_points_received="
            << preprocessing_stats.lidar_points_received
            << " lidar_points_rejected=" << preprocessing_stats.lidar_points_rejected
            << " imu_samples=" << preprocessing_stats.imu_samples_received
            << " warnings=" << preprocessing_stats.warnings_published << '\n';
#ifdef ATLAS_HAS_UI_STREAM
  processed_data_bus.unsubscribe(ui_subscription);
  ui_stream->stop();
  const auto ui_stats = ui_stream->stats();
  std::cout << "[ATLAS UI] frames_sent=" << ui_stats.frames_sent
            << " points_sent=" << ui_stats.points_sent
            << " dropped_points=" << ui_stats.dropped_points
            << " warnings_sent=" << ui_stats.warnings_sent << '\n';
  std::cout << "[ATLAS UI] slam_statuses_sent=" << ui_stats.slam_statuses_sent << '\n';
#endif
  processed_data_bus.unsubscribe(subscription);
  raw_data_bus.unsubscribe(preprocessing_subscription);
  return failed ? 1 : 0;
}

int main(int argc, char** argv) {
  try { return run_probe(argc, argv); }
  catch (const std::exception& error) { std::cerr << "[ATLAS] " << error.what() << '\n'; return 1; }
}
