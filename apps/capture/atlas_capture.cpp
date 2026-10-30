#include <atlas/storage/Capture.hpp>
#include <atlas/storage/PagedVoxelMap.hpp>
#include <atlas/preprocessing/SensorPreprocessor.hpp>
#include <atlas/sensors/SensorDataBus.hpp>
#include "../common/SlamAppSupport.hpp"
#ifdef ATLAS_HAS_UI_STREAM
#include <atlas/ui/UiStreamServer.hpp>
#endif
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
}
int main(int argc, char** argv) {
  try {
    using namespace atlas;
    if (argc < 3) {
      std::cerr << "Usage: atlas_capture synth DIR | inspect DIR | map-demo DIR\n"
                   "       atlas_capture replay DIR [--speed N] [--ui] [--slam]\n"
                   "          [--map NEW_DIRECTORY] [--ply NEW_FILE] [--trajectory NEW_FILE]\n";
      return 1;
    }
    const std::string command = argv[1]; const std::filesystem::path directory = argv[2];
    if (command == "synth") {
      if (argc != 3) throw std::runtime_error("Unexpected synth arguments");
      storage::CaptureRecorder writer(directory, "source=synthetic\ncalibration=none\nclock=synthetic device\n");
      for (uint64_t n = 0; n < 100; ++n) {
        imu::ImuSample imu; imu.sensor_id = 1; imu.timestamp = {n * 5000000, timesync::ClockSource::Unsynchronized};
        imu.accel_z_m_s2 = 9.81F; writer.on_imu_sample(imu);
        sensors::LidarFrame frame; frame.sensor_id = 1; frame.timestamp = imu.timestamp; frame.duration_ns = 1000000;
        for (uint32_t i = 0; i < 100; ++i)
          frame.points.push_back({.x_m = 2.0F, .y_m = (static_cast<float>(i) - 50) * .04F,
                                  .z_m = static_cast<float>(n) * .02F, .offset_ns = i * 10000, .reflectivity = 150});
        writer.on_lidar_frame(frame);
      }
      writer.close(); std::cout << "Created " << writer.stats().written_events << " synthetic events in " << directory << '\n';
      return 0;
    }
    if (command == "map-demo") {
      if (argc != 3) throw std::runtime_error("Unexpected map-demo arguments");
      const auto reservation = storage::PagedVoxelMap::block_payload_reservation(4);
      storage::PagedVoxelMap map(directory, storage::PagedVoxelMap::Mode::Create,
                                 {.voxel_size_m = .05, .max_samples_per_voxel = 4}, reservation * 2);
      for (int pass = 0; pass < 3; ++pass)
        for (int i = -10; i < 10; ++i) {
          if (map.insert({.x_m = i + .01, .z_m = 1}) != slam::VoxelInsertResult::Inserted)
            throw std::runtime_error("Synthetic map insertion rejected");
        }
      map.flush(); const auto stats = map.stats();
      std::cout << "Synthetic map saved: resident_blocks=" << stats.resident_blocks
                << " reserved_payload_bytes=" << stats.reserved_payload_bytes
                << " evictions=" << stats.evictions << '\n'; return 0;
    }
    if (command != "inspect" && command != "replay") throw std::runtime_error("Unknown command");
    double speed = 1.0; bool ui = false, use_slam = false;
    const apps::SlamAppConfig app_config;
    apps::SlamOutputPaths outputs;
    for (int i = 3; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--ui") ui = true;
      else if (option == "--slam") use_slam = true;
      else if ((option == "--map" || option == "--ply" || option == "--trajectory") && i + 1 < argc) {
        const std::filesystem::path path = argv[++i];
        if (option == "--map") outputs.map = path;
        else if (option == "--ply") outputs.ply = path;
        else outputs.trajectory = path;
      }
      else if (option == "--speed" && i + 1 < argc) {
        const std::string value = argv[++i]; size_t used = 0; speed = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(speed) || speed < 0 || (speed > 0 && speed < .001))
          throw std::runtime_error("Speed must be zero (unpaced) or >=0.001");
      } else throw std::runtime_error("Unknown or incomplete option: " + option);
    }
    if (command != "replay" && argc != 3)
      throw std::runtime_error("Replay options require the replay command");
    outputs.validate(use_slam);
    storage::CaptureReader reader(directory);
    std::cout << "Session metadata:\n" << reader.metadata() << '\n';
    sensors::SensorDataBus processed;
    preprocessing::SensorPreprocessorConfig preprocessing_config;
    // The probe writes this source declaration first, including in older captures.
    // Do not interpret arbitrary sensors' tags as MID-360 confidence fields.
    preprocessing_config.filter_mid360_low_confidence_tags =
        reader.metadata().starts_with("source=MID-360 raw bus\n");
    preprocessing::SensorPreprocessor preprocessor(processed, preprocessing_config);
    if (command == "replay")
      std::cout << "[ATLAS PREPROCESSING] MID-360 low-confidence tag filtering "
                << (preprocessing_config.filter_mid360_low_confidence_tags ? "enabled" : "disabled") << '\n';
    std::shared_ptr<slam::VoxelSlamEngine> slam_engine;
    sensors::SensorDataBus::SubscriptionId slam_subscription = 0;
    if (use_slam) {
      slam_engine = std::make_shared<slam::VoxelSlamEngine>();
      slam_subscription = processed.subscribe(slam_engine);
      std::cout << "[ATLAS SLAM] replaying into a local voxel map\n";
    }
#ifdef ATLAS_HAS_UI_STREAM
    std::shared_ptr<ui::UiStreamServer> server;
    sensors::SensorDataBus::SubscriptionId subscription = 0;
    if (ui && command == "replay") {
      ui::UiStreamServerConfig ui_config;
      ui_config.world_frame = use_slam;
      if (use_slam) {
        ui_config.max_points_per_frame = app_config.maximum_snapshot_points;
        ui_config.max_pending_points = app_config.maximum_snapshot_points;
      }
      server = std::make_shared<ui::UiStreamServer>(ui_config);
      if (!server->start()) throw std::runtime_error(server->last_error());
      std::shared_ptr<sensors::ISensorDataSink> ui_sink = server;
      if (use_slam) ui_sink = std::make_shared<apps::WarningRelay>(server);
      subscription = processed.subscribe(ui_sink);
      std::cout << "[ATLAS UI] " << (use_slam ? "SLAM map" : "point")
                << " stream listening on TCP " << ui_config.port << " (_atlas._tcp)\n";
    }
#else
    if (ui) throw std::runtime_error("UI streaming not built");
#endif
    std::signal(SIGINT, stop);
    const auto started = std::chrono::steady_clock::now();
    auto next_status = started;
#ifdef ATLAS_HAS_UI_STREAM
    auto next_map = started;
#endif
    bool failed = false;
    uint64_t count = 0, points = 0;
    while (running) {
      auto event = reader.next(); if (!event) break;
      if (command == "replay" && speed > 0) {
        const long double target_seconds = static_cast<long double>(event->arrival_offset_ns) / 1e9L / speed;
        while (running && std::chrono::duration<long double>(std::chrono::steady_clock::now() - started).count() < target_seconds)
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (!running) break;
      if (const auto* lidar = std::get_if<sensors::LidarFrame>(&event->data)) points += lidar->points.size();
      if (command == "replay") event->dispatch(preprocessor);
      ++count;
      if (slam_engine) {
        // File replay can outrun the sensor callback queue by orders of
        // magnitude. Bound the batch without treating CPU speed as data loss.
        if (count % app_config.replay_batch_events == 0) slam_engine->wait_until_idle();
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_status) {
          apps::print_slam_status(*slam_engine);
          next_status = now + app_config.status_interval;
        }
        if (slam_engine->stats().fatal_error) {
          std::cerr << "[ATLAS SLAM] stopping after fatal error: " << slam_engine->stats().message << '\n';
          failed = true;
          break;
        }
#ifdef ATLAS_HAS_UI_STREAM
        if (server && now >= next_map) {
          apps::publish_slam_snapshot(*slam_engine, *server, app_config.maximum_snapshot_points);
          next_map = now + app_config.map_snapshot_interval;
        }
#endif
      }
    }
    if (slam_engine) {
      processed.unsubscribe(slam_subscription);
      slam_engine->finish();
      apps::print_slam_status(*slam_engine);
      const auto slam_stats = slam_engine->stats();
      failed = failed || slam_stats.fatal_error || slam_stats.scans_accepted == 0 ||
               slam_stats.state == slam::TrackingState::Lost;
      outputs.save(*slam_engine);
#ifdef ATLAS_HAS_UI_STREAM
      if (server && server->stats().client_connected) {
        apps::publish_slam_snapshot(*slam_engine, *server, app_config.maximum_snapshot_points);
        std::this_thread::sleep_for(app_config.final_snapshot_grace);
      }
#endif
    }
#ifdef ATLAS_HAS_UI_STREAM
    if (server) { processed.unsubscribe(subscription); server->stop(); }
#endif
    if (command == "replay") {
      const auto stats = preprocessor.stats();
      std::cout << "[ATLAS PREPROCESSING] lidar_points_received=" << stats.lidar_points_received
                << " lidar_points_rejected=" << stats.lidar_points_rejected
                << " lidar_points_rejected_by_tag=" << stats.lidar_points_rejected_by_tag << '\n';
    }
    std::cout << (running ? "Complete" : "Interrupted") << ": events=" << count << " lidar_points=" << points << '\n';
    return failed ? 1 : running ? 0 : 130;
  } catch (const std::exception& e) { std::cerr << "[ATLAS CAPTURE] " << e.what() << '\n'; return 1; }
}
