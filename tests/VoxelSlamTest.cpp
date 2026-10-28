#include "../apps/slam_demo/SyntheticRoom.hpp"
#include "StorageTestSupport.hpp"

#include <atlas/slam/VoxelSlamEngine.hpp>
#include <atlas/storage/VoxelMapArchive.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
using atlas::demo::SlamDemoConfig;
using atlas::demo::SyntheticRoom;
using atlas::slam::VoxelSlamConfig;
using atlas::slam::VoxelSlamEngine;

void check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

std::string read_text(const std::filesystem::path& path) {
  std::ifstream file(path);
  check(file.good(), "could not read exported file");
  std::ostringstream contents;
  contents << file.rdbuf();
  return contents.str();
}

void check_exports(const VoxelSlamEngine& engine) {
  TempDirectory temp;
  const auto map_path = temp.path / "map";
  const auto ply_path = temp.path / "map.ply";
  const auto trajectory_path = temp.path / "trajectory.csv";
  engine.save_map(map_path);
  engine.export_ply(ply_path);
  engine.export_trajectory(trajectory_path);
  const auto loaded = atlas::storage::VoxelMapArchive::load(map_path);
  const auto stats = engine.stats();
  check(loaded.voxel_count() == stats.map_voxels &&
            loaded.block_count() == stats.map_blocks,
        "SLAM map snapshot did not preserve occupied voxels and blocks");
  const auto ply = read_text(ply_path);
  const auto csv = read_text(trajectory_path);
  const auto session = read_text(map_path / "slam_session.txt");
  check(session.find("georeferenced=false\n") != std::string::npos &&
            session.find("loop_closure=false\n") != std::string::npos,
        "export does not identify the local prototype's coordinate-frame limits");
  check(ply.find("element vertex " + std::to_string(stats.map_voxels) + "\n") != std::string::npos,
        "PLY vertex count disagrees with SLAM canvas");
  check(ply.find("end_header\n") != std::string::npos, "PLY header is incomplete");
  check(static_cast<size_t>(std::count(csv.begin(), csv.end(), '\n')) ==
            engine.trajectory().size() + 1,
        "CSV does not contain all accepted trajectory poses");
  rejects([&] { engine.save_map(map_path); });
  rejects([&] { engine.export_ply(ply_path); });
  rejects([&] { engine.export_trajectory(trajectory_path); });
  check(read_text(ply_path) == ply && read_text(trajectory_path) == csv,
        "rejected export overwrote prior artifacts");
  check(atlas::storage::VoxelMapArchive::load(map_path).voxel_count() == stats.map_voxels,
        "rejected save changed the map snapshot");
}

void check_metrics(const SyntheticRoom& room, const VoxelSlamEngine& engine,
                   const char* scenario) {
  const auto stats = engine.stats();
  const auto trajectory = engine.trajectory();
  const auto metrics = room.evaluate(trajectory);
  const auto map_metrics = room.evaluate_map(engine.map_points(stats.map_voxels));
  std::cout << scenario << ": poses=" << metrics.pose_count
            << " accepted=" << stats.scans_accepted
            << " rejected=" << stats.scans_rejected
            << " terminal_m=" << metrics.terminal_position_error_m
            << " rmse_m=" << metrics.position_rmse_m
            << " max_angle_rad=" << metrics.maximum_orientation_error_rad
            << " map_rmse_m=" << map_metrics.surface_rmse_m
            << " status=" << stats.message << '\n';
  check(room.within_tolerance(metrics, stats),
        std::string(scenario) + ": estimated trajectory exceeds independent truth tolerance");
  check(stats.map_voxels > 1000 && stats.map_blocks > 0,
        std::string(scenario) + ": accepted trajectory did not build a sparse canvas");
  check(map_metrics.point_count > 0 &&
            map_metrics.surface_rmse_m <= room.config().maximum_map_surface_rmse_m,
        std::string(scenario) + ": fused map disagrees with fixed room surfaces");
  check(!trajectory.empty(), "trajectory empty");
  const auto& final = trajectory.back();
  const auto expected_end = room.scan_start_ns(room.config().scan_count) - 1;
  check(final.timestamp.nanoseconds + room.config().scan_duration_ns >= expected_end,
        std::string(scenario) + ": tracking stopped before the final scan");
  for (size_t i = 1; i < trajectory.size(); ++i)
    check(trajectory[i].timestamp.nanoseconds > trajectory[i - 1].timestamp.nanoseconds,
          "accepted trajectory timestamps must increase");
  const auto latest = engine.latest_pose();
  check(latest.has_value(), "accepted trajectory has no latest pose");
  check(std::abs(latest->x_m - final.x_m) < 1e-9 &&
            latest->timestamp.nanoseconds == final.timestamp.nanoseconds,
        "latest pose disagrees with trajectory");
}

void moving_room_without_imu() {
  SlamDemoConfig simulation;
  simulation.scan_count = 30;
  simulation.points_per_scan = 4200;
  simulation.motion_during_scan = false;
  SyntheticRoom room(simulation);
  VoxelSlamConfig config;
  config.use_imu = false;
  VoxelSlamEngine engine(config);
  room.feed(engine, false);
  engine.finish();
  check_metrics(room, engine, "LiDAR-only translation and yaw");
  check_exports(engine);
}

void moving_room_with_imu_and_deskew() {
  SlamDemoConfig simulation;
  simulation.scan_count = 34;
  simulation.points_per_scan = 4200;
  simulation.yaw_rate_rad_s = 0.70;
  simulation.initial_roll_rad = 0.20;
  SyntheticRoom room(simulation);
  VoxelSlamEngine engine;
  room.feed(engine);
  engine.finish();
  check_metrics(room, engine, "IMU, gravity alignment and intra-scan rotation");
  const auto trajectory = engine.trajectory();
  check(!trajectory.empty(), "gravity-aligned trajectory missing");
  const auto& initial = trajectory.front();
  check(std::abs(initial.qx - std::sin(simulation.initial_roll_rad * 0.5)) < 0.02,
        "stationary IMU initialization did not recover initial roll");
  check(engine.stats().imu_samples >= 50, "initialization did not consume IMU samples");
}

atlas::sensors::LidarFrame wall_frame(size_t scan_index) {
  atlas::sensors::LidarFrame frame;
  frame.sensor_id = 360;
  frame.timestamp = {1'000'000'000 + scan_index * 100'000'000,
                     atlas::timesync::ClockSource::Unsynchronized};
  frame.duration_ns = 100'000'000;
  // A single featureless plane cannot constrain translation parallel to it,
  // nor rotation around its normal. Repetition must not create observability.
  for (int y = -30; y <= 30; ++y)
    for (int z = -20; z <= 20; ++z)
      frame.points.push_back({.x_m = 3.0F,
                              .y_m = static_cast<float>(y) * 0.05F,
                              .z_m = static_cast<float>(z) * 0.05F,
                              .offset_ns = static_cast<uint32_t>(frame.points.size() * 35'000),
                              .reflectivity = 80});
  return frame;
}

void degenerate_wall_does_not_seed_canvas() {
  VoxelSlamConfig config;
  config.use_imu = false;
  VoxelSlamEngine engine(config);
  for (size_t index = 0; index < 4; ++index) {
    engine.on_lidar_frame(wall_frame(index));
    engine.wait_until_idle();
  }
  engine.finish();
  const auto stats = engine.stats();
  check(stats.scans_accepted == 0, "featureless wall was accepted as a fully observable pose");
  check(stats.scans_rejected > 0, "degenerate wall was not reported as rejected");
  check(stats.map_voxels == 0 && engine.map_points(100).empty(),
        "rejected degenerate initialization contaminated the canvas");
}

void invalid_input_cannot_build_a_map() {
  VoxelSlamConfig config;
  config.use_imu = false;
  VoxelSlamEngine engine(config);
  auto malformed = wall_frame(0);
  for (auto& point : malformed.points)
    point.x_m = std::numeric_limits<float>::quiet_NaN();
  engine.on_lidar_frame(malformed);
  engine.wait_until_idle();
  engine.finish();
  const auto stats = engine.stats();
  check(stats.scans_accepted == 0 && stats.map_voxels == 0,
        "non-finite points contaminated the map");
  check(stats.rejected_events + stats.scans_rejected > 0,
        "invalid point stream was silently accepted");
  engine.finish();  // EOF must be safe to call twice.
}

void late_events_and_clock_change() {
  SlamDemoConfig simulation;
  simulation.scan_count = 9;
  simulation.points_per_scan = 3500;
  simulation.velocity_m_s = {0.0, 0.0, 0.0};
  simulation.yaw_rate_rad_s = 0.0;
  SyntheticRoom room(simulation);
  VoxelSlamConfig config;
  config.use_imu = false;
  VoxelSlamEngine engine(config);
  room.feed(engine, false);
  const auto before_late = engine.stats();
  check(before_late.scans_accepted > 0, "timestamp test could not initialize");
  auto late = room.frame(simulation.scan_count - 2);
  engine.on_lidar_frame(late);
  engine.wait_until_idle();
  const auto after_late = engine.stats();
  check(after_late.rejected_events > before_late.rejected_events,
        "out-of-order LiDAR timestamp was not rejected");
  check(!after_late.fatal_error, "small packet reordering incorrectly became a fatal clock rollback");
  check(after_late.map_voxels == before_late.map_voxels &&
            after_late.scans_accepted == before_late.scans_accepted,
        "late LiDAR frame changed the accepted map");
  auto different_clock = room.frame(simulation.scan_count + 1);
  different_clock.timestamp.source = atlas::timesync::ClockSource::Gps;
  engine.on_lidar_frame(different_clock);
  engine.wait_until_idle();
  const auto after_clock_change = engine.stats();
  check(after_clock_change.fatal_error, "clock-domain transition must require explicit reset");
  check(after_clock_change.map_voxels == after_late.map_voxels &&
            after_clock_change.scans_accepted == after_late.scans_accepted,
        "clock-domain transition contaminated the map");
  engine.finish();
  engine.reset();
  check(engine.stats().map_voxels == 0 && !engine.stats().fatal_error &&
            engine.trajectory().empty(),
        "reset did not clear map, trajectory and fatal state");
  // Reuse the stopped/reset engine for one stationary seed, then simulate a
  // device clock restart. It must not mix the two time epochs.
  engine.on_lidar_frame(room.frame(simulation.scan_count - 1));
  engine.wait_until_idle();
  const auto before_rollback = engine.stats();
  check(before_rollback.scans_accepted == 1, "rollback test did not seed its map");
  engine.on_lidar_frame(room.frame(0));
  engine.wait_until_idle();
  const auto after_rollback = engine.stats();
  check(after_rollback.fatal_error &&
            after_rollback.rejected_events > before_rollback.rejected_events,
        "large device clock rollback was not reported as fatal");
  check(after_rollback.map_voxels == before_rollback.map_voxels &&
            after_rollback.scans_accepted == before_rollback.scans_accepted,
        "clock restart mixed observations into the prior canvas");
  engine.finish();
}

void bounded_queue_and_invalid_packet_timing() {
  VoxelSlamConfig impossible_history;
  impossible_history.imu_initialization_ns = std::numeric_limits<uint64_t>::max();
  rejects([&] { VoxelSlamEngine engine(impossible_history); });
  VoxelSlamConfig tiny_queue;
  tiny_queue.use_imu = false;
  tiny_queue.max_queue_bytes = 1;
  VoxelSlamEngine overloaded(tiny_queue);
  overloaded.on_lidar_frame(wall_frame(0));
  overloaded.finish();
  const auto overflow = overloaded.stats();
  check(overflow.fatal_error && overflow.rejected_events > 0,
        "oversized queue event did not fail visibly");
  check(overflow.map_voxels == 0 && overflow.scans_accepted == 0 &&
            overflow.peak_queued_bytes <= tiny_queue.max_queue_bytes,
        "queue overflow violated memory bound or changed the canvas");
  overloaded.finish();
  check(overloaded.stats().rejected_events == overflow.rejected_events &&
            overloaded.stats().scans_processed == overflow.scans_processed,
        "second finish changed finalized statistics");

  VoxelSlamConfig config;
  config.use_imu = false;
  VoxelSlamEngine invalid(config);
  auto too_long = wall_frame(0);
  too_long.duration_ns = static_cast<uint32_t>(config.max_scan_duration_ns + 1);
  invalid.on_lidar_frame(too_long);
  invalid.wait_until_idle();
  auto invalid_offset = wall_frame(1);
  invalid_offset.points.front().offset_ns = invalid_offset.duration_ns + 1;
  invalid.on_lidar_frame(invalid_offset);
  invalid.wait_until_idle();
  invalid.finish();
  const auto rejected = invalid.stats();
  check(rejected.rejected_events >= 2, "invalid duration or point offset was silently accepted");
  check(rejected.scans_accepted == 0 && rejected.map_voxels == 0,
        "invalid packet timing contaminated the canvas");
}

void missing_imu_coverage_rejects_scans() {
  SlamDemoConfig simulation;
  simulation.scan_count = 8;
  simulation.points_per_scan = 3500;
  simulation.velocity_m_s = {0.0, 0.0, 0.0};
  simulation.yaw_rate_rad_s = 0.0;
  SyntheticRoom room(simulation);
  VoxelSlamEngine engine;
  room.feed(engine);
  // Flush the preceding covered scan, then stop supplying IMU while LiDAR
  // continues. Both subsequent scans lack end-of-scan gyro coverage.
  engine.on_lidar_frame(room.frame(8));
  engine.wait_until_idle();
  const auto before_gap = engine.stats();
  check(before_gap.scans_accepted > 0, "IMU gap test could not initialize");
  engine.on_lidar_frame(room.frame(9));
  engine.wait_until_idle();
  engine.finish();
  const auto after_gap = engine.stats();
  check(after_gap.scans_rejected > before_gap.scans_rejected,
        "missing IMU coverage was not reported");
  check(after_gap.scans_accepted == before_gap.scans_accepted &&
            after_gap.map_voxels == before_gap.map_voxels,
        "uncovered scans contaminated the accepted canvas");
}

void tracking_gap_preserves_original_rejection() {
  SlamDemoConfig simulation;
  simulation.scan_count = 6;
  simulation.points_per_scan = 3500;
  simulation.velocity_m_s = {0.0, 0.0, 0.0};
  simulation.yaw_rate_rad_s = 0.0;
  SyntheticRoom room(simulation);
  VoxelSlamConfig config;
  config.use_imu = false;
  VoxelSlamEngine engine(config);
  room.feed(engine, false);
  const auto accepted = engine.stats();
  check(accepted.scans_accepted > 0, "rejection-diagnostic test could not initialize");

  auto outside_map = room.frame(6);
  for (auto& point : outside_map.points) point.x_m += 20.0F;
  engine.on_lidar_frame(outside_map);
  engine.wait_until_idle();
  const auto first = engine.stats();
  check(first.state == atlas::slam::TrackingState::Lost &&
            first.message.find("Insufficient map support:") != std::string::npos &&
            first.message.find("correspondences; require") != std::string::npos,
        "registration did not report its correspondence rejection precisely");

  auto sparse = room.frame(7);
  sparse.points.resize(20);
  engine.on_lidar_frame(sparse);
  engine.wait_until_idle();
  check(engine.stats().message == "Insufficient scan points",
        "secondary rejection did not have its own immediate diagnostic");
  engine.on_lidar_frame(room.frame(10));
  engine.wait_until_idle();
  const auto gap = engine.stats();
  check(gap.message.find("Tracking gap too large") != std::string::npos &&
            gap.message.find("first rejection: " + first.message) != std::string::npos,
        "tracking-gap guard hid the original reason for losing tracking");
  check(gap.scans_accepted == accepted.scans_accepted && gap.map_voxels == accepted.map_voxels,
        "diagnostic rejection sequence changed the accepted trajectory or map");
  engine.finish();
  check(engine.stats().message == gap.message, "finalization hid the tracking-loss diagnostic");
}

}  // namespace

int main() {
  try {
    moving_room_without_imu();
    moving_room_with_imu_and_deskew();
    degenerate_wall_does_not_seed_canvas();
    invalid_input_cannot_build_a_map();
    late_events_and_clock_change();
    missing_imu_coverage_rejects_scans();
    bounded_queue_and_invalid_packet_timing();
    tracking_gap_preserves_original_rejection();
    std::cout << "Voxel SLAM synthetic checks passed.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Voxel SLAM test failed: " << error.what() << '\n';
    return 1;
  }
}
