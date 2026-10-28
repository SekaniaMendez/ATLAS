#include "SyntheticRoom.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
  try {
    if (argc > 2) {
      std::cerr << "Usage: atlas_slam_demo [NEW_OUTPUT_DIRECTORY]\n";
      return 2;
    }
    const auto unique_id = std::chrono::system_clock::now().time_since_epoch().count();
    const std::filesystem::path output = argc == 2
        ? std::filesystem::path(argv[1])
        : std::filesystem::path("output") / ("slam-demo-" + std::to_string(unique_id));
    if (std::filesystem::exists(output))
      throw std::runtime_error("Output directory already exists; choose a new destination");
    if (!std::filesystem::create_directories(output))
      throw std::runtime_error("Could not create a new output directory");
    const atlas::demo::SyntheticRoom room;
    atlas::slam::VoxelSlamEngine engine;
    std::cout << "Synthetic room: full spherical FOV, 0.7 s stationary initialization,\n"
                 "known translation/yaw, per-point motion and deterministic range noise.\n";
    const auto start = std::chrono::steady_clock::now();
    room.feed(engine);
    engine.finish();
    const double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    const auto stats = engine.stats();
    const auto trajectory = engine.trajectory();
    const auto metrics = room.evaluate(trajectory);
    const auto map_metrics = room.evaluate_map(engine.map_points(stats.map_voxels));
    if (stats.map_voxels > 0) {
      engine.save_map(output / "voxel-map");
      engine.export_ply(output / "map.ply");
    } else {
      std::cerr << "No accepted map; retaining trajectory and failure metrics only.\n";
    }
    engine.export_trajectory(output / "trajectory.csv");
    std::ofstream truth(output / "ground-truth.csv");
    truth << std::setprecision(12) << "timestamp_ns,x_m,y_m,z_m,qx,qy,qz,qw\n";
    for (const auto& pose : trajectory) {
      const auto reference = room.pose_at(pose.timestamp.nanoseconds);
      truth << pose.timestamp.nanoseconds;
      for (double value : reference.position_m) truth << ',' << value;
      for (double value : reference.orientation_xyzw) truth << ',' << value;
      truth << '\n';
    }
    truth.close();
    if (!truth) throw std::runtime_error("Could not write ground-truth trajectory");
    const bool passed = room.within_tolerance(metrics, stats) && map_metrics.point_count > 0 &&
        map_metrics.surface_rmse_m <= room.config().maximum_map_surface_rmse_m;
    std::ofstream report(output / "metrics.txt");
    auto print_report = [&](std::ostream& stream) {
      stream << std::fixed << std::setprecision(6)
             << "result=" << (passed ? "PASS" : "FAIL") << '\n'
             << "scans_processed=" << stats.scans_processed << '\n'
             << "scans_accepted=" << stats.scans_accepted << '\n'
             << "scans_rejected=" << stats.scans_rejected << '\n'
             << "map_voxels=" << stats.map_voxels << '\n'
             << "map_blocks=" << stats.map_blocks << '\n'
             << "replay_elapsed_s=" << elapsed_s << '\n'
             << "replay_scans_per_s=" << static_cast<double>(stats.scans_processed) / elapsed_s << '\n'
             << "trajectory_poses=" << metrics.pose_count << '\n'
             << "position_rmse_m=" << metrics.position_rmse_m << '\n'
             << "terminal_position_error_m=" << metrics.terminal_position_error_m << '\n'
             << "maximum_position_error_m=" << metrics.maximum_position_error_m << '\n'
             << "maximum_orientation_error_rad=" << metrics.maximum_orientation_error_rad << '\n'
             << "terminal_orientation_error_rad=" << metrics.terminal_orientation_error_rad << '\n'
             << "map_surface_rmse_m=" << map_metrics.surface_rmse_m << '\n'
             << "maximum_map_surface_error_m=" << map_metrics.maximum_surface_error_m << '\n'
             << "fatal_error=" << stats.fatal_error << '\n'
             << "status=" << stats.message << '\n'
             << "Synthetic validation only: not a hardware accuracy claim.\n";
    };
    print_report(report);
    report.close();
    if (!report) throw std::runtime_error("Could not write demo metrics");
    print_report(std::cout);
    std::cout << "Artifacts: " << std::filesystem::absolute(output) << '\n';
    return passed ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "atlas_slam_demo: " << error.what() << '\n';
    return 1;
  }
}
