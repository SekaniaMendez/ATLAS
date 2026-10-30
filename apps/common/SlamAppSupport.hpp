#pragma once

#include "slam_app_config.hpp"
#include <atlas/slam/VoxelSlamEngine.hpp>
#ifdef ATLAS_HAS_UI_STREAM
#include <atlas/ui/UiStreamServer.hpp>
#endif
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace atlas::apps {

class WarningRelay final : public sensors::ISensorDataSink {
 public:
  explicit WarningRelay(std::shared_ptr<sensors::ISensorDataSink> sink) : sink_(std::move(sink)) {}
  void on_sensor_warning(const sensors::SensorWarning& warning) override {
    sink_->on_sensor_warning(warning);
  }
 private:
  std::shared_ptr<sensors::ISensorDataSink> sink_;
};

struct SlamOutputPaths {
  std::filesystem::path map;
  std::filesystem::path ply;
  std::filesystem::path trajectory;
  std::filesystem::path session_report;

  void validate(bool slam_enabled) const {
    if (!slam_enabled && (!map.empty() || !ply.empty() || !trajectory.empty() || !session_report.empty()))
      throw std::runtime_error("--map, --ply, --trajectory and --session-report require --slam");
    const std::vector<std::filesystem::path> paths{map, ply, trajectory, session_report};
    for (const auto& path : paths) {
      if (!path.empty() && std::filesystem::exists(path))
        throw std::runtime_error("Output already exists; choose a new destination: " + path.string());
    }
    for (size_t i = 0; i < paths.size(); ++i)
      for (size_t j = i + 1; j < paths.size(); ++j)
        if (!paths[i].empty() && paths[i] == paths[j])
          throw std::runtime_error("Each SLAM output needs a different destination");
  }

  void save(const slam::VoxelSlamEngine& engine) const {
    const auto stats = engine.stats();
    if (!session_report.empty()) {
      engine.export_session_report(session_report);
      std::cout << "[ATLAS SLAM] session_report=" << session_report << '\n';
    }
    if (stats.map_voxels != 0) {
      if ((stats.fatal_error || stats.state == slam::TrackingState::Lost) &&
          (!map.empty() || !ply.empty() || !trajectory.empty()))
        std::cerr << "[ATLAS SLAM] partial local map; tracking stopped; exporting accepted scans only\n";
      if (!map.empty()) { engine.save_map(map); std::cout << "[ATLAS SLAM] map=" << map << '\n'; }
      if (!ply.empty()) { engine.export_ply(ply); std::cout << "[ATLAS SLAM] PLY=" << ply << '\n'; }
    } else if (!map.empty() || !ply.empty()) {
      std::cerr << "[ATLAS SLAM] no accepted map points; map and PLY outputs were not created\n";
    }
    if (!trajectory.empty()) {
      engine.export_trajectory(trajectory);
      std::cout << "[ATLAS SLAM] trajectory=" << trajectory << '\n';
      if (stats.scans_accepted == 0)
        std::cerr << "[ATLAS SLAM] trajectory is empty: no accepted poses\n";
    }
  }
};

inline const char* tracking_state_name(slam::TrackingState state) {
  switch (state) {
    case slam::TrackingState::Uninitialized: return "uninitialized";
    case slam::TrackingState::Initializing: return "initializing";
    case slam::TrackingState::Tracking: return "tracking";
    case slam::TrackingState::Lost: return "lost";
  }
  return "unknown";
}

/// Coordinates are local SLAM/world coordinates, never a geographic claim.
inline sensors::LidarFrame map_snapshot(const slam::VoxelSlamEngine& engine, size_t maximum_points) {
  sensors::LidarFrame frame;
  if (const auto pose = engine.latest_pose()) frame.timestamp = pose->timestamp;
  const auto points = engine.map_points(maximum_points);
  frame.points.reserve(points.size());
  for (const auto& point : points) {
    frame.points.push_back({.x_m = static_cast<float>(point.x_m),
                            .y_m = static_cast<float>(point.y_m),
                            .z_m = static_cast<float>(point.z_m),
                            .reflectivity = point.reflectivity, .rgb = point.rgb, .has_color = point.has_color});
  }
  return frame;
}

#ifdef ATLAS_HAS_UI_STREAM
inline void publish_slam_snapshot(const slam::VoxelSlamEngine& engine,
                                  ui::UiStreamServer& server, size_t maximum_points) {
  if (!server.stats().client_connected) return;
  const auto stats = engine.stats();
  ui::wire::SlamStatus status;
  switch (stats.state) {
    case slam::TrackingState::Uninitialized: status.state = ui::wire::SlamTrackingState::Uninitialized; break;
    case slam::TrackingState::Initializing: status.state = ui::wire::SlamTrackingState::Initializing; break;
    case slam::TrackingState::Tracking: status.state = ui::wire::SlamTrackingState::Tracking; break;
    case slam::TrackingState::Lost: status.state = ui::wire::SlamTrackingState::Lost; break;
  }
  status.fatal_error = stats.fatal_error;
  status.scans_accepted = stats.scans_accepted;
  status.scans_rejected = stats.scans_rejected;
  status.message = stats.message;
  if (const auto pose = engine.latest_pose()) status.timestamp_ns = pose->timestamp.nanoseconds;
  server.publish_slam_status(status);
  server.on_lidar_frame(map_snapshot(engine, maximum_points));
}
#endif

inline void print_slam_status(const slam::VoxelSlamEngine& engine) {
  const auto stats = engine.stats();
  std::cout << "[ATLAS SLAM] state=" << tracking_state_name(stats.state)
            << " scans=" << stats.scans_processed << " accepted=" << stats.scans_accepted
            << " rejected=" << stats.scans_rejected
            << " voxels=" << stats.map_voxels << " blocks=" << stats.map_blocks
            << " candidates=" << stats.candidate_voxels << " cleared=" << stats.cleared_voxels
            << " provisional_reference=" << stats.provisional_matching_voxels
            << " contradicted_candidates=" << stats.contradicted_candidate_voxels
            << " expired_candidates=" << stats.expired_candidate_voxels
            << " temporal_skipped=" << stats.temporal_skipped_scans
            << " colored=" << stats.colored_points << " color_conflicts=" << stats.color_conflicts
            << " rmse_m=" << stats.last_rmse_m
            << " correspondences=" << stats.last_correspondences
            << " rejected_events=" << stats.rejected_events;
  if (const auto pose = engine.latest_pose())
    std::cout << " position_m=(" << pose->x_m << ',' << pose->y_m << ',' << pose->z_m << ')';
  std::cout << " message=\"" << stats.message << "\"\n";
}

}  // namespace atlas::apps
