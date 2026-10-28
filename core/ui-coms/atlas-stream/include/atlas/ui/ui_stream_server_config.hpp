#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace atlas::ui {

struct UiStreamServerConfig {
  uint16_t port = 47777;
  std::string service_name = "ATLAS";
  std::chrono::milliseconds frame_interval{50};
  std::chrono::milliseconds send_timeout{1000};
  size_t max_points_per_frame = 10'000;
  size_t max_pending_points = 40'000;
  float maximum_distance_m = 50.0F;
  /// Each incoming frame replaces the pending snapshot. Coordinates are fixed
  /// in the SLAM map frame, not suitable for a live camera overlay.
  bool world_frame = false;
};

inline constexpr size_t kMaximumPendingWarnings = 16;

}  // namespace atlas::ui
