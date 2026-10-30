#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace atlas::ui {

/// Explicit acquisition provenance; an absent/unknown source must never enable
/// automatic camera capture on a viewer.
enum class StreamSourceKind : uint8_t { Unknown = 0, LiveLidar = 1, Recorded = 2 };

struct UiStreamServerConfig {
  uint16_t port = 47777;
  std::string service_name = "ATLAS";
  StreamSourceKind source_kind = StreamSourceKind::Unknown;
  std::chrono::milliseconds frame_interval{50};
  std::chrono::milliseconds send_timeout{1000};
  size_t max_points_per_frame = 10'000;
  size_t max_pending_points = 40'000;
  float maximum_distance_m = 50.0F;
  /// Each incoming frame replaces the pending snapshot. Coordinates are fixed
  /// in the SLAM map frame, not suitable for a live camera overlay.
  bool world_frame = false;
  /// Provisional measured surfaces, computed on a separate bounded worker.
  bool live_geometry = true;
  std::chrono::milliseconds geometry_interval{1000};
};

inline constexpr size_t kMaximumPendingWarnings = 16;

}  // namespace atlas::ui
