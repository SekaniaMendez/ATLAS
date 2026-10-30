#pragma once

#include <atlas/geometry/live_geometry_config.hpp>
#include <atlas/sensors/LidarFrame.hpp>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace atlas::geometry {

enum class SurfaceKind : uint32_t { Horizontal = 1, Vertical = 2, Inclined = 3, Freeform = 4 };

struct Surface {
  /// Deterministic snapshot label; NOT a tracked identity across captures.
  uint32_t id = 0;
  SurfaceKind kind = SurfaceKind::Inclined;
  std::array<float, 3> origin{};
  /// Three right-handed basis columns, each consecutive XYZ; local -> map.
  std::array<float, 9> axes{};
  float rmse_m = 0;
  float area_m2 = 0;
  uint32_t support_count = 0;
  /// Already in the unchanged, local metric SLAM map (Z-up).
  std::vector<std::array<float, 3>> vertices;
  std::vector<uint32_t> indices;
};

struct GeometrySnapshot {
  uint64_t source_timestamp_ns = 0;
  uint32_t input_point_count = 0;
  bool truncated = false;
  std::vector<Surface> surfaces;
};

/// Pure synchronous extraction; call from a background worker with an owned
/// snapshot. No state is retained, so removed measurements cannot leave cached
/// surfaces behind. Triangles interpolate only inside occupied local UV cells;
/// gaps below that resolution can be bridged. The output is an open preview,
/// not a complete mesh, a semantic classification or a geographical reference.
[[nodiscard]] GeometrySnapshot extract_geometry(
    std::span<const sensors::LidarPoint> points, uint64_t timestamp_ns = 0,
    const LiveGeometryConfig& config = {});

}  // namespace atlas::geometry
