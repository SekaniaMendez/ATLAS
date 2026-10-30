#pragma once

#include <atlas/geometry/LiveGeometry.hpp>
#include <atlas/ui/WireProtocol.hpp>

namespace atlas::ui::wire {
// SensorStatus is understood and ignored by older viewers for unknown subtypes.
inline constexpr uint32_t kSensorStatusGeometry = 2;
inline constexpr uint16_t kGeometryVersion = 1;
inline constexpr size_t kGeometryPrefixSize = 32;
inline constexpr size_t kGeometrySurfaceSize = 84;
inline constexpr size_t kGeometryMaximumSurfaces = 32;
inline constexpr size_t kGeometryMaximumVertices = 30'000;
inline constexpr size_t kGeometryMaximumIndices = 90'000;

/// Complete replacement in the ATLAS map frame, including an empty clear.
/// Vertices are in map coordinates; each surface's indices are local to its
/// contiguous vertex range. Axes are column-major local-to-map bases.
[[nodiscard]] std::vector<std::byte> encode_geometry_snapshot(
    const geometry::GeometrySnapshot& snapshot, uint32_t sequence,
    uint64_t source_revision, float processing_ms);
}  // namespace atlas::ui::wire
