#pragma once

#include <atlas/slam/MapBounds.hpp>
#include <cstddef>
#include <optional>

namespace atlas::slam {

struct SparseVoxelMapConfig {
  double voxel_size_m = 0.05;
  std::optional<MapBounds> bounds;
  /// Recent samples in a ring, NOT an unbiased sample or a fused surface.
  /// Positive and finite by design; full captures belong in external storage.
  size_t max_samples_per_voxel = 32;
};

}  // namespace atlas::slam
