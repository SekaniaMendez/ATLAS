#pragma once

#include <cstddef>
#include <cstdint>

namespace atlas::geometry {

/// Bounded preview extraction. The final offline reconstruction is independent.
struct LiveGeometryConfig {
  size_t maximum_input_points = 50'000;
  size_t maximum_surfaces = 32;
  size_t maximum_vertices = 30'000;
  size_t maximum_indices = 90'000;
  size_t maximum_plane_hypotheses = 16;
  size_t ransac_iterations = 96;
  size_t minimum_support_points = 80;
  double distance_threshold_m = 0.025;
  double surface_cell_size_m = 0.16;
  double minimum_surface_area_m2 = 0.12;
  double orientation_tolerance_degrees = 12.0;
  uint64_t random_seed = 7;
};

}  // namespace atlas::geometry
