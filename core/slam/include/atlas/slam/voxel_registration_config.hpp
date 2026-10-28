#pragma once

#include <cstddef>
#include <cstdint>

namespace atlas::slam {

/// Scan-to-map point-to-plane registration limits. Distances are in metres;
/// angles are in radians. These defaults target a 20 cm matching-map grid.
struct VoxelRegistrationConfig {
  size_t max_iterations = 15;
  size_t max_query_points = 2500;
  size_t minimum_correspondences = 40;
  size_t nearest_neighbors = 12;
  size_t minimum_plane_neighbors = 6;
  uint32_t neighbor_radius_voxels = 2;
  double max_correspondence_distance_m = 0.60;
  double max_plane_residual_m = 0.20;
  double max_planarity_ratio = 0.08;
  double minimum_plane_spread_m = 0.025;
  double huber_delta_m = 0.05;
  double convergence_translation_m = 0.001;
  double convergence_rotation_rad = 0.0005;
  double max_translation_correction_m = 0.80;
  double max_rotation_correction_rad = 0.35;
  double max_rmse_m = 0.08;
  double minimum_correspondence_ratio = 0.20;
  double max_condition_number = 100000.0;
  /// Eigenvalue threshold of the mean, range-normalized information matrix.
  double minimum_information_eigenvalue = 0.00001;
};

}  // namespace atlas::slam
