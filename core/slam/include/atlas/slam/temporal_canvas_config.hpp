#pragma once

#include <cstddef>
#include <cstdint>

namespace atlas::slam {

struct TemporalCanvasConfig {
  bool enabled = true;
  uint32_t confirmation_scans = 3;
  uint32_t clearing_scans = 3;
  uint64_t evidence_window_ns = 2'000'000'000;
  uint64_t candidate_lifetime_ns = 2'000'000'000;
  size_t maximum_cells = 300'000;
  size_t maximum_rays_per_scan = 2'000;
  size_t maximum_steps_per_ray = 2'048;
  double maximum_ray_length_m = 40.0;
  /// Only rays passing close to a stored sample can contradict it.
  double ray_tolerance_m = 0.03;
  /// Exclude the hit surface and near-sensor region from free-space evidence.
  double surface_margin_m = 0.20;
  double origin_margin_m = 0.60;
  /// Stricter than registration acceptance: freeze temporal mapping on weak poses.
  double maximum_pose_rmse_m = 0.04;
  double maximum_pose_condition_number = 10'000.0;
  /// Normalized RGB distance; color can delay confirmation, never erase geometry.
  double color_difference_threshold = 0.25;
  uint32_t color_confirmation_images = 3;
};

}  // namespace atlas::slam
