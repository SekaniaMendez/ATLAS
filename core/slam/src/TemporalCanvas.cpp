#include "TemporalCanvas.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace atlas::slam {
namespace {
using Keys = std::unordered_set<VoxelKey, VoxelKeyHash>;
using Vec = std::array<double, 3>;
Vec xyz(const MapPoint& p) { return {p.x_m, p.y_m, p.z_m}; }
bool positive(double x) { return std::isfinite(x) && x > 0; }

// Chromaticity ignores uniform exposure changes. Dark/neutral colors provide
// no useful hue evidence; shadows alone must not block geometric confirmation.
bool usable_color(const MapPoint& p) {
  const auto [lo, hi] = std::minmax_element(p.rgb.begin(), p.rgb.end());
  return p.has_color && p.color_timestamp_ns && *hi >= 40 && *hi < 250 &&
         double(*hi - *lo) / *hi >= 0.20;
}
double color_difference(const MapPoint& a, const MapPoint& b) {
  const double sa = a.rgb[0] + a.rgb[1] + a.rgb[2];
  const double sb = b.rgb[0] + b.rgb[1] + b.rgb[2];
  double d = 0;
  for (size_t i = 0; i < 3; ++i) d += std::pow(a.rgb[i] / sa - b.rgb[i] / sb, 2);
  return std::sqrt(d);
}

template <typename Visit>
void matching_cells(const VoxelKey& key, const SparseVoxelMap& canvas,
                    const SparseVoxelMap& matching, Visit visit) {
  const auto bounds = canvas.voxel_bounds(key);
  const auto first = matching.voxel_key_for({.x_m = bounds.min_x_m, .y_m = bounds.min_y_m,
                                            .z_m = bounds.min_z_m});
  const auto last = matching.voxel_key_for({.x_m = std::nextafter(bounds.max_x_m, bounds.min_x_m),
      .y_m = std::nextafter(bounds.max_y_m, bounds.min_y_m),
      .z_m = std::nextafter(bounds.max_z_m, bounds.min_z_m)});
  if (!first || !last) return;
  for (int64_t z = first->z; z <= last->z; ++z)
    for (int64_t y = first->y; y <= last->y; ++y)
      for (int64_t x = first->x; x <= last->x; ++x)
        visit(VoxelKey{static_cast<int32_t>(x), static_cast<int32_t>(y), static_cast<int32_t>(z)});
}

// Bounded voxel traversal, including negative coordinates. No space behind a
// return is visited. Hitting the work budget yields less evidence, never a clear.
template <typename Visit>
void trace(const RayObservation& ray, const SparseVoxelMap& grid,
           const TemporalCanvasConfig& config, Visit visit) {
  const auto origin = xyz(ray.origin), end = xyz(ray.point);
  Vec direction;
  double length2 = 0;
  for (size_t axis = 0; axis < 3; ++axis) {
    direction[axis] = end[axis] - origin[axis];
    length2 += direction[axis] * direction[axis];
  }
  const double length = std::sqrt(length2);
  if (!positive(length) || length > config.maximum_ray_length_m) return;
  const double stop = length - config.surface_margin_m;
  if (stop <= config.origin_margin_m) return;
  for (auto& value : direction) value /= length;
  MapPoint start{.x_m = origin[0] + direction[0] * config.origin_margin_m,
                 .y_m = origin[1] + direction[1] * config.origin_margin_m,
                 .z_m = origin[2] + direction[2] * config.origin_margin_m};
  const auto start_key = grid.voxel_key_for(start);
  if (!start_key) return;
  std::array<int64_t, 3> cell{start_key->x, start_key->y, start_key->z};
  Vec next, delta;
  std::array<int, 3> step;
  const double size = grid.config().voxel_size_m;
  for (size_t a = 0; a < 3; ++a) {
    step[a] = direction[a] > 0 ? 1 : -1;
    if (std::abs(direction[a]) < 1e-12) {
      next[a] = delta[a] = std::numeric_limits<double>::infinity();
    } else {
      const double boundary = (cell[a] + (step[a] > 0 ? 1 : 0)) * size;
      next[a] = (boundary - origin[a]) / direction[a];
      delta[a] = size / std::abs(direction[a]);
    }
  }
  double distance = config.origin_margin_m;
  for (size_t n = 0; n < config.maximum_steps_per_ray && distance < stop; ++n) {
    const double boundary = *std::min_element(next.begin(), next.end());
    if (boundary > distance + 1e-12)
      visit(VoxelKey{static_cast<int32_t>(cell[0]), static_cast<int32_t>(cell[1]),
                     static_cast<int32_t>(cell[2])}, origin, direction, stop);
    distance = boundary;
    // Crossing an edge/corner must not visit zero-length side cells: touching a
    // boundary is not evidence that the neighboring cell is free.
    for (size_t axis = 0; axis < 3; ++axis) {
      if (next[axis] > boundary + 1e-12) continue;
      cell[axis] += step[axis];
      if (cell[axis] < INT32_MIN || cell[axis] > INT32_MAX) return;
      next[axis] += delta[axis];
    }
  }
}
}  // namespace

TemporalCanvas::TemporalCanvas(TemporalCanvasConfig config, size_t canvas_block_limit,
                                size_t matching_block_limit)
    : config_(config), canvas_block_limit_(canvas_block_limit), matching_block_limit_(matching_block_limit) {
  if (config.confirmation_scans < 2 || config.clearing_scans < 2 ||
      !config.evidence_window_ns || !config.candidate_lifetime_ns || !config.maximum_cells ||
      !config.maximum_rays_per_scan || !config.maximum_steps_per_ray ||
      !positive(config.maximum_ray_length_m) || !positive(config.ray_tolerance_m) ||
      !positive(config.surface_margin_m) || !positive(config.origin_margin_m) ||
      config.maximum_ray_length_m <= config.surface_margin_m + config.origin_margin_m ||
      !positive(config.maximum_pose_rmse_m) || !positive(config.maximum_pose_condition_number) ||
      !canvas_block_limit || !matching_block_limit ||
      !positive(config.color_difference_threshold) || config.color_confirmation_images < 2)
    throw std::invalid_argument("Invalid temporal canvas configuration");
}

void TemporalCanvas::remove_support(const VoxelKey& key, SparseVoxelMap& canvas, SparseVoxelMap& matching) {
  (void)canvas.erase(key);
  // Grids have different resolutions. Remove only samples supported by this fine
  // canvas cell, preserving background samples sharing the coarse matching cell.
  // The cell may straddle matching boundaries when resolutions are not multiples.
  matching_cells(key, canvas, matching, [&](const VoxelKey& match_key) {
    (void)matching.erase_samples(match_key, [&](const MapPoint& p) { return canvas.voxel_key_for(p) == key; });
  });
}

void TemporalCanvas::update(uint64_t timestamp, std::span<const RayObservation> observations,
                            bool reliable_pose, SparseVoxelMap& canvas, SparseVoxelMap& matching) {
  if (!reliable_pose) { ++stats_.skipped_scans; return; }
  if (has_update_ && timestamp <= last_update_ns_)
    throw std::invalid_argument("Temporal canvas requires increasing scan timestamps");
  if (canvas.config().voxel_size_m > matching.config().voxel_size_m)
    throw std::invalid_argument("Temporal canvas resolution must not exceed matching voxel size");
  // Preflight capacity before modifying either geometry or evidence.
  Keys observed;
  for (const auto& ray : observations) {
    const auto key = canvas.voxel_key_for(ray.point);
    if (key && canvas.voxel_key_for(ray.origin) &&
        (!canvas.config().bounds || canvas.config().bounds->contains(ray.point)))
      observed.insert(*key);
  }
  size_t additions = 0;
  for (const auto& key : observed) if (!cells_.contains(key)) ++additions;
  if (additions > config_.maximum_cells - cells_.size())
    throw std::length_error("Temporal canvas cell budget exceeded; save and start a new session");

  // Promotion uses the original measured sample, which can lie on the other side
  // of a matching-grid boundary from today's observation. Budget those exact samples.
  std::unordered_set<BlockKey, BlockKeyHash> canvas_additions, matching_additions;
  for (const auto& key : observed) {
    const auto it = cells_.find(key);
    if (it == cells_.end()) continue;
    const auto& cell = it->second;
    if (cell.stable || cell.observations < config_.confirmation_scans - 1 ||
        timestamp - cell.first_seen_ns > config_.evidence_window_ns) continue;
    const auto block = SparseVoxelMap::address_for(key).block;
    if (!canvas.find_block(block)) canvas_additions.insert(block);
    if (const auto match_key = matching.voxel_key_for(cell.point)) {
      const auto match_block = SparseVoxelMap::address_for(*match_key).block;
      if (!matching.find_block(match_block)) matching_additions.insert(match_block);
    }
  }
  if (canvas.block_count() > canvas_block_limit_ ||
      canvas_additions.size() > canvas_block_limit_ - canvas.block_count() ||
      matching.block_count() > matching_block_limit_ ||
      matching_additions.size() > matching_block_limit_ - matching.block_count())
    throw std::length_error("Temporal canvas map block budget exceeded; save and start a new session");

  Keys free;
  const size_t rays = std::min(observations.size(), config_.maximum_rays_per_scan);
  for (size_t i = 0; i < rays; ++i) {
    // Rotate the bounded selection so a stationary scan pattern does not leave
    // the same cells permanently untested between selected rays.
    const auto index = (static_cast<size_t>(static_cast<long double>(i) * observations.size() / rays) +
                        ray_phase_) % observations.size();
    trace(observations[index], canvas, config_, [&](const VoxelKey& key, const Vec& origin,
                                                   const Vec& direction, double stop) {
      if (observed.contains(key) || free.contains(key)) return;
      const auto found = cells_.find(key);
      if (found == cells_.end()) return;
      const auto point = xyz(found->second.point);
      double along = 0, norm2 = 0;
      for (size_t a = 0; a < 3; ++a) {
        const double d = point[a] - origin[a];
        along += d * direction[a]; norm2 += d * d;
      }
      if (along > config_.origin_margin_m && along < stop &&
          std::max(0.0, norm2 - along * along) <= config_.ray_tolerance_m * config_.ray_tolerance_m)
        free.insert(key);
    });
  }

  for (auto it = cells_.begin(); it != cells_.end();) {
    auto& cell = it->second;
    if (free.contains(it->first)) {
      if (!cell.contradictions || timestamp - cell.first_free_ns > config_.evidence_window_ns) {
        cell.contradictions = 0; cell.first_free_ns = timestamp;
      }
      ++cell.contradictions;
    }
    const bool contradicted = cell.contradictions >= config_.clearing_scans;
    const bool expired = !cell.stable && !cell.reference_only && !observed.contains(it->first) &&
                         timestamp - cell.last_seen_ns > config_.candidate_lifetime_ns;
    if (expired && !contradicted) {
      bool has_reference = false;
      matching_cells(it->first, canvas, matching, [&](const VoxelKey& key) {
        if (const auto* voxel = matching.find(key))
          for (const auto& p : voxel->samples)
            has_reference = has_reference || canvas.voxel_key_for(p) == it->first;
      });
      if (has_reference) {
        // Absence is not free-space evidence. Keep provisional seed support and
        // its visibility history, but never export it as confirmed canvas geometry.
        cell.reference_only = true;
        cell.observations = 0;
        ++stats_.expired_candidates;
        ++it;
        continue;
      }
    }
    if (contradicted || expired) {
      remove_support(it->first, canvas, matching);
      if (contradicted && cell.stable) ++stats_.cleared_cells;
      else if (contradicted) ++stats_.contradicted_candidates;
      else ++stats_.expired_candidates;
      it = cells_.erase(it);
    } else ++it;
  }
  for (const auto& ray : observations) {
    const auto key = canvas.voxel_key_for(ray.point);
    if (!key || !canvas.voxel_key_for(ray.origin) || !observed.erase(*key)) continue;
    auto [it, inserted] = cells_.try_emplace(*key);
    auto& cell = it->second;
    if (inserted || cell.reference_only) {
      cell.point = ray.point; cell.first_seen_ns = timestamp;
      cell.reference_only = false;
      cell.color_hold = false; cell.consistent_colors = 0; cell.last_color_ns = 0;
    }
    if (!cell.stable && cell.observations &&
        timestamp - cell.first_seen_ns > config_.evidence_window_ns) {
      cell.observations = 0; cell.first_seen_ns = timestamp; cell.point = ray.point;
      cell.color_hold = false; cell.consistent_colors = 0; cell.last_color_ns = 0;
    }
    if (!cell.stable && usable_color(ray.point) && ray.point.color_timestamp_ns > cell.last_color_ns) {
      if (usable_color(cell.point) &&
          color_difference(cell.point, ray.point) > config_.color_difference_threshold) {
        cell.observations = 0; cell.first_seen_ns = timestamp; cell.point = ray.point;
        cell.color_hold = true; cell.consistent_colors = 1;
        ++stats_.color_conflicts;
      } else {
        if (!cell.point.has_color) {
          // Keep a measured RGB/XYZ pair rather than painting an older return.
          cell.point = ray.point; cell.observations = 0; cell.first_seen_ns = timestamp;
        }
        if (cell.consistent_colors < config_.color_confirmation_images) ++cell.consistent_colors;
      }
      cell.last_color_ns = ray.point.color_timestamp_ns;
    }
    cell.last_seen_ns = timestamp;
    cell.contradictions = 0;
    if (cell.observations < config_.confirmation_scans) ++cell.observations;
    if (!cell.stable && cell.observations >= config_.confirmation_scans &&
        (!cell.color_hold || cell.consistent_colors >= config_.color_confirmation_images)) {
      (void)canvas.insert(cell.point);
      (void)matching.insert(cell.point);
      cell.stable = true;
      ++stats_.promoted_cells;
    }
    // Stable geometry is not overwritten by a new foreground return in the same cell.
  }
  stats_.stable_cells = 0;
  stats_.provisional_matching_cells = 0;
  for (const auto& [key, cell] : cells_) {
    (void)key;
    stats_.stable_cells += cell.stable;
    stats_.provisional_matching_cells += cell.reference_only;
  }
  stats_.candidate_cells = cells_.size() - stats_.stable_cells - stats_.provisional_matching_cells;
  last_update_ns_ = timestamp; has_update_ = true;
  ray_phase_ = observations.empty() ? 0 : (ray_phase_ + 1) % observations.size();
}

}  // namespace atlas::slam
