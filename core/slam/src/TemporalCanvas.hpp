#pragma once

#include <atlas/slam/SparseVoxelMap.hpp>
#include <atlas/slam/temporal_canvas_config.hpp>
#include <span>
#include <unordered_map>

namespace atlas::slam {

/// Both positions are in the fixed map frame. The origin is the sensor position
/// at this return's acquisition time, not a shared end-of-scan approximation.
struct RayObservation {
  MapPoint origin;
  MapPoint point;
};

struct TemporalCanvasStats {
  size_t candidate_cells = 0;
  size_t stable_cells = 0;
  size_t provisional_matching_cells = 0;
  uint64_t promoted_cells = 0;
  uint64_t cleared_cells = 0;
  uint64_t contradicted_candidates = 0;
  uint64_t expired_candidates = 0;
  uint64_t skipped_scans = 0;
  uint64_t color_conflicts = 0;
};

/// Single SLAM-worker owner. Candidates never overwrite confirmed geometry.
/// Evidence is tied to distinct accepted scans and original measured samples;
/// no object identity or semantics are inferred. MapPoint retains optional RGB.
class TemporalCanvas {
 public:
  explicit TemporalCanvas(TemporalCanvasConfig config, size_t canvas_block_limit = SIZE_MAX,
                          size_t matching_block_limit = SIZE_MAX);
  void update(uint64_t timestamp_ns, std::span<const RayObservation> observations,
              bool reliable_pose, SparseVoxelMap& canvas, SparseVoxelMap& matching);
  [[nodiscard]] TemporalCanvasStats stats() const { return stats_; }

 private:
  struct Cell {
    MapPoint point;
    uint64_t first_seen_ns = 0, last_seen_ns = 0, first_free_ns = 0;
    uint32_t observations = 0, contradictions = 0;
    bool stable = false;
    bool reference_only = false;
    uint64_t last_color_ns = 0;
    uint32_t consistent_colors = 0;
    bool color_hold = false;
  };
  void remove_support(const VoxelKey& key, SparseVoxelMap& canvas, SparseVoxelMap& matching);
  TemporalCanvasConfig config_;
  size_t canvas_block_limit_, matching_block_limit_;
  std::unordered_map<VoxelKey, Cell, VoxelKeyHash> cells_;
  uint64_t last_update_ns_ = 0;
  bool has_update_ = false;
  size_t ray_phase_ = 0;
  TemporalCanvasStats stats_;
};

}  // namespace atlas::slam
