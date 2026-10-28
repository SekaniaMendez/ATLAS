#pragma once
#include <atlas/slam/SparseVoxelMap.hpp>
#include <atlas/storage/paged_voxel_map_config.hpp>
#include <filesystem>
#include <memory>

namespace atlas::storage {
struct PagedMapStats {
  size_t resident_blocks = 0;
  size_t resident_block_limit = 0;
  size_t reserved_payload_bytes = 0;
  size_t payload_budget_bytes = 0;
  uint64_t loads = 0, evictions = 0, writes = 0;
};

/// Single-threaded write-back LRU cache. Hot lookup is hashed; cold lookup reads
/// SQLite. Budget reserves worst-case voxel/sample payload per resident block,
/// not total process RSS (SQLite cache, bounded I/O scratch and caller copies are
/// additional). No unbounded in-RAM directory of all archived blocks is kept.
/// Call flush() to checkpoint; failed writes throw and retain the dirty block.
/// This storage layer is independent of raw capture and pose estimation.
class PagedVoxelMap {
 public:
  enum class Mode { Create, Open };
  PagedVoxelMap(const std::filesystem::path& directory, Mode mode,
                slam::SparseVoxelMapConfig config = {},
                size_t payload_budget_bytes = kDefaultPagedMapPayloadBudgetBytes);
  ~PagedVoxelMap();
  PagedVoxelMap(const PagedVoxelMap&) = delete;
  PagedVoxelMap& operator=(const PagedVoxelMap&) = delete;
  [[nodiscard]] const slam::SparseVoxelMapConfig& config() const;
  slam::VoxelInsertResult insert(const slam::MapPoint& point);
  /// Returns a copy; later cache evictions cannot invalidate it.
  [[nodiscard]] std::optional<slam::Voxel> get_voxel(const slam::VoxelKey& key);
  void set_bounds(std::optional<slam::MapBounds> bounds);
  void flush();
  [[nodiscard]] PagedMapStats stats() const;
  [[nodiscard]] static size_t block_payload_reservation(size_t max_samples_per_voxel);
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
