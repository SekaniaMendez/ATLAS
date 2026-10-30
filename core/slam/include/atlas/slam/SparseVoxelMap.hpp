#pragma once

#include <atlas/slam/sparse_voxel_map_config.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

namespace atlas::storage { class VoxelMapArchive; }

namespace atlas::slam {

/// Position in the fixed project-local metric frame. GNSS/project transforms
/// and sensor poses are supplied by the caller, outside this container.
struct MapPoint {
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  uint8_t reflectivity = 0;
  std::array<uint8_t, 3> rgb{0, 0, 0};
  bool has_color = false;
  uint64_t color_timestamp_ns = 0; // runtime evidence only; archive retains RGB
};

struct VoxelKey {
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;
  [[nodiscard]] bool operator==(const VoxelKey&) const = default;
};

struct BlockKey {
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;
  [[nodiscard]] bool operator==(const BlockKey&) const = default;
};

struct VoxelKeyHash {
  [[nodiscard]] size_t operator()(const VoxelKey& key) const noexcept;
};
struct BlockKeyHash {
  [[nodiscard]] size_t operator()(const BlockKey& key) const noexcept;
};

struct Voxel {
  uint64_t hit_count = 0;
  /// Ring storage order is not chronological after the first replacement.
  std::vector<MapPoint> samples;
};

inline constexpr int32_t kVoxelBlockSide = 8;
inline constexpr size_t kVoxelsPerBlock = 512;

struct VoxelAddress {
  BlockKey block;
  /// x + 8*y + 64*z, with local coordinates in [0, 8).
  uint16_t local_index = 0;
};

struct VoxelBlock {
  std::array<Voxel, kVoxelsPerBlock> voxels;
  size_t occupied_voxel_count = 0;
  uint64_t revision = 0;
  bool change_pending = false;
};

enum class VoxelInsertResult { Inserted, InvalidPoint, OutsideBounds, CoordinateOutOfRange };

struct VoxelNeighbor {
  VoxelKey key;
  const Voxel* voxel = nullptr;
};

struct BlockChange {
  BlockKey key;
  uint64_t revision = 0;
  bool removed = false;
};

/// Single-consumer notification batch. No geometry is copied. On reset,
/// discard previously cached blocks, then process blocks in this batch.
struct MapChanges {
  uint64_t revision = 0;
  bool reset = false;
  bool bounds_changed = false;
  std::vector<BlockChange> blocks;
};

struct VoxelMapMemoryStats {
  size_t block_count = 0;
  size_t occupied_voxel_count = 0;
  size_t retained_sample_count = 0;
  size_t allocated_voxel_slots = 0;
  size_t block_storage_bytes = 0;
  size_t sample_capacity_bytes = 0;
  /// sizeof(map) + block payloads + sample capacities + hash bucket estimate
  /// + pending-change capacity. Excludes allocator and hash-node overhead;
  /// this is NOT resident process memory or a hard memory budget.
  size_t estimated_bytes = 0;
};

/// Globally sparse, locally dense canvas. Only observed blocks allocate voxel
/// metadata; unobserved cells inside those blocks have no sample allocation.
/// Missing cells are unknown, not confirmed free space.
///
/// Single-threaded: externally synchronize all reads/writes or copy a snapshot.
/// Voxel/block pointers survive insertions and hash rehashing, but not erasure of
/// their block, clear() or destruction. Samples can change on mutation. No sensor integration,
/// surface fitting, persistence, eviction or GNSS projection is performed here.
class SparseVoxelMap {
 public:
  using Storage = std::unordered_map<BlockKey, VoxelBlock, BlockKeyHash>;
  explicit SparseVoxelMap(SparseVoxelMapConfig config = {});

  [[nodiscard]] const SparseVoxelMapConfig& config() const noexcept { return config_; }
  /// Update the capture boundary only. Existing geometry and keys are preserved,
  /// even outside a shrunken box. nullopt removes the capture boundary.
  void set_bounds(std::optional<MapBounds> bounds);

  /// Coordinate lookup ignores capture bounds. Rejects NaN/Inf and int32 overflow.
  [[nodiscard]] std::optional<VoxelKey> voxel_key_for(const MapPoint& point) const noexcept;
  [[nodiscard]] static VoxelAddress address_for(const VoxelKey& key) noexcept;
  /// Inverse of address_for; rejects invalid local indices or block coordinates
  /// whose reconstructed voxel would lie outside the int32 grid.
  [[nodiscard]] static std::optional<VoxelKey> key_for_address(const VoxelAddress& address) noexcept;
  [[nodiscard]] MapBounds voxel_bounds(const VoxelKey& key) const noexcept;
  [[nodiscard]] MapPoint voxel_center(const VoxelKey& key) const noexcept;

  VoxelInsertResult insert(const MapPoint& point);
  /// Return a cell to unknown, preserving the grid and all other cells.
  /// Reclaims empty blocks and emits a removal notification. Missing cells are a no-op.
  bool erase(const VoxelKey& key);
  /// Remove only matching samples. A partial removal restarts this cell's
  /// observation count from its surviving samples (historical attribution is unknown).
  size_t erase_samples(const VoxelKey& key, const std::function<bool(const MapPoint&)>& predicate);
  [[nodiscard]] const Voxel* find(const VoxelKey& key) const noexcept;
  [[nodiscard]] const Voxel* find_at(const MapPoint& point) const noexcept;
  [[nodiscard]] const VoxelBlock* find_block(const BlockKey& key) const noexcept;
  [[nodiscard]] const Storage& blocks() const noexcept { return blocks_; }
  /// Occupied cells in a cube, including the center, in z/y/x order.
  /// radius=1 probes at most 27 cells. Radius must be <=8 to bound local work.
  /// This is a voxel neighborhood, not an exact metric nearest-neighbor query.
  [[nodiscard]] std::vector<VoxelNeighbor> neighbors(const VoxelKey& center,
                                                    uint32_t radius = 1) const;

  [[nodiscard]] size_t block_count() const noexcept { return blocks_.size(); }
  [[nodiscard]] size_t voxel_count() const noexcept { return voxel_count_; }
  /// Sum of observation counts still represented by the retained cells, not lifetime input.
  [[nodiscard]] uint64_t accepted_hit_count() const noexcept { return accepted_hit_count_; }
  [[nodiscard]] uint64_t revision() const noexcept { return revision_; }
  /// Drains deduplicated notifications. Consumers read blocks under the same
  /// synchronization as the map; asynchronous workers need their own snapshot.
  [[nodiscard]] MapChanges take_changes();
  /// O(allocated voxel slots); intended for diagnostics, not per-point calls.
  [[nodiscard]] VoxelMapMemoryStats memory_stats() const noexcept;
  void reserve_blocks(size_t block_capacity);
  /// Removes all geometry, preserves configuration, emits reset notification.
  void clear();

 private:
  friend class atlas::storage::VoxelMapArchive;
  void require_revision_capacity() const;
  void replace_samples(const VoxelKey& key, std::vector<MapPoint> samples);
  SparseVoxelMapConfig config_;
  Storage blocks_;
  std::vector<BlockKey> pending_blocks_;
  std::unordered_map<BlockKey, uint64_t, BlockKeyHash> removed_blocks_;
  size_t voxel_count_ = 0;
  uint64_t accepted_hit_count_ = 0;
  uint64_t revision_ = 0;
  bool reset_pending_ = true;
  bool bounds_changed_pending_ = true;
};

}  // namespace atlas::slam
