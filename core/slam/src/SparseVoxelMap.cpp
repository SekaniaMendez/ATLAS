#include <atlas/slam/SparseVoxelMap.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace atlas::slam {
namespace {

uint64_t mix(uint64_t value) noexcept {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

size_t hash_coordinates(int32_t x, int32_t y, int32_t z) noexcept {
  // Unsigned conversions have defined behavior for negative coordinates.
  return static_cast<size_t>(mix(mix(static_cast<uint32_t>(x)) ^
      mix(static_cast<uint32_t>(y) + 0x517cc1b727220a95ULL) ^
      mix(static_cast<uint32_t>(z) + 0x6eed0e9da4d94a4fULL)));
}

bool is_finite(const MapPoint& point) noexcept {
  return std::isfinite(point.x_m) && std::isfinite(point.y_m) && std::isfinite(point.z_m);
}

std::optional<int32_t> cell_index(double coordinate, double cell_size) noexcept {
  const double cell = std::floor(coordinate / cell_size);
  if (!std::isfinite(cell) || cell < std::numeric_limits<int32_t>::min() ||
      cell > std::numeric_limits<int32_t>::max()) return std::nullopt;
  return static_cast<int32_t>(cell);
}

int32_t block_index(int32_t cell) noexcept {
  // C++ integer division truncates toward zero; the grid needs floor division.
  return cell / kVoxelBlockSide - (cell % kVoxelBlockSide < 0 ? 1 : 0);
}

void validate_bounds(const std::optional<MapBounds>& bounds) {
  if (bounds && !bounds->is_valid())
    throw std::invalid_argument("SparseVoxelMap bounds must be a finite non-empty box");
}

}  // namespace

size_t VoxelKeyHash::operator()(const VoxelKey& key) const noexcept {
  return hash_coordinates(key.x, key.y, key.z);
}

size_t BlockKeyHash::operator()(const BlockKey& key) const noexcept {
  return hash_coordinates(key.x, key.y, key.z);
}

bool MapBounds::is_valid() const noexcept {
  return std::isfinite(min_x_m) && std::isfinite(min_y_m) && std::isfinite(min_z_m) &&
         std::isfinite(max_x_m) && std::isfinite(max_y_m) && std::isfinite(max_z_m) &&
         min_x_m < max_x_m && min_y_m < max_y_m && min_z_m < max_z_m &&
         std::isfinite(max_x_m - min_x_m) && std::isfinite(max_y_m - min_y_m) &&
         std::isfinite(max_z_m - min_z_m);
}

bool MapBounds::contains(const MapPoint& point) const noexcept {
  return point.x_m >= min_x_m && point.x_m < max_x_m && point.y_m >= min_y_m &&
         point.y_m < max_y_m && point.z_m >= min_z_m && point.z_m < max_z_m;
}

std::array<double, 3> MapBounds::dimensions_m() const noexcept {
  return {max_x_m - min_x_m, max_y_m - min_y_m, max_z_m - min_z_m};
}

SparseVoxelMap::SparseVoxelMap(SparseVoxelMapConfig config) : config_(std::move(config)) {
  if (!std::isfinite(config_.voxel_size_m) ||
      config_.voxel_size_m < std::numeric_limits<double>::min() ||
      !std::isfinite(config_.voxel_size_m * 2147483648.0))
    throw std::invalid_argument("SparseVoxelMap voxel_size_m must be normal, positive and give a finite grid");
  if (config_.max_samples_per_voxel == 0 ||
      config_.max_samples_per_voxel > std::vector<MapPoint>{}.max_size())
    throw std::invalid_argument("SparseVoxelMap requires a positive bounded sample capacity");
  validate_bounds(config_.bounds);
}

void SparseVoxelMap::require_revision_capacity() const {
  if (revision_ == std::numeric_limits<uint64_t>::max())
    throw std::overflow_error("SparseVoxelMap revision exhausted");
}

void SparseVoxelMap::set_bounds(std::optional<MapBounds> bounds) {
  validate_bounds(bounds);
  require_revision_capacity();
  config_.bounds = bounds;
  ++revision_;
  bounds_changed_pending_ = true;
}

std::optional<VoxelKey> SparseVoxelMap::voxel_key_for(const MapPoint& point) const noexcept {
  if (!is_finite(point)) return std::nullopt;
  const auto x = cell_index(point.x_m, config_.voxel_size_m);
  const auto y = cell_index(point.y_m, config_.voxel_size_m);
  const auto z = cell_index(point.z_m, config_.voxel_size_m);
  if (!x || !y || !z) return std::nullopt;
  return VoxelKey{*x, *y, *z};
}

VoxelAddress SparseVoxelMap::address_for(const VoxelKey& key) noexcept {
  const BlockKey block{block_index(key.x), block_index(key.y), block_index(key.z)};
  const auto x = key.x - block.x * kVoxelBlockSide;
  const auto y = key.y - block.y * kVoxelBlockSide;
  const auto z = key.z - block.z * kVoxelBlockSide;
  return {block, static_cast<uint16_t>(x + 8 * y + 64 * z)};
}

std::optional<VoxelKey> SparseVoxelMap::key_for_address(const VoxelAddress& address) noexcept {
  if (address.local_index >= kVoxelsPerBlock) return std::nullopt;
  const int64_t x = static_cast<int64_t>(address.block.x) * 8 + address.local_index % 8;
  const int64_t y = static_cast<int64_t>(address.block.y) * 8 + (address.local_index / 8) % 8;
  const int64_t z = static_cast<int64_t>(address.block.z) * 8 + address.local_index / 64;
  if (x < INT32_MIN || x > INT32_MAX || y < INT32_MIN || y > INT32_MAX ||
      z < INT32_MIN || z > INT32_MAX) return std::nullopt;
  return VoxelKey{static_cast<int32_t>(x), static_cast<int32_t>(y), static_cast<int32_t>(z)};
}

MapBounds SparseVoxelMap::voxel_bounds(const VoxelKey& key) const noexcept {
  const double s = config_.voxel_size_m;
  return {key.x * s, key.y * s, key.z * s,
          (static_cast<double>(key.x) + 1.0) * s,
          (static_cast<double>(key.y) + 1.0) * s,
          (static_cast<double>(key.z) + 1.0) * s};
}

MapPoint SparseVoxelMap::voxel_center(const VoxelKey& key) const noexcept {
  const double s = config_.voxel_size_m;
  return {.x_m = (static_cast<double>(key.x) + 0.5) * s,
          .y_m = (static_cast<double>(key.y) + 0.5) * s,
          .z_m = (static_cast<double>(key.z) + 0.5) * s};
}

VoxelInsertResult SparseVoxelMap::insert(const MapPoint& point) {
  if (!is_finite(point)) return VoxelInsertResult::InvalidPoint;
  if (config_.bounds && !config_.bounds->contains(point)) return VoxelInsertResult::OutsideBounds;
  const auto key = voxel_key_for(point);
  if (!key) return VoxelInsertResult::CoordinateOutOfRange;
  require_revision_capacity();

  const auto address = address_for(*key);
  auto [iterator, new_block] = blocks_.try_emplace(address.block);
  auto& block = iterator->second;
  auto& voxel = block.voxels[address.local_index];
  // Queue before changing geometry, then roll back if sample allocation fails.
  // Existing blocks preserve content and notifications on allocation failure.
  const bool queue_change = !block.change_pending;
  bool queued = false;
  try {
    if (queue_change) {
      pending_blocks_.push_back(address.block);
      queued = true;
    }
    if (voxel.samples.size() < config_.max_samples_per_voxel) {
      voxel.samples.push_back(point);
    } else {
      voxel.samples[voxel.hit_count % config_.max_samples_per_voxel] = point;
    }
  } catch (...) {
    if (queued) pending_blocks_.pop_back();
    if (new_block) blocks_.erase(iterator);
    throw;
  }
  if (voxel.hit_count == 0) {
    ++voxel_count_;
    ++block.occupied_voxel_count;
  }
  ++voxel.hit_count;
  ++accepted_hit_count_;
  block.revision = ++revision_;
  block.change_pending = true;
  return VoxelInsertResult::Inserted;
}

const VoxelBlock* SparseVoxelMap::find_block(const BlockKey& key) const noexcept {
  const auto iterator = blocks_.find(key);
  return iterator == blocks_.end() ? nullptr : &iterator->second;
}

const Voxel* SparseVoxelMap::find(const VoxelKey& key) const noexcept {
  const auto address = address_for(key);
  const auto* block = find_block(address.block);
  if (!block) return nullptr;
  const auto& voxel = block->voxels[address.local_index];
  return voxel.hit_count == 0 ? nullptr : &voxel;
}

const Voxel* SparseVoxelMap::find_at(const MapPoint& point) const noexcept {
  const auto key = voxel_key_for(point);
  return key ? find(*key) : nullptr;
}

std::vector<VoxelNeighbor> SparseVoxelMap::neighbors(const VoxelKey& center, uint32_t radius) const {
  if (radius > 8) throw std::invalid_argument("SparseVoxelMap neighborhood radius must be <=8");
  std::vector<VoxelNeighbor> result;
  const int64_t r = radius;
  for (int64_t z = static_cast<int64_t>(center.z) - r; z <= static_cast<int64_t>(center.z) + r; ++z) {
    if (z < INT32_MIN || z > INT32_MAX) continue;
    for (int64_t y = static_cast<int64_t>(center.y) - r; y <= static_cast<int64_t>(center.y) + r; ++y) {
      if (y < INT32_MIN || y > INT32_MAX) continue;
      for (int64_t x = static_cast<int64_t>(center.x) - r; x <= static_cast<int64_t>(center.x) + r; ++x) {
        if (x < INT32_MIN || x > INT32_MAX) continue;
        const VoxelKey key{static_cast<int32_t>(x), static_cast<int32_t>(y), static_cast<int32_t>(z)};
        if (const auto* voxel = find(key)) result.push_back({key, voxel});
      }
    }
  }
  return result;
}

MapChanges SparseVoxelMap::take_changes() {
  MapChanges result{revision_, reset_pending_, bounds_changed_pending_, {}};
  result.blocks.reserve(pending_blocks_.size());
  for (const auto& key : pending_blocks_) {
    auto& block = blocks_.at(key);
    result.blocks.push_back({key, block.revision});
    block.change_pending = false;
  }
  pending_blocks_.clear();
  reset_pending_ = false;
  bounds_changed_pending_ = false;
  return result;
}

VoxelMapMemoryStats SparseVoxelMap::memory_stats() const noexcept {
  VoxelMapMemoryStats result;
  result.block_count = blocks_.size();
  result.occupied_voxel_count = voxel_count_;
  result.allocated_voxel_slots = blocks_.size() * kVoxelsPerBlock;
  result.block_storage_bytes = blocks_.size() * sizeof(Storage::value_type);
  for (const auto& [key, block] : blocks_) {
    (void)key;
    for (const auto& voxel : block.voxels) {
      result.retained_sample_count += voxel.samples.size();
      result.sample_capacity_bytes += voxel.samples.capacity() * sizeof(MapPoint);
    }
  }
  result.estimated_bytes = sizeof(*this) + result.block_storage_bytes + result.sample_capacity_bytes +
      blocks_.bucket_count() * sizeof(void*) + pending_blocks_.capacity() * sizeof(BlockKey);
  return result;
}

void SparseVoxelMap::reserve_blocks(size_t block_capacity) { blocks_.reserve(block_capacity); }

void SparseVoxelMap::clear() {
  require_revision_capacity();
  blocks_.clear();
  pending_blocks_.clear();
  voxel_count_ = 0;
  accepted_hit_count_ = 0;
  ++revision_;
  reset_pending_ = true;
}

}  // namespace atlas::slam
