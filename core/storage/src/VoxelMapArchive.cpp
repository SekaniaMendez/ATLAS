#include <atlas/storage/VoxelMapArchive.hpp>
#include "BinaryIO.hpp"
#include <algorithm>
#include <tuple>

namespace atlas::storage {
using namespace atlas::slam;

void VoxelMapArchive::write(std::ostream& output, const SparseVoxelMap& map) {
  binary::magic(output, "ATLVOX01");
  binary::Encoder header;
  const auto& config = map.config();
  header.f64(config.voxel_size_m); header.u64(config.max_samples_per_voxel);
  header.u8(config.bounds.has_value());
  if (config.bounds) {
    const auto& b = *config.bounds;
    for (double v : {b.min_x_m, b.min_y_m, b.min_z_m, b.max_x_m, b.max_y_m, b.max_z_m}) header.f64(v);
  }
  header.u64(map.revision()); header.u64(map.accepted_hit_count());
  header.u64(map.voxel_count()); header.u64(map.block_count());
  binary::write_record(output, header);
  std::vector<BlockKey> keys;
  for (const auto& [key, block] : map.blocks()) { (void)block; keys.push_back(key); }
  std::sort(keys.begin(), keys.end(), [](const auto& a, const auto& b) {
    return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
  });
  for (const auto& key : keys) {
    const auto& block = *map.find_block(key);
    binary::Encoder e;
    e.u32(static_cast<uint32_t>(key.x)); e.u32(static_cast<uint32_t>(key.y)); e.u32(static_cast<uint32_t>(key.z));
    e.u64(block.revision); e.u32(static_cast<uint32_t>(block.occupied_voxel_count));
    for (uint32_t i = 0; i < kVoxelsPerBlock; ++i) {
      const auto& voxel = block.voxels[i]; if (!voxel.hit_count) continue;
      e.u32(i); e.u64(voxel.hit_count); e.u64(voxel.samples.size());
      for (const auto& p : voxel.samples) {
        e.f64(p.x_m); e.f64(p.y_m); e.f64(p.z_m); e.u8(p.reflectivity);
        for (auto c : p.rgb) e.u8(c);
        e.u8(p.has_color);
      }
    }
    binary::write_record(output, e);
  }
  binary::magic(output, "VOXEND01");
}

SparseVoxelMap VoxelMapArchive::read(std::istream& input, size_t budget) {
  binary::expect_magic(input, "ATLVOX01");
  const auto header = binary::read_record(input); binary::Decoder d{header};
  SparseVoxelMapConfig config;
  config.voxel_size_m = d.f64();
  const auto capacity = d.u64();
  if (capacity == 0 || capacity > budget / sizeof(MapPoint)) throw std::runtime_error("Invalid snapshot sample capacity");
  config.max_samples_per_voxel = static_cast<size_t>(capacity);
  const auto has_bounds = d.u8(); if (has_bounds > 1) throw std::runtime_error("Invalid bounds flag");
  if (has_bounds) config.bounds = MapBounds{d.f64(), d.f64(), d.f64(), d.f64(), d.f64(), d.f64()};
  const auto revision = d.u64(), hits = d.u64(), voxels = d.u64(), blocks = d.u64();
  d.finish();
  if (blocks > budget / sizeof(VoxelBlock) || hits > revision || voxels > hits)
    throw std::runtime_error("Snapshot budget/counts invalid");
  SparseVoxelMap map(config);
  size_t charged = 0;
  for (uint64_t b = 0; b < blocks; ++b) {
    const auto bytes = binary::read_record(input); binary::Decoder r{bytes};
    const BlockKey key{std::bit_cast<int32_t>(r.u32()), std::bit_cast<int32_t>(r.u32()), std::bit_cast<int32_t>(r.u32())};
    if (!map.key_for_address({key, 0})) throw std::runtime_error("Invalid snapshot block key");
    const auto block_revision = r.u64(); const auto count = r.u32();
    if (count == 0 || count > kVoxelsPerBlock || block_revision == 0 || block_revision > revision)
      throw std::runtime_error("Invalid snapshot block metadata");
    if (sizeof(VoxelBlock) > budget - charged) throw std::runtime_error("Snapshot exceeds load budget");
    charged += sizeof(VoxelBlock);
    auto [it, inserted] = map.blocks_.try_emplace(key);
    if (!inserted) throw std::runtime_error("Duplicate snapshot block");
    auto& block = it->second;
    for (uint32_t i = 0; i < count; ++i) {
      const auto index = r.u32(); const auto hit_count = r.u64(), samples = r.u64();
      if (index >= kVoxelsPerBlock || hit_count == 0 || samples != std::min(hit_count, capacity) ||
          samples > r.remaining() / 29 || samples > (budget - charged) / sizeof(MapPoint))
        throw std::runtime_error("Invalid snapshot voxel or load budget exceeded");
      auto& voxel = block.voxels[index];
      if (voxel.hit_count) throw std::runtime_error("Duplicate snapshot voxel");
      voxel.samples.reserve(static_cast<size_t>(samples));
      charged += static_cast<size_t>(samples) * sizeof(MapPoint);
      const auto voxel_key = map.key_for_address({key, static_cast<uint16_t>(index)});
      for (uint64_t j = 0; j < samples; ++j) {
        MapPoint p{.x_m = r.f64(), .y_m = r.f64(), .z_m = r.f64(), .reflectivity = r.u8(),
                   .rgb = {r.u8(), r.u8(), r.u8()}};
        const auto color = r.u8();
        if (color > 1 || map.voxel_key_for(p) != voxel_key) throw std::runtime_error("Invalid snapshot point");
        p.has_color = color; voxel.samples.push_back(p);
      }
      if (hit_count > UINT64_MAX - map.accepted_hit_count_) throw std::runtime_error("Snapshot hit count overflow");
      voxel.hit_count = hit_count; map.accepted_hit_count_ += hit_count; ++map.voxel_count_;
    }
    r.finish(); block.revision = block_revision; block.occupied_voxel_count = count; block.change_pending = true;
    map.pending_blocks_.push_back(key);
  }
  binary::expect_magic(input, "VOXEND01"); binary::expect_eof(input);
  if (map.accepted_hit_count_ != hits || map.voxel_count_ != voxels) throw std::runtime_error("Snapshot totals mismatch");
  map.revision_ = revision;
  return map; // pending reset and all loaded blocks notify a fresh consumer
}

void VoxelMapArchive::save(const std::filesystem::path& directory, const SparseVoxelMap& map) {
  binary::create_new_directory(directory);
  std::ofstream output(directory / "map.atlas", std::ios::binary | std::ios::trunc);
  write(output, map); output.flush(); output.close();
  if (!output) throw std::runtime_error("Snapshot close failed");
}
SparseVoxelMap VoxelMapArchive::load(const std::filesystem::path& directory, size_t budget) {
  std::ifstream input(directory / "map.atlas", std::ios::binary);
  return read(input, budget);
}
}  // namespace atlas::storage
