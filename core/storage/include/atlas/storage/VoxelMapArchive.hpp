#pragma once
#include <atlas/slam/SparseVoxelMap.hpp>
#include <atlas/storage/voxel_map_archive_config.hpp>
#include <filesystem>
#include <iosfwd>

namespace atlas::storage {
/// Versioned little-endian snapshots with per-record checksums. Destination
/// directory must not exist. Full in-memory load is explicitly separate from
/// the paged store. A complete snapshot preserves sample order, hits and revision.
class VoxelMapArchive {
 public:
  static void save(const std::filesystem::path& directory, const slam::SparseVoxelMap& map);
  static slam::SparseVoxelMap load(const std::filesystem::path& directory,
                                  size_t max_payload_bytes = kDefaultVoxelMapArchivePayloadBudgetBytes);
  static void write(std::ostream&, const slam::SparseVoxelMap&);
  static slam::SparseVoxelMap read(std::istream&, size_t max_payload_bytes);
};
}
