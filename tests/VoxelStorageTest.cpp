#include "StorageTestSupport.hpp"
#include <atlas/storage/VoxelMapArchive.hpp>
#include <atlas/storage/PagedVoxelMap.hpp>
#include <sqlite3.h>
#include <iostream>
#include <sstream>

int main() {
  try {
    using namespace atlas::slam;
    using namespace atlas::storage;
    TempDirectory temp;
    SparseVoxelMap map({.voxel_size_m = 0.25, .max_samples_per_voxel = 3});
    for (int i = 0; i < 20; ++i)
      REQUIRE(map.insert({.x_m = -1.1, .z_m = 0.1, .reflectivity = static_cast<uint8_t>(i), .rgb = {2, 3, 4}, .has_color = true}) == VoxelInsertResult::Inserted);
    REQUIRE(map.insert({.x_m = 100}) == VoxelInsertResult::Inserted);
    map.set_bounds(MapBounds{-1, -1, -1, 1, 1, 1});
    VoxelMapArchive::save(temp.path / "snapshot", map);
    auto loaded = VoxelMapArchive::load(temp.path / "snapshot");
    REQUIRE(loaded.voxel_count() == map.voxel_count() && loaded.block_count() == map.block_count());
    REQUIRE(loaded.revision() == map.revision() && loaded.accepted_hit_count() == 21);
    REQUIRE(loaded.find_at({.x_m = 100}) != nullptr); // preserved outside edited boundary
    REQUIRE(loaded.find_at({.x_m = -1.1, .z_m = 0.1})->samples[0].reflectivity == 18);
    REQUIRE(loaded.take_changes().reset);
    loaded.set_bounds(std::nullopt);
    REQUIRE(loaded.insert({.x_m = -1.1, .z_m = 0.1, .reflectivity = 99}) == VoxelInsertResult::Inserted);
    REQUIRE(loaded.find_at({.x_m = -1.1, .z_m = 0.1})->samples[2].reflectivity == 99);
    rejects([&] { (void)VoxelMapArchive::load(temp.path / "snapshot", 1); });
    rejects([&] { VoxelMapArchive::save(temp.path / "snapshot", map); });
    std::stringstream serialized(std::ios::binary | std::ios::in | std::ios::out);
    VoxelMapArchive::write(serialized, map);
    std::string damaged = serialized.str(); damaged[damaged.size() - 12] ^= 1;
    std::istringstream bad(damaged); rejects([&] { (void)VoxelMapArchive::read(bad, 10000000); });

    // A cleared cell stays unknown across save/load; no stale block is resurrected.
    REQUIRE(map.erase(*map.voxel_key_for({.x_m = 100})));
    VoxelMapArchive::save(temp.path / "erased", map);
    auto erased = VoxelMapArchive::load(temp.path / "erased");
    REQUIRE(erased.find_at({.x_m = 100}) == nullptr);
    REQUIRE(erased.voxel_count() == 1 && erased.accepted_hit_count() == 20);
    const auto kept_key = *erased.voxel_key_for({.x_m = -1.1, .z_m = 0.1});
    REQUIRE(erased.erase_samples(kept_key, [](const MapPoint& p) { return p.reflectivity == 18; }) == 1);
    VoxelMapArchive::save(temp.path / "partial-erased", erased);
    auto partial = VoxelMapArchive::load(temp.path / "partial-erased");
    REQUIRE(partial.find(kept_key)->samples.size() == 2 && partial.accepted_hit_count() == 2);
    REQUIRE(partial.erase(kept_key));
    VoxelMapArchive::save(temp.path / "blank", partial);
    auto blank = VoxelMapArchive::load(temp.path / "blank");
    REQUIRE(blank.voxel_count() == 0 && blank.block_count() == 0);

    const auto paged_path = temp.path / "paged";
    const auto reservation = PagedVoxelMap::block_payload_reservation(3);
    {
      PagedVoxelMap paged(paged_path, PagedVoxelMap::Mode::Create,
                           {.voxel_size_m = 0.25, .max_samples_per_voxel = 3}, reservation * 2);
      rejects([&] { PagedVoxelMap other(paged_path, PagedVoxelMap::Mode::Open); });
      for (int pass = 0; pass < 5; ++pass)
        for (int block = -12; block < 12; ++block) {
          REQUIRE(paged.insert({.x_m = block * 2.0 + 0.1, .reflectivity = static_cast<uint8_t>(pass)}) == VoxelInsertResult::Inserted);
          REQUIRE(paged.stats().resident_blocks <= 2 && paged.stats().reserved_payload_bytes <= reservation * 2);
        }
      REQUIRE(paged.stats().evictions > 100);
      auto copy = paged.get_voxel({-96, 0, 0}); REQUIRE(copy && copy->hit_count == 5);
      REQUIRE(copy->samples[0].reflectivity == 3 && copy->samples[1].reflectivity == 4);
      REQUIRE(!paged.get_voxel({10000, 0, 0}));
      paged.set_bounds(MapBounds{-1, -1, -1, 1, 1, 1}); paged.flush();
    }
    {
      PagedVoxelMap reopened(paged_path, PagedVoxelMap::Mode::Open, {}, reservation * 2);
      REQUIRE(reopened.config().voxel_size_m == 0.25);
      REQUIRE(reopened.config().bounds->max_x_m == 1);
      for (int block = -12; block < 12; ++block) {
        const auto voxel = reopened.get_voxel({block * 8, 0, 0}); REQUIRE(voxel && voxel->hit_count == 5);
      }
      REQUIRE(reopened.insert({.x_m = 100}) == VoxelInsertResult::OutsideBounds);
      reopened.set_bounds(std::nullopt);
      REQUIRE(reopened.insert({.x_m = 100}) == VoxelInsertResult::Inserted);
      // Inject a deterministic disk-commit failure; dirty data must survive.
      sqlite3* db = nullptr; REQUIRE(sqlite3_open((paged_path / "blocks.sqlite").string().c_str(), &db) == SQLITE_OK);
      REQUIRE(sqlite3_exec(db, "CREATE TRIGGER fail_write BEFORE INSERT ON blocks BEGIN SELECT RAISE(ABORT,'test I/O failure'); END;", nullptr, nullptr, nullptr) == SQLITE_OK);
      rejects([&] { reopened.flush(); });
      REQUIRE(reopened.get_voxel({400, 0, 0})->hit_count == 1);
      REQUIRE(sqlite3_exec(db, "DROP TRIGGER fail_write", nullptr, nullptr, nullptr) == SQLITE_OK); sqlite3_close(db);
      reopened.flush();
    }
    { PagedVoxelMap final(paged_path, PagedVoxelMap::Mode::Open); REQUIRE(final.get_voxel({400, 0, 0})->hit_count == 1); }
    std::cout << "Map snapshot, ring continuation, bounded paging, reopen and failed-commit recovery passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
