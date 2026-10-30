#include <atlas/slam/SparseVoxelMap.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {
using namespace atlas::slam;

// Unlike assert, checks and their expressions execute in Release too.
#define CHECK(...) do { if (!(__VA_ARGS__)) throw std::runtime_error( \
    std::string("line ") + std::to_string(__LINE__) + ": " + #__VA_ARGS__); } while (false)

template <typename Function>
void expect_invalid(Function function) {
  bool threw = false;
  try { function(); } catch (const std::invalid_argument&) { threw = true; }
  CHECK(threw);
}

void coordinates_and_blocks() {
  SparseVoxelMap map({.voxel_size_m = 0.125});
  CHECK(map.block_count() == 0);
  CHECK(map.find({0, 0, 0}) == nullptr);
  CHECK(map.voxel_key_for({.x_m = -0.001}) == VoxelKey{-1, 0, 0});
  CHECK(map.voxel_key_for({.x_m = -1.0}) == VoxelKey{-8, 0, 0});
  CHECK(map.voxel_key_for({.x_m = std::nextafter(-1.0, -2.0)}) == VoxelKey{-9, 0, 0});
  CHECK(map.voxel_key_for({.x_m = std::nextafter(1.0, 0.0)}) == VoxelKey{7, 0, 0});
  CHECK(map.voxel_key_for({.x_m = 1.0}) == VoxelKey{8, 0, 0});

  // Cover every local index and transitions across negative and positive blocks.
  for (int z = -17; z <= 17; ++z)
    for (int y = -17; y <= 17; ++y)
      for (int x = -17; x <= 17; ++x) {
        const VoxelKey key{x, y, z};
        const auto address = map.address_for(key);
        CHECK(map.key_for_address(address) == key);
        CHECK(address.local_index < kVoxelsPerBlock);
        const int local_x = address.local_index % 8;
        const int local_y = (address.local_index / 8) % 8;
        const int local_z = address.local_index / 64;
        CHECK(address.block.x * 8 + local_x == x);
        CHECK(address.block.y * 8 + local_y == y);
        CHECK(address.block.z * 8 + local_z == z);
        const auto point = map.voxel_center(key);
        CHECK(map.voxel_key_for(point) == key);
        CHECK(map.voxel_bounds(key).contains(point));
      }

  for (const auto value : {INT32_MIN, INT32_MIN + 1, INT32_MAX - 1, INT32_MAX}) {
    const VoxelKey key{value, value, value};
    const auto address = map.address_for(key);
    CHECK(map.key_for_address(address) == key);
    CHECK(static_cast<int64_t>(address.block.x) * 8 + address.local_index % 8 == value);
    const auto point = map.voxel_center(key);
    CHECK(map.voxel_key_for(point) == key);
    CHECK(map.insert(point) == VoxelInsertResult::Inserted);
    CHECK(map.find(key) != nullptr);
    CHECK(!map.neighbors(key).empty()); // must not wrap coordinates
  }

  SparseVoxelMap adjacent({.voxel_size_m = 1.0});
  CHECK(!map.key_for_address({{0, 0, 0}, 512}));
  CHECK(!map.key_for_address({{INT32_MAX, 0, 0}, 0}));
  CHECK(!map.key_for_address({{0, INT32_MIN, 0}, 0}));
  for (const int x : {0, 7, 8, -1, -8, -9})
    CHECK(adjacent.insert({.x_m = x + 0.25}) == VoxelInsertResult::Inserted);
  CHECK(adjacent.block_count() == 4);
  CHECK(adjacent.voxel_count() == 6);
  CHECK(adjacent.find({1, 0, 0}) == nullptr); // allocated cell still unknown
  const auto* stable_pointer = adjacent.find({0, 0, 0});
  adjacent.reserve_blocks(1000);
  CHECK(adjacent.find({0, 0, 0}) == stable_pointer);
}

void editable_bounds_and_errors() {
  SparseVoxelMap map({.voxel_size_m = 1.0, .bounds = MapBounds{-2, -2, -2, 2, 2, 2}});
  CHECK(map.config().bounds->dimensions_m() == std::array<double, 3>{4, 4, 4});
  CHECK(map.insert({.x_m = -2.0}) == VoxelInsertResult::Inserted);
  CHECK(map.insert({.x_m = 2.0}) == VoxelInsertResult::OutsideBounds);
  CHECK(map.insert({.x_m = -2.001}) == VoxelInsertResult::OutsideBounds);
  const auto* preserved = map.find({-2, 0, 0});
  map.set_bounds(MapBounds{-1, -1, -1, 1, 1, 1});
  CHECK(map.find_at({.x_m = -2.0}) == preserved);
  CHECK(map.voxel_key_for({.x_m = -2.0}) == VoxelKey{-2, 0, 0});
  CHECK(map.insert({.x_m = -2.0}) == VoxelInsertResult::OutsideBounds);
  map.set_bounds(MapBounds{-3, -3, -3, 3, 3, 3});
  CHECK(map.insert({.x_m = 2.0}) == VoxelInsertResult::Inserted);
  map.set_bounds(std::nullopt);
  CHECK(map.insert({.x_m = 1000.0}) == VoxelInsertResult::Inserted);

  const auto revision = map.revision();
  const auto count = map.accepted_hit_count();
  for (const double bad : {std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    CHECK(map.insert({.x_m = bad}) == VoxelInsertResult::InvalidPoint);
    CHECK(map.insert({.y_m = bad}) == VoxelInsertResult::InvalidPoint);
    CHECK(map.insert({.z_m = bad}) == VoxelInsertResult::InvalidPoint);
    CHECK(map.find_at({.x_m = bad}) == nullptr);
  }
  CHECK(map.insert({.x_m = 2147483648.0}) == VoxelInsertResult::CoordinateOutOfRange);
  CHECK(map.insert({.x_m = -2147483649.0}) == VoxelInsertResult::CoordinateOutOfRange);
  CHECK(map.insert({.x_m = std::numeric_limits<double>::max()}) ==
        VoxelInsertResult::CoordinateOutOfRange);
  expect_invalid([&] { map.set_bounds(MapBounds{}); });
  expect_invalid([&] { map.set_bounds(MapBounds{1, 0, 0, -1, 1, 1}); });
  CHECK(!map.config().bounds);
  CHECK(map.revision() == revision);
  CHECK(map.accepted_hit_count() == count);

  for (const double size : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::denorm_min(),
                            std::numeric_limits<double>::max()})
    expect_invalid([&] { SparseVoxelMap invalid({.voxel_size_m = size}); });
  expect_invalid([] { SparseVoxelMap invalid({.bounds = MapBounds{}}); });
  expect_invalid([] { SparseVoxelMap invalid({.max_samples_per_voxel = 0}); });
  SparseVoxelMap tiny({.voxel_size_m = std::numeric_limits<double>::min()});
  CHECK(tiny.insert({.x_m = 1.0}) == VoxelInsertResult::CoordinateOutOfRange);
}

void changes_and_recent_samples() {
  SparseVoxelMap map({.voxel_size_m = 1.0, .max_samples_per_voxel = 2});
  auto changes = map.take_changes();
  CHECK(changes.reset && changes.bounds_changed && changes.blocks.empty());
  CHECK(!map.take_changes().reset);
  for (int i = 0; i < 5; ++i)
    CHECK(map.insert({.x_m = 0.25, .reflectivity = static_cast<uint8_t>(i),
                      .rgb = {static_cast<uint8_t>(i), 20, 30}, .has_color = i > 0}) ==
          VoxelInsertResult::Inserted);
  changes = map.take_changes();
  CHECK(changes.blocks.size() == 1);
  CHECK(changes.blocks.front().key == BlockKey{0, 0, 0});
  CHECK(changes.blocks.front().revision == 5);
  CHECK(changes.revision == 5);
  CHECK(!changes.reset);
  const auto* voxel = map.find({0, 0, 0});
  CHECK(voxel && voxel->hit_count == 5 && voxel->samples.size() == 2);
  CHECK(voxel->samples[0].reflectivity == 4);
  CHECK(voxel->samples[1].reflectivity == 3);
  CHECK(voxel->samples[0].has_color && voxel->samples[0].rgb[0] == 4);
  CHECK(map.take_changes().blocks.empty());

  CHECK(map.insert({.x_m = 0.5}) == VoxelInsertResult::Inserted);
  CHECK(map.insert({.x_m = 1.5}) == VoxelInsertResult::Inserted); // same block
  CHECK(map.insert({.x_m = 8.5}) == VoxelInsertResult::Inserted);
  changes = map.take_changes();
  CHECK(changes.blocks.size() == 2);
  CHECK(changes.blocks[0].revision == 7 && changes.blocks[1].revision == 8);
  CHECK(map.find_block({0, 0, 0})->occupied_voxel_count == 2);
  map.set_bounds(MapBounds{-1, -1, -1, 1, 1, 1});
  changes = map.take_changes();
  CHECK(changes.bounds_changed && !changes.reset && changes.blocks.empty());
  CHECK(map.insert({.x_m = 2}) == VoxelInsertResult::OutsideBounds);
  CHECK(map.take_changes().blocks.empty());

  CHECK(map.insert({.x_m = 0.5}) == VoxelInsertResult::Inserted);
  map.clear(); // clears pending block notifications too
  CHECK(map.voxel_count() == 0 && map.block_count() == 0 && map.accepted_hit_count() == 0);
  CHECK(map.config().bounds.has_value());
  CHECK(map.insert({.x_m = -0.5}) == VoxelInsertResult::Inserted);
  changes = map.take_changes();
  CHECK(changes.reset && changes.blocks.size() == 1);
  CHECK(changes.blocks[0].key == BlockKey{-1, 0, 0});
  CHECK(changes.blocks[0].revision == changes.revision);
  CHECK(!map.take_changes().reset);

  const auto memory = map.memory_stats();
  CHECK(memory.block_count == 1 && memory.occupied_voxel_count == 1);
  CHECK(memory.retained_sample_count == 1 && memory.allocated_voxel_slots == 512);
  CHECK(memory.sample_capacity_bytes >= sizeof(MapPoint));
  CHECK(memory.estimated_bytes >= memory.block_storage_bytes + memory.sample_capacity_bytes);
}

void neighborhoods_and_reference_model() {
  SparseVoxelMap map({.voxel_size_m = 1.0, .max_samples_per_voxel = 3});
  // Cross an eight-block corner.
  for (int z = 6; z <= 8; ++z)
    for (int y = 6; y <= 8; ++y)
      for (int x = 6; x <= 8; ++x)
        CHECK(map.insert(map.voxel_center({x, y, z})) == VoxelInsertResult::Inserted);
  CHECK(map.block_count() == 8);
  CHECK(map.neighbors({7, 7, 7}).size() == 27);
  CHECK(map.neighbors({7, 7, 7}, 0).size() == 1);
  CHECK(map.neighbors({100, 100, 100}).empty());
  expect_invalid([&] { (void)map.neighbors({0, 0, 0}, 9); });

  map.clear();
  std::mt19937 generator(2026);
  std::uniform_int_distribution<int> coordinate(-12, 12);
  std::unordered_map<VoxelKey, uint64_t, VoxelKeyHash> reference;
  for (int i = 0; i < 10000; ++i) {
    const VoxelKey key{coordinate(generator), coordinate(generator), coordinate(generator)};
    const MapPoint point{.x_m = key.x + 0.25, .y_m = key.y + 0.25, .z_m = key.z + 0.25};
    CHECK(map.insert(point) == VoxelInsertResult::Inserted);
    ++reference[key];
  }
  CHECK(map.voxel_count() == reference.size());
  CHECK(map.accepted_hit_count() == 10000);
  size_t expected_samples = 0;
  for (const auto& [key, hits] : reference) {
    const auto* voxel = map.find(key);
    CHECK(voxel && voxel->hit_count == hits);
    CHECK(voxel->samples.size() == std::min<uint64_t>(hits, 3));
    expected_samples += voxel->samples.size();
  }
  CHECK(map.memory_stats().retained_sample_count == expected_samples);
  for (int i = 0; i < 100; ++i) {
    const VoxelKey center{coordinate(generator), coordinate(generator), coordinate(generator)};
    size_t expected = 0;
    for (const auto& [key, hits] : reference) {
      (void)hits;
      if (std::abs(key.x - center.x) <= 1 && std::abs(key.y - center.y) <= 1 &&
          std::abs(key.z - center.z) <= 1) ++expected;
    }
    const auto neighbors = map.neighbors(center);
    CHECK(neighbors.size() == expected);
    for (const auto& neighbor : neighbors) CHECK(neighbor.voxel == map.find(neighbor.key));
  }
}

void selective_erasure() {
  SparseVoxelMap map({.voxel_size_m = 0.125, .max_samples_per_voxel = 3});
  (void)map.take_changes();
  const MapPoint foreground{.x_m = -0.10, .reflectivity = 10};
  const MapPoint background{.x_m = -0.09, .reflectivity = 20};
  (void)map.insert(foreground);
  (void)map.insert(background);
  const auto key = *map.voxel_key_for(foreground);
  CHECK(map.erase_samples(key, [](const MapPoint& p) { return p.reflectivity == 10; }) == 1);
  CHECK(map.find(key)->samples.size() == 1);
  CHECK(map.find(key)->samples[0].reflectivity == 20);
  CHECK(map.accepted_hit_count() == 1 && map.voxel_count() == 1);
  auto changes = map.take_changes();
  CHECK(changes.blocks.size() == 1 && !changes.blocks[0].removed && !changes.reset);
  const auto revision = map.revision();
  CHECK(!map.erase({100, 100, 100}));
  CHECK(map.erase_samples(key, [](const MapPoint&) { return false; }) == 0);
  CHECK(map.revision() == revision && map.take_changes().blocks.empty());
  try {
    map.erase_samples(key, [](const MapPoint&) -> bool { throw std::invalid_argument("predicate"); });
    CHECK(false);
  } catch (const std::invalid_argument&) {}
  CHECK(map.revision() == revision && map.find(key)->samples.size() == 1);
  CHECK(map.erase(key));
  CHECK(map.find(key) == nullptr && map.block_count() == 0 && map.voxel_count() == 0);
  CHECK(map.accepted_hit_count() == 0);
  changes = map.take_changes();
  CHECK(changes.blocks.size() == 1 && changes.blocks[0].removed && !changes.reset);
  CHECK(changes.blocks[0].revision == revision + 1);
  CHECK(map.config().voxel_size_m == 0.125);

  // Recreate/delete/recreate before draining must emit one latest-state event.
  (void)map.insert(foreground);
  CHECK(map.erase(key));
  (void)map.insert(background);
  changes = map.take_changes();
  CHECK(changes.blocks.size() == 1 && !changes.blocks[0].removed);
  CHECK(changes.blocks[0].revision == map.revision());
  CHECK(map.erase(key));
  map.clear();
  changes = map.take_changes();
  CHECK(changes.reset && changes.blocks.empty());

  // Deleting one cell leaves adjacent geometry and its counts unchanged.
  (void)map.insert({.x_m = 0.01});
  (void)map.insert({.x_m = 0.2});
  CHECK(map.erase({0, 0, 0}));
  CHECK(map.block_count() == 1 && map.voxel_count() == 1 && map.accepted_hit_count() == 1);
  CHECK(map.find_at({.x_m = 0.2}) != nullptr);
  CHECK(map.find_block({0, 0, 0})->occupied_voxel_count == 1);
}

}  // namespace

int main() {
  try {
    coordinates_and_blocks();
    editable_bounds_and_errors();
    changes_and_recent_samples();
    neighborhoods_and_reference_model();
    selective_erasure();
    std::cout << "SparseVoxelMap: coordinate, bounds, change, sample and reference checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
