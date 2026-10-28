#include <atlas/slam/SparseVoxelMap.hpp>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

namespace {
using namespace atlas::slam;
using Clock = std::chrono::steady_clock;

void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

double milliseconds(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

}  // namespace

int main() {
  try {
    // Binary-exact spacing isolates indexing behavior from decimal rounding.
    // Six surfaces of a 10 x 8 x 4 m room, shifted across the project origin.
    constexpr int nx = 160, ny = 128, nz = 64;
    constexpr double size_m = 0.0625;
    constexpr size_t dense_slots = static_cast<size_t>(nx) * ny * nz;
    constexpr size_t surface_slots = dense_slots - static_cast<size_t>(nx - 2) * (ny - 2) * (nz - 2);
    SparseVoxelMap map({.voxel_size_m = size_m,
                       .bounds = MapBounds{-5, -4, 0, 5, 4, 4},
                       .max_samples_per_voxel = 4});
    std::vector<MapPoint> points;
    points.reserve(surface_slots);
    for (int z = 0; z < nz; ++z)
      for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x) {
          if (x != 0 && x != nx - 1 && y != 0 && y != ny - 1 && z != 0 && z != nz - 1) continue;
          points.push_back({.x_m = (x + 0.5) * size_m - 5,
                            .y_m = (y + 0.5) * size_m - 4,
                            .z_m = (z + 0.5) * size_m,
                            .rgb = {120, 160, 200}, .has_color = true});
        }
    check(points.size() == surface_slots, "synthetic surface count");
    std::mt19937 generator(360);
    std::shuffle(points.begin(), points.end(), generator);
    const auto empty = map.memory_stats();
    check(empty.block_count == 0 && empty.sample_capacity_bytes == 0, "blank canvas allocated geometry");
    (void)map.take_changes();

    const auto insert_start = Clock::now();
    for (const auto& point : points)
      check(map.insert(point) == VoxelInsertResult::Inserted, "room point rejected");
    const auto insert_end = Clock::now();
    check(map.voxel_count() == surface_slots, "room occupied voxel count");
    check(map.find_at({.z_m = 2.0}) == nullptr, "unknown interior became occupied");
    check(map.find_block({0, 0, 4}) == nullptr, "unobserved interior block allocated");
    auto changes = map.take_changes();
    check(changes.blocks.size() == map.block_count(), "changes were not deduplicated");

    // Fill the four-sample ring, then check two more passes do not increase
    // retained data or allocation capacities. Geometry fusion is not tested here.
    for (int pass = 1; pass < 4; ++pass)
      for (const auto& point : points)
        check(map.insert(point) == VoxelInsertResult::Inserted, "repeat point rejected");
    const auto saturated = map.memory_stats();
    const auto update_start = Clock::now();
    for (int pass = 0; pass < 2; ++pass)
      for (const auto& point : points)
        check(map.insert(point) == VoxelInsertResult::Inserted, "update rejected");
    const auto update_end = Clock::now();
    const auto memory = map.memory_stats();
    check(memory.estimated_bytes == saturated.estimated_bytes, "stationary capture grew allocated memory");
    check(memory.retained_sample_count == surface_slots * 4, "ring retention is not bounded");
    check(map.accepted_hit_count() == surface_slots * 6, "observation count");
    check(memory.allocated_voxel_slots < dense_slots, "sparse room used dense volume storage");

    // Include both hits and misses. Shuffle prevents an unrealistically ordered
    // traversal; a checksum ensures Release cannot remove the query loop.
    std::vector<MapPoint> queries = points;
    queries.reserve(points.size() * 2);
    for (size_t i = 0; i < points.size(); ++i) {
      const int x = 1 + static_cast<int>(generator() % (nx - 2));
      const int y = 1 + static_cast<int>(generator() % (ny - 2));
      const int z = 1 + static_cast<int>(generator() % (nz - 2));
      queries.push_back({.x_m = (x + 0.5) * size_m - 5,
                         .y_m = (y + 0.5) * size_m - 4,
                         .z_m = (z + 0.5) * size_m});
    }
    std::shuffle(queries.begin(), queries.end(), generator);
    uint64_t checksum = 0;
    const auto query_start = Clock::now();
    for (const auto& point : queries)
      if (const auto* voxel = map.find_at(point)) checksum += voxel->hit_count;
    const auto query_end = Clock::now();
    check(checksum == surface_slots * 6, "hit/miss query checksum");

    constexpr double mib = 1024.0 * 1024.0;
    // Hypothetical fully dense grid of the SAME Voxel type and retained samples.
    // This is not a comparison against another sparse map implementation.
    const auto dense_estimate = dense_slots * sizeof(Voxel) + memory.sample_capacity_bytes;
    std::cout << std::fixed << std::setprecision(4)
              << "room_m=10x8x4 voxel_m=" << size_m << '\n' << std::setprecision(3)
              << "surface_voxels=" << surface_slots << " blocks=" << map.block_count()
              << " allocated_slots=" << memory.allocated_voxel_slots << " dense_slots=" << dense_slots << '\n'
              << "initial_insert_ms=" << milliseconds(insert_start, insert_end)
              << " initial_Mpoints_s=" << points.size() / milliseconds(insert_start, insert_end) / 1000.0 << '\n'
              << "update_Mpoints_s=" << (points.size() * 2) / milliseconds(update_start, update_end) / 1000.0
              << " query_Mpoints_s=" << queries.size() / milliseconds(query_start, query_end) / 1000.0 << '\n'
              << "estimated_map_MiB=" << memory.estimated_bytes / mib
              << " dense_same_payload_MiB=" << dense_estimate / mib
              << " retained_samples=" << memory.retained_sample_count << '\n'
              << "Memory excludes allocator overhead, process RSS and benchmark input arrays.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
