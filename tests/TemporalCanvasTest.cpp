#include "StorageTestSupport.hpp"
#include "TemporalCanvas.hpp"
#include "../apps/slam_demo/SyntheticRoom.hpp"
#include <atlas/storage/VoxelMapArchive.hpp>
#include <algorithm>
#include <iostream>
#include <limits>

namespace {
using namespace atlas::slam;
constexpr uint64_t tick = 100'000'000;
const MapPoint origin{.x_m = 0.0625, .y_m = 0.0625, .z_m = 0.0625};
const MapPoint body{.x_m = 2.0625, .y_m = 0.0625, .z_m = 0.0625, .reflectivity = 99};
const MapPoint wall{.x_m = 4.0625, .y_m = 0.0625, .z_m = 0.0625, .reflectivity = 50};

void body_leaves_canvas_and_wall_survives() {
  SparseVoxelMap canvas({.voxel_size_m = 0.125, .max_samples_per_voxel = 1});
  SparseVoxelMap matching({.voxel_size_m = 0.5, .max_samples_per_voxel = 8});
  TemporalCanvas temporal({});
  uint64_t time = 0;
  auto observe = [&](MapPoint p, bool reliable = true, size_t copies = 1) {
    std::vector<RayObservation> rays(copies, {origin, p});
    temporal.update(time += tick, rays, reliable, canvas, matching);
  };
  observe(wall, true, 100);
  REQUIRE(canvas.voxel_count() == 0); // 100 returns != 100 independent scans
  observe(wall); observe(wall);
  REQUIRE(canvas.find_at(wall) && temporal.stats().stable_cells == 1);

  // A foreground surface occludes the wall. Even many observations cannot clear it.
  for (int i = 0; i < 8; ++i) observe(body);
  REQUIRE(canvas.find_at(body) && canvas.find_at(wall));
  REQUIRE(temporal.stats().cleared_cells == 0);
  // Preserve neighboring evidence in the SAME coarse matching voxel.
  const MapPoint adjacent{.x_m = 2.31, .y_m = 0.18, .z_m = 0.0625};
  REQUIRE(matching.insert(adjacent) == VoxelInsertResult::Inserted);
  // Unreliable poses cannot clear or age anything, even after a long gap.
  const auto revision = canvas.revision();
  time += 10'000'000'000;
  for (int i = 0; i < 4; ++i) observe(wall, false, 100);
  REQUIRE(canvas.revision() == revision && canvas.find_at(body));
  observe(wall, true, 100);
  observe(wall);
  REQUIRE(canvas.find_at(body)); // two independent contradictions are insufficient
  observe(wall);
  REQUIRE(canvas.find_at(body) == nullptr && canvas.find_at(wall));
  const auto* remaining = matching.find_at(adjacent);
  REQUIRE(remaining && remaining->samples.size() == 1);
  REQUIRE(remaining->samples[0].x_m == adjacent.x_m);
  REQUIRE(temporal.stats().cleared_cells == 1 && temporal.stats().skipped_scans == 4);
  REQUIRE(canvas.voxel_count() == 1 && temporal.stats().candidate_cells == 0);
  const auto changes = canvas.take_changes();
  REQUIRE(std::any_of(changes.blocks.begin(), changes.blocks.end(), [](auto c) { return c.removed; }));
  TempDirectory temp;
  atlas::storage::VoxelMapArchive::save(temp.path / "clean", canvas);
  const auto restored = atlas::storage::VoxelMapArchive::load(temp.path / "clean");
  REQUIRE(!restored.find_at(body) && restored.find_at(wall));
}

void evidence_and_occlusion_guards() {
  SparseVoxelMap canvas({.voxel_size_m = 0.125, .max_samples_per_voxel = 1});
  SparseVoxelMap matching({.voxel_size_m = 0.5, .max_samples_per_voxel = 8});
  TemporalCanvas temporal({});
  uint64_t time = 0;
  auto update = [&](std::vector<RayObservation> rays, uint64_t dt = tick) {
    temporal.update(time += dt, rays, true, canvas, matching);
  };
  for (int i = 0; i < 3; ++i) update({{origin, body}});
  const auto original = canvas.find_at(body)->samples[0];
  auto shifted = body; shifted.x_m += 0.02; shifted.reflectivity = 1;
  for (int i = 0; i < 5; ++i) update({{origin, shifted}});
  REQUIRE(canvas.find_at(body)->samples[0].reflectivity == original.reflectivity);
  REQUIRE(canvas.find_at(body)->samples[0].x_m == original.x_m);
  // No observations do not prove free space. Close surface returns do not either.
  update({}, 3'000'000'000);
  auto near = body; near.x_m += 0.13;
  for (int i = 0; i < 4; ++i) update({{origin, near}});
  REQUIRE(canvas.find_at(body));
  // Sparse contradictions spanning beyond the evidence window cannot accumulate.
  for (int i = 0; i < 4; ++i) update({{origin, wall}}, 2'100'000'000);
  REQUIRE(canvas.find_at(body));
  // A candidate may expire, but this must not erase confirmed geometry nearby.
  const MapPoint fleeting{.x_m = -2.0625, .y_m = 0.0625, .z_m = 0.0625};
  REQUIRE(matching.insert(fleeting) == VoxelInsertResult::Inserted); // provisional seed support
  update({{origin, fleeting}});
  REQUIRE(!canvas.find_at(fleeting));
  update({}, 3'000'000'000);
  REQUIRE(matching.find_at(fleeting) && !canvas.find_at(fleeting) && canvas.find_at(body));
  REQUIRE(temporal.stats().provisional_matching_cells == 1);
  REQUIRE(temporal.stats().expired_candidates > 0);
  // Negative-axis traversal and per-return origin (sensor moved to the far side).
  const MapPoint far{.x_m = -4.0625, .y_m = 0.0625, .z_m = 0.0625};
  for (int i = 0; i < 3; ++i)
    update({{{.x_m = 6.0625, .y_m = 0.0625, .z_m = 0.0625}, far}});
  REQUIRE(!canvas.find_at(body) && !matching.find_at(fleeting));
  const auto revision = canvas.revision();
  rejects([&] { temporal.update(time, {}, true, canvas, matching); });
  REQUIRE(canvas.revision() == revision);

  TemporalCanvas limited({.maximum_cells = 1});
  SparseVoxelMap small({.voxel_size_m = 0.125});
  SparseVoxelMap reference({.voxel_size_m = 0.5});
  const std::vector<RayObservation> two{{origin, wall}, {origin, body}};
  rejects([&] { limited.update(tick, two, true, small, reference); });
  REQUIRE(small.voxel_count() == 0 && limited.stats().candidate_cells == 0);

  TemporalCanvas block_limited({}, 1, 1);
  block_limited.update(tick, two, true, small, reference);
  block_limited.update(2 * tick, two, true, small, reference);
  rejects([&] { block_limited.update(3 * tick, two, true, small, reference); });
  REQUIRE(small.voxel_count() == 0 && reference.voxel_count() == 0);
}

void ray_corner_is_not_free_evidence() {
  SparseVoxelMap canvas({.voxel_size_m = 0.125});
  SparseVoxelMap matching({.voxel_size_m = 0.5});
  TemporalCanvas temporal({});
  // x=y rays touch this cell's corner but do not traverse its interior.
  const MapPoint beside{.x_m = 0.63, .y_m = 0.62, .z_m = 0.0625};
  const MapPoint diagonal{.x_m = 4.0625, .y_m = 4.0625, .z_m = 0.0625};
  const std::vector<RayObservation> initial{{origin, beside}}, later{{origin, diagonal}};
  for (uint64_t n = 1; n <= 3; ++n) temporal.update(n * tick, initial, true, canvas, matching);
  for (uint64_t n = 4; n <= 9; ++n) temporal.update(n * tick, later, true, canvas, matching);
  REQUIRE(canvas.find_at(beside) && temporal.stats().cleared_cells == 0);
}

size_t body_points(const VoxelSlamEngine& engine) {
  const auto points = engine.map_points(1'000'000);
  return std::count_if(points.begin(), points.end(), [](const MapPoint& p) {
    return p.x_m > 1.40 && p.x_m < 1.60 && std::abs(p.y_m) < 0.36 && std::abs(p.z_m) < 0.46;
  });
}

void engine_restores_canvas() {
  atlas::demo::SlamDemoConfig scene;
  scene.points_per_scan = 6000;
  scene.velocity_m_s = {0, 0, 0}; scene.yaw_rate_rad_s = 0;
  scene.range_noise_stddev_m = 0; scene.motion_during_scan = false;
  atlas::demo::SyntheticRoom room(scene);
  VoxelSlamConfig config;
  config.use_imu = false;
  VoxelSlamEngine engine(config);
  auto feed = [&](size_t index, bool object, bool invalid = false) {
    auto frame = room.frame(index);
    for (auto& p : frame.points) {
      // A small foreground panel physically occludes returns from the room.
      if (object && p.x_m > 1.5F) {
        const float scale = 1.5F / p.x_m;
        if (std::abs(p.y_m * scale) < 0.3F && std::abs(p.z_m * scale) < 0.4F) {
          p.x_m *= scale; p.y_m *= scale; p.z_m *= scale; p.reflectivity = 220;
        }
      }
      if (invalid) p.x_m += 100;
    }
    engine.on_lidar_frame(frame); engine.wait_until_idle();
  };
  for (size_t i = 0; i < 4; ++i) feed(i, false);
  REQUIRE(engine.stats().map_voxels > 1000 && body_points(engine) == 0);
  for (size_t i = 4; i < 10; ++i) feed(i, true);
  const auto occupied = body_points(engine);
  REQUIRE(occupied > 5);
  const auto before = engine.stats();
  feed(10, false, true);
  REQUIRE(engine.stats().map_voxels == before.map_voxels);
  REQUIRE(engine.stats().cleared_voxels == before.cleared_voxels);
  // Bounded ray sampling needs several scans to revisit every beam three times.
  for (size_t i = 11; i < 27; ++i) feed(i, false);
  engine.finish();
  std::cout << "Moving foreground: initial=" << occupied << " remaining=" << body_points(engine)
            << " cleared=" << engine.stats().cleared_voxels << " status=" << engine.stats().message << '\n';
  REQUIRE(engine.stats().state == TrackingState::Tracking);
  REQUIRE(body_points(engine) == 0 && engine.stats().cleared_voxels >= occupied);
  REQUIRE(room.evaluate_map(engine.map_points(engine.stats().map_voxels)).surface_rmse_m < 0.02);
  TempDirectory temp;
  engine.save_map(temp.path / "map");
  engine.export_ply(temp.path / "map.ply");
  const auto saved = atlas::storage::VoxelMapArchive::load(temp.path / "map");
  REQUIRE(saved.voxel_count() == engine.stats().map_voxels);
  engine.reset();
  REQUIRE(engine.stats().candidate_voxels == 0 && engine.stats().cleared_voxels == 0);
  REQUIRE(engine.map_points(100).empty());
}
}  // namespace

int main() {
  try {
    body_leaves_canvas_and_wall_survives();
    evidence_and_occlusion_guards();
    ray_corner_is_not_free_evidence();
    engine_restores_canvas();
    std::cout << "Temporal canvas: confirmation, occlusion, deletion, pose guards and exports passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
