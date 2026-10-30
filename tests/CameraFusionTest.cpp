#include "StorageTestSupport.hpp"
#include "CameraColorizer.hpp"
#include "TemporalCanvas.hpp"
#include "../apps/slam_demo/SyntheticRoom.hpp"
#include <atlas/storage/VoxelMapArchive.hpp>
#include <iostream>

using namespace atlas;
namespace {
constexpr uint64_t time_ns = 1'000'000'000;
slam::CameraFusionConfig configuration() {
  slam::CameraFusionConfig c;
  c.clock_alignment_verified = true; c.clock_uncertainty_ns = 1'000'000;
  c.lidar_clock = timesync::ClockSource::Unsynchronized;
  return c;
}
sensors::CameraRgbFrame image(uint64_t timestamp = time_ns) {
  sensors::CameraRgbFrame f;
  f.timestamp_ns = timestamp; f.width = 8; f.height = 8;
  f.intrinsics = {4, 4, 4, 4};
  f.camera_from_lidar_rotation = {1,0,0,0,1,0,0,0,1};
  f.calibration_verified = f.tracking_normal = true;
  f.rgb.resize(8*8*3);
  for (size_t y = 0; y < 8; ++y) for (size_t x = 0; x < 8; ++x) {
    const size_t i = (y*8+x)*3;
    f.rgb[i] = uint8_t(x*20); f.rgb[i+1] = uint8_t(y*20); f.rgb[i+2] = 25;
  }
  return f;
}
void projection_guards() {
  auto c = configuration(); c.depth_cell_pixels = 1;
  slam::CameraColorizer color(c);
  REQUIRE(color.submit(image())); REQUIRE(!color.submit(image()));
  std::vector<sensors::LidarPoint> p{{.x_m=0,.y_m=0,.z_m=-2},
      {.x_m=0,.y_m=0,.z_m=-4}, {.x_m=0,.y_m=0,.z_m=2},
      {.x_m=0,.y_m=1,.z_m=-2}, {.x_m=20,.y_m=0,.z_m=-2}};
  REQUIRE(color.colorize(p, {time_ns, c.lidar_clock}) == 2);
  REQUIRE(p[0].rgb == std::array<uint8_t,3>{80,80,25});
  REQUIRE(p[3].rgb == std::array<uint8_t,3>{80,40,25}); // top-row-first, Y up
  REQUIRE(!p[1].has_color && !p[2].has_color && !p[4].has_color);
  REQUIRE(color.colorize(p, {time_ns+14'000'001,c.lidar_clock}) == 0);
  REQUIRE(color.colorize(p, {time_ns,timesync::ClockSource::Ptp}) == 0);
  p[0].offset_ns = 100'000'000; p[3].offset_ns = 0;
  REQUIRE(color.colorize(p, {time_ns,c.lidar_clock}, time_ns-100'000'000) == 1);
  c.phone_to_lidar_offset_ns = -100'000'000;
  slam::CameraColorizer offset(c); REQUIRE(offset.submit(image(time_ns+100'000'000)));
  REQUIRE(offset.colorize(p, {time_ns,c.lidar_clock}) == 2);
  c.phone_to_lidar_offset_ns = INT64_MIN;
  slam::CameraColorizer underflow(c); REQUIRE(underflow.submit(image()));
  REQUIRE(underflow.colorize(p, {time_ns,c.lidar_clock}) == 0);
  c = configuration(); c.clock_alignment_verified = false;
  slam::CameraColorizer unaligned(c); REQUIRE(unaligned.submit(image()));
  REQUIRE(unaligned.colorize(p, {time_ns,c.lidar_clock}) == 0);
  for (bool tracking : {false,true}) {
    auto f = image(); f.tracking_normal = tracking; f.calibration_verified = !tracking;
    slam::CameraColorizer invalid(configuration()); REQUIRE(invalid.submit(f));
    REQUIRE(invalid.colorize(p, {time_ns,c.lidar_clock}) == 0);
  }
  auto rig_frame = image();
  rig_frame.camera_from_lidar_rotation = {0,0,-1,0,-1,0,-1,0,0};
  rig_frame.camera_from_lidar_translation = {.075,-.025,-.031};
  slam::CameraColorizer rig(configuration()); REQUIRE(rig.submit(rig_frame));
  std::vector<sensors::LidarPoint> rig_point{{.x_m=1.969F,.y_m=-.825F,.z_m=-.125F}};
  REQUIRE(rig.colorize(rig_point,{time_ns,c.lidar_clock}) == 1);
  REQUIRE(rig_point[0].rgb == std::array<uint8_t,3>{80,40,25});
  auto bad = image(); bad.rgb.pop_back(); REQUIRE(!color.submit(bad));
  bad = image(); bad.camera_from_lidar_rotation[0] = -1; REQUIRE(!color.submit(bad));
  bad = image(); bad.intrinsics[0] = 0; REQUIRE(!color.submit(bad));
  bad = image(); bad.width = UINT32_MAX; REQUIRE(!color.submit(bad));
}
void color_delays_candidates_only() {
  slam::SparseVoxelMap canvas({.voxel_size_m=.125, .max_samples_per_voxel=1});
  slam::SparseVoxelMap matching({.voxel_size_m=.5, .max_samples_per_voxel=8});
  slam::TemporalCanvas temporal({});
  uint64_t t = time_ns;
  slam::MapPoint p{.x_m=2.0625,.y_m=.0625,.z_m=.0625,.rgb={200,40,40},.has_color=true};
  auto step = [&](uint64_t exposure) {
    p.color_timestamp_ns = exposure;
    const std::vector<slam::RayObservation> rays{{{.x_m=.0625,.y_m=.0625,.z_m=.0625},p}};
    temporal.update(t += 100'000'000,rays,true,canvas,matching);
  };
  step(1); step(2);
  p.rgb = {40,40,200}; step(3);
  REQUIRE(canvas.voxel_count() == 0 && temporal.stats().color_conflicts == 1);
  for (int i=0;i<5;++i) step(3); // repeated image is not independent evidence
  REQUIRE(canvas.voxel_count() == 0);
  p.rgb = {20,20,100}; step(4); // uniform exposure change is not a color change
  REQUIRE(canvas.voxel_count() == 0 && temporal.stats().color_conflicts == 1);
  step(5); REQUIRE(canvas.voxel_count() == 1);
  const auto original = canvas.find_at(p)->samples[0].rgb;
  for (uint64_t i=6;i<12;++i) { p.rgb = i%2 ? std::array<uint8_t,3>{200,40,40} : std::array<uint8_t,3>{40,200,40}; step(i); }
  REQUIRE(canvas.voxel_count() == 1 && canvas.find_at(p)->samples[0].rgb == original);
  REQUIRE(temporal.stats().cleared_cells == 0);
  p.x_m = 4.0625; p.has_color = false;
  for (int i=0;i<3;++i) step(0);
  REQUIRE(!canvas.find_at({.x_m=2.0625,.y_m=.0625,.z_m=.0625}));
  REQUIRE(temporal.stats().cleared_cells == 1); // only geometric rays can clear
}
void engine_preserves_rgb() {
  demo::SlamDemoConfig simulation; simulation.scan_count = 4; simulation.points_per_scan = 4000;
  demo::SyntheticRoom room(simulation);
  slam::VoxelSlamConfig c; c.use_imu = false; c.camera_fusion = configuration();
  c.camera_fusion.maximum_wait_ns = 0;
  slam::VoxelSlamEngine engine(c);
  for (size_t i=0;i<4;++i) {
    auto scan = room.frame(i);
    auto f = image(scan.timestamp.nanoseconds + scan.duration_ns);
    REQUIRE(engine.submit_camera_image(f));
    engine.on_lidar_frame(scan); engine.wait_until_idle();
  }
  engine.finish(); REQUIRE(!engine.stats().fatal_error && engine.stats().colored_points > 0);
  auto points = engine.map_points(100'000);
  REQUIRE(std::any_of(points.begin(),points.end(),[](auto& p){return p.has_color;}));
  TempDirectory temp; engine.save_map(temp.path/"map"); engine.export_ply(temp.path/"map.ply");
  auto restored = storage::VoxelMapArchive::load(temp.path/"map");
  size_t colors=0;
  for (const auto& p : points) if (p.has_color) {
    const auto* cell = restored.find_at(p); REQUIRE(cell && cell->samples[0].rgb == p.rgb); ++colors;
  }
  REQUIRE(colors > 0);
  std::ifstream file(temp.path/"map.ply");
  const std::string ply((std::istreambuf_iterator<char>(file)),{});
  REQUIRE(ply.find("property uchar red\n") != std::string::npos);
}
}
int main() {
  try { projection_guards(); color_delays_candidates_only(); engine_preserves_rgb(); }
  catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
  std::cout<<"Camera projection, clock/calibration guards, color evidence and RGB export passed\n";
}
