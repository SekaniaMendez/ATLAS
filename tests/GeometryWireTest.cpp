#include <atlas/ui/GeometryWireProtocol.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace atlas;
namespace {
void check(bool ok) { if (!ok) throw std::runtime_error("Geometry wire regression"); }
template <class T> T integer(const std::vector<std::byte>& data, size_t offset) {
  T result = 0;
  for (size_t i = 0; i < sizeof(T); ++i) result |= T(std::to_integer<unsigned>(data.at(offset+i))) << (i*8);
  return result;
}
geometry::GeometrySnapshot fixture() {
  geometry::Surface s;
  s.id = 7; s.kind = geometry::SurfaceKind::Vertical;
  s.origin = {2,-3,1}; s.axes = {1,0,0, 0,0,1, 0,-1,0};
  s.area_m2 = 4; s.rmse_m = .006F; s.support_count = 100;
  s.vertices = {{1,-3,0},{3,-3,0},{3,-3,2},{1,-3,2}};
  s.indices = {0,1,2,0,2,3};
  geometry::GeometrySnapshot result;
  result.source_timestamp_ns = 123456789; result.input_point_count = 100;
  result.surfaces = {s}; return result;
}
template <class F> void rejects(F change) {
  auto sample = fixture(); change(sample);
  bool failed = false;
  try { (void)ui::wire::encode_geometry_snapshot(sample, 42, 9, 12.5F); }
  catch (const std::invalid_argument&) { failed = true; }
  check(failed);
}
}
int main(int argc, char** argv) {
  auto sample = fixture();
  const auto bytes = ui::wire::encode_geometry_snapshot(sample, 42, 9, 12.5F);
  check(bytes.size() == 32+32+84+4*12+6*4);
  check(integer<uint32_t>(bytes,0) == ui::wire::kMagic);
  check(integer<uint16_t>(bytes,6) == 2 && integer<uint32_t>(bytes,28) == 2);
  check(integer<uint32_t>(bytes,8) == bytes.size()-32);
  check(integer<uint32_t>(bytes,12) == 42 && integer<uint64_t>(bytes,16) == 123456789);
  check(integer<uint32_t>(bytes,24) == 1);
  check(integer<uint16_t>(bytes,32) == 1 && integer<uint16_t>(bytes,34) == 1);
  check(integer<uint32_t>(bytes,36) == 1 && integer<uint32_t>(bytes,40) == 4);
  check(integer<uint32_t>(bytes,44) == 6 && integer<uint64_t>(bytes,48) == 9);
  check(integer<uint32_t>(bytes,64) == 7 && integer<uint32_t>(bytes,68) == 2);
  check(integer<uint32_t>(bytes,64+68) == 0 && integer<uint32_t>(bytes,64+72) == 4);
  check(integer<uint32_t>(bytes,64+76) == 0 && integer<uint32_t>(bytes,64+80) == 6);
  check(integer<uint32_t>(bytes,bytes.size()-4) == 3);
  auto second = sample.surfaces.front(); second.id=8; sample.surfaces.push_back(second);
  const auto two = ui::wire::encode_geometry_snapshot(sample,43,10,5);
  check(integer<uint32_t>(two,64+84+68) == 4 && integer<uint32_t>(two,64+84+76) == 6);
  const auto empty = ui::wire::encode_geometry_snapshot({},44,11,0);
  check(empty.size() == 64 && integer<uint32_t>(empty,24) == 0 && integer<uint16_t>(empty,34) == 1);
  rejects([](auto& x) { x.surfaces[0].indices[0]=4; });
  rejects([](auto& x) { x.surfaces[0].indices[0]=1; });
  rejects([](auto& x) { x.surfaces[0].axes[8]=1; });
  rejects([](auto& x) { x.surfaces[0].vertices[0][0]=std::numeric_limits<float>::quiet_NaN(); });
  rejects([](auto& x) { x.surfaces[0].vertices.resize(30'001); });
  rejects([](auto& x) { x.surfaces.push_back(x.surfaces.front()); });
  rejects([](auto& x) { x.surfaces[0].indices.pop_back(); });
  if (argc == 2) {
    std::ofstream stream(argv[1], std::ios::binary);
    stream.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    check(stream.good());
  }
  std::cout << "Live geometry wire layout and validation passed\n";
}
