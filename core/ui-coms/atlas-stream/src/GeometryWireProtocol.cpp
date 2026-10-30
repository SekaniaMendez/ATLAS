#include <atlas/ui/GeometryWireProtocol.hpp>
#include <algorithm>
#include <bit>
#include <cmath>
#include <set>
#include <stdexcept>
#include <type_traits>

namespace atlas::ui::wire {
namespace {
template <typename T> void integer(std::vector<std::byte>& out, T value) {
  using U = std::make_unsigned_t<T>;
  for (size_t i = 0; i < sizeof(T); ++i)
    out.push_back(std::byte((static_cast<U>(value) >> (8 * i)) & 255));
}
void number(std::vector<std::byte>& out, float value) { integer(out, std::bit_cast<uint32_t>(value)); }
void validate(const geometry::Surface& s) {
  const auto kind = static_cast<uint32_t>(s.kind);
  const auto finite = [](float x) { return std::isfinite(x); };
  if (!s.id || kind < 1 || kind > 4 || !std::isfinite(s.area_m2) || s.area_m2 <= 0 ||
      !std::isfinite(s.rmse_m) || s.rmse_m < 0 || s.vertices.size() < 3 || s.support_count < 3 ||
      s.indices.empty() || s.indices.size() % 3 ||
      !std::all_of(s.origin.begin(), s.origin.end(), finite) ||
      !std::all_of(s.axes.begin(), s.axes.end(), finite))
    throw std::invalid_argument("Invalid live surface metadata");
  for (size_t a = 0; a < 3; ++a)
    for (size_t b = 0; b < 3; ++b) {
      float dot = 0;
      for (size_t k = 0; k < 3; ++k) dot += s.axes[3*a+k] * s.axes[3*b+k];
      if (std::abs(dot - (a == b ? 1.F : 0.F)) > .002F)
        throw std::invalid_argument("Non-orthonormal live surface frame");
    }
  const auto& r = s.axes;
  const float determinant = r[0]*(r[4]*r[8]-r[5]*r[7]) - r[3]*(r[1]*r[8]-r[2]*r[7]) +
                            r[6]*(r[1]*r[5]-r[2]*r[4]);
  if (determinant < .998F) throw std::invalid_argument("Left-handed live surface frame");
  for (const auto& v : s.vertices)
    if (!std::all_of(v.begin(), v.end(), finite)) throw std::invalid_argument("Invalid live vertex");
  for (auto i : s.indices)
    if (i >= s.vertices.size()) throw std::invalid_argument("Invalid live triangle index");
  for (size_t i = 0; i < s.indices.size(); i += 3) {
    const auto& a = s.vertices[s.indices[i]];
    const auto& b = s.vertices[s.indices[i+1]];
    const auto& c = s.vertices[s.indices[i+2]];
    const double ux = double(b[0])-a[0], uy = double(b[1])-a[1], uz = double(b[2])-a[2];
    const double vx = double(c[0])-a[0], vy = double(c[1])-a[1], vz = double(c[2])-a[2];
    const double x = uy*vz-uz*vy, y = uz*vx-ux*vz, z = ux*vy-uy*vx;
    if (x*x+y*y+z*z <= 1e-16) throw std::invalid_argument("Degenerate live triangle");
  }
}
}  // namespace

std::vector<std::byte> encode_geometry_snapshot(const geometry::GeometrySnapshot& snapshot,
    uint32_t sequence, uint64_t revision, float processing_ms) {
  if (snapshot.surfaces.size() > kGeometryMaximumSurfaces || !std::isfinite(processing_ms) || processing_ms < 0)
    throw std::invalid_argument("Invalid geometry snapshot limits");
  size_t vertices = 0, indices = 0;
  std::set<uint32_t> ids;
  for (const auto& s : snapshot.surfaces) {
    validate(s);
    if (!ids.insert(s.id).second) throw std::invalid_argument("Duplicate live surface ID");
    if (s.vertices.size() > kGeometryMaximumVertices - vertices ||
        s.indices.size() > kGeometryMaximumIndices - indices)
      throw std::invalid_argument("Live geometry exceeds wire limits");
    vertices += s.vertices.size(); indices += s.indices.size();
  }
  const auto count = static_cast<uint32_t>(snapshot.surfaces.size());
  const auto payload = static_cast<uint32_t>(kGeometryPrefixSize + count*kGeometrySurfaceSize + vertices*12 + indices*4);
  std::vector<std::byte> out;
  out.reserve(kHeaderSize + payload);
  integer(out, kMagic); integer(out, kProtocolVersion); integer(out, uint16_t(MessageType::SensorStatus));
  integer(out, payload); integer(out, sequence); integer(out, snapshot.source_timestamp_ns);
  integer(out, count); integer(out, kSensorStatusGeometry);
  integer(out, kGeometryVersion); integer(out, uint16_t(1 | (snapshot.truncated ? 2 : 0)));
  integer(out, count); integer(out, uint32_t(vertices)); integer(out, uint32_t(indices));
  integer(out, revision); integer(out, snapshot.input_point_count); number(out, processing_ms);
  uint32_t first_vertex = 0, first_index = 0;
  for (const auto& s : snapshot.surfaces) {
    integer(out, s.id); integer(out, static_cast<uint32_t>(s.kind));
    for (float v : s.origin) number(out, v);
    for (float v : s.axes) number(out, v);
    number(out, s.area_m2); number(out, s.rmse_m); integer(out, s.support_count);
    integer(out, first_vertex); integer(out, uint32_t(s.vertices.size()));
    integer(out, first_index); integer(out, uint32_t(s.indices.size()));
    first_vertex += static_cast<uint32_t>(s.vertices.size());
    first_index += static_cast<uint32_t>(s.indices.size());
  }
  for (const auto& s : snapshot.surfaces) for (const auto& v : s.vertices) for (float x : v) number(out, x);
  for (const auto& s : snapshot.surfaces) for (auto index : s.indices) integer(out, index);
  return out;
}
}  // namespace atlas::ui::wire
