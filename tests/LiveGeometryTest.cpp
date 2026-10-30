#include <atlas/geometry/LiveGeometry.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace atlas;
using geometry::SurfaceKind;
void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
Eigen::Vector3d vector(const std::array<float, 3>& p) { return {p[0], p[1], p[2]}; }

std::vector<sensors::LidarPoint> wall(bool hole = false, double offset = 0) {
  std::vector<sensors::LidarPoint> result;
  for (int x = -40; x <= 40; ++x) for (int y = -40; y <= 40; ++y) {
    if (hole && std::abs(x) < 12 && std::abs(y) < 12) continue;
    result.push_back({.x_m = static_cast<float>(x * .04 + offset),
                      .y_m = static_cast<float>(y * .04), .z_m = .5F});
  }
  return result;
}

void validate_surface(const geometry::Surface& s) {
  Eigen::Matrix3d basis;
  for (int col = 0; col < 3; ++col) for (int row = 0; row < 3; ++row)
    basis(row, col) = s.axes[static_cast<size_t>(col * 3 + row)];
  check((basis.transpose() * basis - Eigen::Matrix3d::Identity()).norm() < 1e-5,
        "surface basis is not orthonormal");
  check(std::abs(basis.determinant() - 1) < 1e-5, "surface basis is not right handed");
  check(s.indices.size() % 3 == 0 && !s.indices.empty(), "invalid triangle index count");
  check(s.support_count >= 80 && s.area_m2 >= .12F && s.rmse_m <= .025F, "weak plane support");
  double area = 0;
  for (size_t i = 0; i < s.indices.size(); i += 3) {
    for (size_t j = 0; j < 3; ++j) check(s.indices[i + j] < s.vertices.size(), "out of range index");
    const auto a = vector(s.vertices[s.indices[i]]);
    const auto b = vector(s.vertices[s.indices[i + 1]]);
    const auto c = vector(s.vertices[s.indices[i + 2]]);
    const auto normal = (b - a).cross(c - a);
    check(normal.norm() > 1e-9, "degenerate triangle");
    check(normal.dot(basis.col(2)) > 0, "wrong winding");
    check(std::max({(a - b).norm(), (a - c).norm(), (b - c).norm()}) < .227,
          "triangle bridges more than one bounded UV cell");
    for (const auto& p : {a, b, c})
      check(std::abs((p - vector(s.origin)).dot(basis.col(2))) < 2e-5,
            "vertices are not map-frame points on the fitted plane");
    area += normal.norm() / 2;
  }
  check(std::abs(area - s.area_m2) < 1e-4, "wrong surface area");
}

void holes_and_replacement() {
  auto points = wall(true);
  const auto original = points;
  const auto snapshot = geometry::extract_geometry(points, 123456);
  check(snapshot.source_timestamp_ns == 123456, "source timestamp lost");
  check(snapshot.input_point_count == points.size(), "input count lost");
  check(snapshot.surfaces.size() == 1, "single connected wall unexpectedly split");
  for (const auto& surface : snapshot.surfaces) {
    validate_surface(surface);
    check(surface.kind == SurfaceKind::Horizontal, "horizontal orientation misclassified");
    for (size_t i = 0; i < surface.indices.size(); i += 3) {
      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      for (size_t j = 0; j < 3; ++j) center += vector(surface.vertices[surface.indices[i + j]]) / 3;
      check(std::abs(center.x()) > .35 || std::abs(center.y()) > .35, "filled an unobserved hole");
    }
  }
  check(points.front().x_m == original.front().x_m && points.back().y_m == original.back().y_m,
        "changed source points");
  check(geometry::extract_geometry({}, 123457).surfaces.empty(), "empty canvas retained stale geometry");
  std::erase_if(points, [](const auto& p) { return p.x_m > 0; });
  const auto reduced = geometry::extract_geometry(points, 123458);
  check(!reduced.surfaces.empty(), "retained wall vanished");
  for (const auto& s : reduced.surfaces) for (const auto& p : s.vertices)
    check(p[0] <= .00001F, "removed region remained in replacement geometry");
}

void disconnected_and_oblique() {
  auto points = wall(false, -3.5);
  auto other = wall(false, 3.5);
  points.insert(points.end(), other.begin(), other.end());
  auto snapshot = geometry::extract_geometry(points);
  check(snapshot.surfaces.size() == 2, "disconnected coplanar surfaces were merged");
  for (const auto& s : snapshot.surfaces) validate_surface(s);
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.61, Eigen::Vector3d::UnitX()).toRotationMatrix();
  for (auto& p : points) {
    const Eigen::Vector3d q = rotation * Eigen::Vector3d(p.x_m, p.y_m, p.z_m) + Eigen::Vector3d(2, -1, 4);
    p.x_m = static_cast<float>(q.x()); p.y_m = static_cast<float>(q.y()); p.z_m = static_cast<float>(q.z());
  }
  snapshot = geometry::extract_geometry(points);
  check(snapshot.surfaces.size() == 2, "oblique surfaces missing");
  for (const auto& s : snapshot.surfaces) {
    validate_surface(s);
    check(s.kind == SurfaceKind::Inclined, "oblique wall forced to Manhattan frame");
  }
  std::reverse(points.begin(), points.end());
  const auto reversed = geometry::extract_geometry(points);
  check(reversed.surfaces.size() == snapshot.surfaces.size(), "point order changed extraction");
  for (size_t i = 0; i < snapshot.surfaces.size(); ++i) {
    check(reversed.surfaces[i].vertices == snapshot.surfaces[i].vertices, "nondeterministic vertices");
    check(reversed.surfaces[i].indices == snapshot.surfaces[i].indices, "nondeterministic triangles");
  }
}

void invalid_and_limits() {
  std::vector<sensors::LidarPoint> sparse{{.x_m = 1}, {.y_m = 2}, {.z_m = 3}};
  sparse.push_back({.x_m = std::numeric_limits<float>::quiet_NaN()});
  sparse.push_back({.x_m = std::numeric_limits<float>::infinity()});
  check(geometry::extract_geometry(sparse).surfaces.empty(), "invented surface for isolated input");
  auto points = wall();
  auto other = wall(false, 4);
  points.insert(points.end(), other.begin(), other.end());
  geometry::LiveGeometryConfig config;
  config.maximum_surfaces = 1;
  const auto limited = geometry::extract_geometry(points, 0, config);
  check(limited.truncated && limited.surfaces.size() == 1, "surface limit not reported");
  config.maximum_input_points = 4000;
  const auto sampled = geometry::extract_geometry(points, 0, config);
  check(sampled.truncated && sampled.input_point_count == points.size(), "input limit not reported");
  config.surface_cell_size_m = -1;
  try { (void)geometry::extract_geometry(points, 0, config); }
  catch (const std::invalid_argument&) { return; }
  throw std::runtime_error("invalid configuration accepted");
}
}  // namespace

int main() {
  try {
    const auto start = std::chrono::steady_clock::now();
    holes_and_replacement();
    disconnected_and_oblique();
    invalid_and_limits();
    std::cout << "Live geometry tests passed in "
              << std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() << " s\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Live geometry test failed: " << error.what() << '\n';
    return 1;
  }
}
