#include <atlas/geometry/LiveGeometry.hpp>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <utility>

namespace atlas::geometry {
namespace {
using Point = Eigen::Vector3d;
using Key = std::pair<int64_t, int64_t>;
constexpr double kPi = 3.14159265358979323846;

struct Plane {
  Point origin = Point::Zero();
  Point normal = Point::UnitZ();
  Eigen::Matrix3d axes = Eigen::Matrix3d::Identity();
};
struct UVPoint {
  double x = 0, y = 0;
  size_t index = 0;
};
struct Cell {
  std::vector<UVPoint> points;
  std::vector<UVPoint> hull;
};
using Cells = std::map<Key, Cell>;

void validate(const LiveGeometryConfig& c) {
  if (c.maximum_input_points < 3 || c.maximum_input_points > 50'000 ||
      !c.maximum_surfaces || c.maximum_surfaces > 32 ||
      c.maximum_vertices < 3 || c.maximum_vertices > 30'000 ||
      c.maximum_indices < 3 || c.maximum_indices > 90'000 ||
      !c.maximum_plane_hypotheses || c.maximum_plane_hypotheses > 64 ||
      !c.ransac_iterations || c.ransac_iterations > 1024 || c.minimum_support_points < 3 ||
      !std::isfinite(c.distance_threshold_m) || c.distance_threshold_m <= 0 ||
      !std::isfinite(c.surface_cell_size_m) || c.surface_cell_size_m <= 0 ||
      !std::isfinite(c.minimum_surface_area_m2) || c.minimum_surface_area_m2 <= 0 ||
      !std::isfinite(c.orientation_tolerance_degrees) ||
      c.orientation_tolerance_degrees <= 0 || c.orientation_tolerance_degrees >= 45) {
    throw std::invalid_argument("Invalid bounded live geometry configuration");
  }
}

Point canonical(Point normal) {
  normal.normalize();
  Eigen::Index dominant = 0;
  normal.cwiseAbs().maxCoeff(&dominant);
  if (normal[dominant] < 0) normal = -normal;
  return normal;
}

Eigen::Matrix3d axes_for(const Point& normal) {
  Point x;
  if (std::abs(normal.z()) < .9) x = Point::UnitZ().cross(normal);
  else x = Point::UnitX() - normal * normal.x();
  x.normalize();
  Eigen::Matrix3d axes;
  axes.col(0) = x;
  axes.col(1) = normal.cross(x);
  axes.col(2) = normal;
  return axes;
}

bool fit(const std::vector<Point>& points, const std::vector<size_t>& indices, Plane& plane) {
  if (indices.size() < 3) return false;
  plane.origin.setZero();
  for (const size_t i : indices) plane.origin += points[i];
  plane.origin /= static_cast<double>(indices.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const size_t i : indices) {
    const Point delta = points[i] - plane.origin;
    covariance.noalias() += delta * delta.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success || solver.eigenvalues()[1] < 1e-8) return false;
  plane.normal = canonical(solver.eigenvectors().col(0));
  plane.axes = axes_for(plane.normal);
  return plane.origin.allFinite() && plane.normal.allFinite();
}

uint64_t random_next(uint64_t& state) {
  state ^= state >> 12;
  state ^= state << 25;
  state ^= state >> 27;
  return state * 2685821657736338717ULL;
}

std::vector<size_t> ransac(const std::vector<Point>& points,
                           const std::vector<size_t>& available,
                           const LiveGeometryConfig& c, uint64_t& random) {
  size_t best_count = 0;
  double best_error = std::numeric_limits<double>::infinity();
  Point best_normal = Point::Zero(), best_origin = Point::Zero();
  for (size_t trial = 0; trial < c.ransac_iterations; ++trial) {
    const size_t a = available[random_next(random) % available.size()];
    const size_t b = available[random_next(random) % available.size()];
    const size_t d = available[random_next(random) % available.size()];
    Point normal = (points[b] - points[a]).cross(points[d] - points[a]);
    if (normal.squaredNorm() < 1e-10) continue;
    normal.normalize();
    size_t count = 0;
    double error = 0;
    for (const size_t i : available) {
      const double distance = std::abs(normal.dot(points[i] - points[a]));
      if (distance <= c.distance_threshold_m) { ++count; error += distance; }
    }
    if (count > best_count || (count == best_count && error < best_error)) {
      best_count = count; best_error = error;
      best_normal = normal; best_origin = points[a];
    }
  }
  std::vector<size_t> selected;
  if (best_count < c.minimum_support_points) return selected;
  selected.reserve(best_count);
  for (const size_t i : available)
    if (std::abs(best_normal.dot(points[i] - best_origin)) <= c.distance_threshold_m)
      selected.push_back(i);
  return selected;
}

double cross2(const UVPoint& a, const UVPoint& b, const UVPoint& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

std::vector<UVPoint> local_hull(std::vector<UVPoint> points) {
  std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  });
  points.erase(std::unique(points.begin(), points.end(), [](const auto& a, const auto& b) {
    return a.x == b.x && a.y == b.y;
  }), points.end());
  if (points.size() < 3) return {};
  std::vector<UVPoint> hull;
  for (const auto& p : points) {
    while (hull.size() >= 2 && cross2(hull[hull.size() - 2], hull.back(), p) <= 1e-10)
      hull.pop_back();
    hull.push_back(p);
  }
  const size_t lower = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (hull.size() > lower && cross2(hull[hull.size() - 2], hull.back(), *it) <= 1e-10)
      hull.pop_back();
    hull.push_back(*it);
  }
  if (!hull.empty()) hull.pop_back();
  return hull.size() >= 3 ? hull : std::vector<UVPoint>{};
}

Cells cells_for(const std::vector<Point>& points, const std::vector<size_t>& indices,
                const Plane& plane, double size) {
  Cells cells;
  for (const size_t i : indices) {
    const Point offset = points[i] - plane.origin;
    const double x = offset.dot(plane.axes.col(0)), y = offset.dot(plane.axes.col(1));
    if (std::abs(x / size) > 1e12 || std::abs(y / size) > 1e12) continue;
    cells[{static_cast<int64_t>(std::floor(x / size)),
           static_cast<int64_t>(std::floor(y / size))}].points.push_back({x, y, i});
  }
  for (auto it = cells.begin(); it != cells.end();) {
    it->second.hull = local_hull(it->second.points);
    if (it->second.hull.empty()) it = cells.erase(it);
    else ++it;
  }
  return cells;
}

std::vector<std::vector<size_t>> connected_support(const Cells& cells) {
  std::set<Key> visited;
  std::vector<std::vector<size_t>> groups;
  for (const auto& [key, _] : cells) {
    if (!visited.insert(key).second) continue;
    std::deque<Key> queue{key};
    std::vector<size_t> indices;
    while (!queue.empty()) {
      const Key current = queue.front(); queue.pop_front();
      for (const auto& point : cells.at(current).points) indices.push_back(point.index);
      for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) {
        if (!dx && !dy) continue;
        const Key neighbor{current.first + dx, current.second + dy};
        if (cells.contains(neighbor) && visited.insert(neighbor).second) queue.push_back(neighbor);
      }
    }
    groups.push_back(std::move(indices));
  }
  return groups;
}

std::array<float, 3> packed(const Point& p) {
  return {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())};
}

Surface make_surface(const std::vector<Point>& points, std::vector<size_t> indices,
                      const LiveGeometryConfig& config) {
  Surface surface;
  Plane plane;
  for (int iteration = 0; iteration < 4; ++iteration) {
    if (indices.size() < config.minimum_support_points || !fit(points, indices, plane)) return surface;
    const auto previous_size = indices.size();
    std::erase_if(indices, [&](size_t i) {
      return std::abs(plane.normal.dot(points[i] - plane.origin)) > config.distance_threshold_m;
    });
    if (indices.size() == previous_size) break;
    if (iteration == 3) return surface;
  }
  const auto cells = cells_for(points, indices, plane, config.surface_cell_size_m);
  double area = 0, squared_residual = 0;
  size_t supported = 0;
  for (const auto& [_, cell] : cells) {
    const uint32_t base = static_cast<uint32_t>(surface.vertices.size());
    for (const auto& p : cell.hull)
      surface.vertices.push_back(packed(plane.origin + plane.axes.col(0) * p.x + plane.axes.col(1) * p.y));
    for (size_t i = 1; i + 1 < cell.hull.size(); ++i) {
      const double triangle_area = cross2(cell.hull[0], cell.hull[i], cell.hull[i + 1]) * .5;
      if (triangle_area <= 1e-9) continue;
      area += triangle_area;
      surface.indices.insert(surface.indices.end(), {base, base + static_cast<uint32_t>(i),
                                                     base + static_cast<uint32_t>(i + 1)});
    }
    for (const auto& p : cell.points) {
      const double distance = plane.normal.dot(points[p.index] - plane.origin);
      squared_residual += distance * distance;
      ++supported;
    }
  }
  if (supported < config.minimum_support_points || area < config.minimum_surface_area_m2 ||
      surface.indices.empty()) return {};
  surface.origin = packed(plane.origin);
  for (size_t col = 0; col < 3; ++col) for (size_t row = 0; row < 3; ++row)
    surface.axes[col * 3 + row] = static_cast<float>(plane.axes(static_cast<int>(row), static_cast<int>(col)));
  const double vertical = std::abs(plane.normal.z());
  const double angle = config.orientation_tolerance_degrees * kPi / 180;
  surface.kind = vertical >= std::cos(angle) ? SurfaceKind::Horizontal
      : vertical <= std::sin(angle) ? SurfaceKind::Vertical : SurfaceKind::Inclined;
  surface.area_m2 = static_cast<float>(area);
  surface.rmse_m = static_cast<float>(std::sqrt(squared_residual / static_cast<double>(supported)));
  surface.support_count = static_cast<uint32_t>(supported);
  return surface;
}
}  // namespace

GeometrySnapshot extract_geometry(std::span<const sensors::LidarPoint> input, uint64_t timestamp_ns,
                                   const LiveGeometryConfig& config) {
  validate(config);
  GeometrySnapshot result;
  result.source_timestamp_ns = timestamp_ns;
  result.input_point_count = static_cast<uint32_t>(std::min(input.size(), static_cast<size_t>(UINT32_MAX)));
  result.truncated = input.size() > config.maximum_input_points;
  const size_t take = std::min(input.size(), config.maximum_input_points);
  std::vector<Point> points;
  points.reserve(take);
  for (size_t i = 0; i < take; ++i) {
    const auto& source = input[i * input.size() / take];
    const Point p(source.x_m, source.y_m, source.z_m);
    if (p.allFinite() && p.cwiseAbs().maxCoeff() <= 1e6) points.push_back(p);
  }
  std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
    if (a.x() != b.x()) return a.x() < b.x();
    if (a.y() != b.y()) return a.y() < b.y();
    return a.z() < b.z();
  });
  points.erase(std::unique(points.begin(), points.end(), [](const Point& a, const Point& b) {
    return a == b;
  }), points.end());
  std::vector<size_t> available(points.size());
  std::iota(available.begin(), available.end(), 0);
  uint64_t random = config.random_seed ? config.random_seed : 0x9e3779b97f4a7c15ULL;
  for (size_t hypothesis = 0; hypothesis < config.maximum_plane_hypotheses; ++hypothesis) {
    if (available.size() < config.minimum_support_points) break;
    auto candidate = ransac(points, available, config, random);
    if (candidate.empty()) break;
    std::vector<bool> selected(points.size(), false);
    for (const size_t i : candidate) selected[i] = true;
    std::erase_if(available, [&](size_t i) { return selected[i]; });
    Plane plane;
    if (!fit(points, candidate, plane)) continue;
    std::erase_if(candidate, [&](size_t i) {
      return std::abs(plane.normal.dot(points[i] - plane.origin)) > config.distance_threshold_m;
    });
    const auto cells = cells_for(points, candidate, plane, config.surface_cell_size_m);
    for (auto& group : connected_support(cells)) {
      if (group.size() < config.minimum_support_points) continue;
      auto surface = make_surface(points, std::move(group), config);
      if (!surface.indices.empty()) result.surfaces.push_back(std::move(surface));
    }
  }
  std::sort(result.surfaces.begin(), result.surfaces.end(), [](const Surface& a, const Surface& b) {
    if (a.area_m2 != b.area_m2) return a.area_m2 > b.area_m2;
    return a.origin < b.origin;
  });
  size_t vertices = 0, indices = 0, accepted = 0;
  for (auto& surface : result.surfaces) {
    if (accepted == config.maximum_surfaces || vertices + surface.vertices.size() > config.maximum_vertices ||
        indices + surface.indices.size() > config.maximum_indices) {
      result.truncated = true;
      continue;
    }
    vertices += surface.vertices.size(); indices += surface.indices.size();
    surface.id = static_cast<uint32_t>(accepted + 1);
    if (&surface != &result.surfaces[accepted]) result.surfaces[accepted] = std::move(surface);
    ++accepted;
  }
  result.surfaces.resize(accepted);
  return result;
}
}  // namespace atlas::geometry
