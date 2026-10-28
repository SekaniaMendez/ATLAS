#pragma once

#include <array>

namespace atlas::slam {

struct MapPoint;

/// Half-open metric box: minimum included, maximum excluded.
struct MapBounds {
  double min_x_m = 0.0;
  double min_y_m = 0.0;
  double min_z_m = 0.0;
  double max_x_m = 0.0;
  double max_y_m = 0.0;
  double max_z_m = 0.0;
  [[nodiscard]] bool is_valid() const noexcept;
  [[nodiscard]] bool contains(const MapPoint& point) const noexcept;
  [[nodiscard]] std::array<double, 3> dimensions_m() const noexcept;
};

}  // namespace atlas::slam
