#pragma once

#include <atlas/sensors/LidarFrame.hpp>
#include <cstddef>

namespace atlas::preprocessing {

struct LiDARFilterResult {
  atlas::sensors::LidarFrame frame;
  size_t rejected_points = 0;
};

class LiDARFilter {
 public:
  explicit LiDARFilter(float minimum_distance_m);

  [[nodiscard]] LiDARFilterResult filter(const atlas::sensors::LidarFrame& input) const;

 private:
  float minimum_distance_squared_ = 0.0F;
};

}  // namespace atlas::preprocessing
