#pragma once

#include <atlas/sensors/LidarFrame.hpp>
#include <cstddef>

namespace atlas::preprocessing {

struct LiDARFilterResult {
  atlas::sensors::LidarFrame frame;
  size_t rejected_points = 0;
  size_t rejected_by_tag = 0;
};

class LiDARFilter {
 public:
  LiDARFilter(float minimum_distance_m, bool filter_mid360_low_confidence_tags);

  [[nodiscard]] LiDARFilterResult filter(const atlas::sensors::LidarFrame& input) const;

 private:
  float minimum_distance_squared_ = 0.0F;
  bool filter_mid360_low_confidence_tags_ = false;
};

}  // namespace atlas::preprocessing
