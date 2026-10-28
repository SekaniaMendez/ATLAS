#include "LiDARFilter.hpp"

#include <algorithm>
#include <cmath>

namespace atlas::preprocessing {

LiDARFilter::LiDARFilter(const float minimum_distance_m)
    : minimum_distance_squared_(std::max(0.0F, minimum_distance_m) *
                                std::max(0.0F, minimum_distance_m)) {}

LiDARFilterResult LiDARFilter::filter(const atlas::sensors::LidarFrame& input) const {
  LiDARFilterResult result;
  result.frame.sensor_id = input.sensor_id;
  result.frame.packet_sequence = input.packet_sequence;
  result.frame.frame_index = input.frame_index;
  result.frame.timestamp = input.timestamp;
  result.frame.duration_ns = input.duration_ns;
  result.frame.points.reserve(input.points.size());

  for (const auto& point : input.points) {
    const float distance_squared =
        point.x_m * point.x_m + point.y_m * point.y_m + point.z_m * point.z_m;
    if (!std::isfinite(distance_squared) || distance_squared < minimum_distance_squared_) {
      ++result.rejected_points;
      continue;
    }
    result.frame.points.push_back(point);
  }
  return result;
}

}  // namespace atlas::preprocessing
