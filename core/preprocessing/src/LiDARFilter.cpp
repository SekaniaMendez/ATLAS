#include "LiDARFilter.hpp"

#include <algorithm>
#include <cmath>

namespace atlas::preprocessing {

LiDARFilter::LiDARFilter(const float minimum_distance_m, const bool filter_mid360_low_confidence_tags)
    : minimum_distance_squared_(std::max(0.0F, minimum_distance_m) *
                                std::max(0.0F, minimum_distance_m)),
      filter_mid360_low_confidence_tags_(filter_mid360_low_confidence_tags) {}

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
    // MID-360 protocol, Tag Information: three two-bit confidence fields
    // (adhesion, small particles, other). 0=high, 1=medium, 2=low, 3=reserved.
    // Bits 6-7 are reserved, not a return-number/confidence field.
    const bool low_confidence = (point.tag & 0x03U) == 0x02U ||
                               ((point.tag >> 2U) & 0x03U) == 0x02U ||
                               ((point.tag >> 4U) & 0x03U) == 0x02U;
    if (filter_mid360_low_confidence_tags_ && low_confidence) {
      ++result.rejected_points;
      ++result.rejected_by_tag;
      continue;
    }
    result.frame.points.push_back(point);
  }
  return result;
}

}  // namespace atlas::preprocessing
