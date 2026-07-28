#include <algorithm>
#include <atlas/ui/WireProtocol.hpp>
#include <bit>
#include <limits>
#include <type_traits>

namespace atlas::ui::wire {
namespace {

template <typename Integer>
void append_integer(std::vector<std::byte>& output, Integer value) {
  static_assert(std::is_integral_v<Integer>);
  using Unsigned = std::make_unsigned_t<Integer>;
  const auto unsigned_value = static_cast<Unsigned>(value);
  for (size_t index = 0; index < sizeof(Integer); ++index) {
    output.push_back(static_cast<std::byte>((unsigned_value >> (index * 8U)) & 0xFFU));
  }
}

void append_float(std::vector<std::byte>& output, const float value) {
  append_integer(output, std::bit_cast<uint32_t>(value));
}

void append_header(std::vector<std::byte>& output, const MessageHeader& header) {
  append_integer(output, kMagic);
  append_integer(output, header.version);
  append_integer(output, static_cast<uint16_t>(header.type));
  append_integer(output, header.payload_size);
  append_integer(output, header.sequence);
  append_integer(output, header.timestamp_ns);
  append_integer(output, header.element_count);
  append_integer(output, header.flags);
}

}  // namespace

std::vector<std::byte> encode_point_cloud(const std::span<const atlas::sensors::LidarPoint> points,
                                          const uint32_t sequence, const uint64_t timestamp_ns) {
  const auto capped_count = std::min<size_t>(points.size(), std::numeric_limits<uint32_t>::max());
  const auto payload_size = capped_count * kPointSize;

  std::vector<std::byte> output;
  output.reserve(kHeaderSize + payload_size);
  append_header(output, {
                            .type = MessageType::PointCloud,
                            .payload_size = static_cast<uint32_t>(payload_size),
                            .sequence = sequence,
                            .timestamp_ns = timestamp_ns,
                            .element_count = static_cast<uint32_t>(capped_count),
                        });

  for (const auto& point : points.first(capped_count)) {
    append_float(output, point.x_m);
    append_float(output, point.y_m);
    append_float(output, point.z_m);
    output.push_back(static_cast<std::byte>(point.reflectivity));
    output.push_back(static_cast<std::byte>(point.tag));
    append_integer<uint16_t>(output, 0);
  }
  return output;
}

}  // namespace atlas::ui::wire
