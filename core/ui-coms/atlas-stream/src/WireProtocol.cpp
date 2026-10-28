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
                                          const uint32_t sequence, const uint64_t timestamp_ns,
                                          const uint32_t flags) {
  const auto capped_count = std::min<size_t>(
      points.size(), std::numeric_limits<uint32_t>::max() / kPointSize);
  const auto payload_size = capped_count * kPointSize;

  std::vector<std::byte> output;
  output.reserve(kHeaderSize + payload_size);
  append_header(output, {
                            .type = MessageType::PointCloud,
                            .payload_size = static_cast<uint32_t>(payload_size),
                            .sequence = sequence,
                            .timestamp_ns = timestamp_ns,
                            .element_count = static_cast<uint32_t>(capped_count),
                            .flags = flags,
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

std::vector<std::byte> encode_sensor_warning(const atlas::sensors::SensorWarning& warning,
                                             const uint32_t sequence) {
  std::vector<std::byte> output;
  output.reserve(kHeaderSize + kSensorWarningSize);
  append_header(output, {
                            .type = MessageType::SensorWarning,
                            .payload_size = static_cast<uint32_t>(kSensorWarningSize),
                            .sequence = sequence,
                            .timestamp_ns = warning.timestamp.nanoseconds,
                            .element_count = 1,
                            .flags = warning.sensor_id,
                        });
  append_integer(output, static_cast<uint16_t>(warning.code));
  output.push_back(static_cast<std::byte>(static_cast<uint8_t>(warning.severity)));
  output.push_back(std::byte{0});
  append_float(output, warning.measured_value);
  append_float(output, warning.threshold);
  return output;
}

std::vector<std::byte> encode_slam_status(const SlamStatus& status, const uint32_t sequence) {
  size_t message_size = std::min(status.message.size(), kMaximumSlamStatusMessageBytes);
  // Do not cut a UTF-8 codepoint when limiting diagnostic text.
  while (message_size < status.message.size() && message_size != 0 &&
         (static_cast<unsigned char>(status.message[message_size]) & 0xC0U) == 0x80U)
    --message_size;
  std::vector<std::byte> output;
  output.reserve(kHeaderSize + kSlamStatusFixedSize + message_size);
  append_header(output, {.type = MessageType::SensorStatus,
                         .payload_size = static_cast<uint32_t>(kSlamStatusFixedSize + message_size),
                         .sequence = sequence,
                         .timestamp_ns = status.timestamp_ns,
                         .element_count = 1,
                         .flags = kSensorStatusSlam});
  append_integer(output, kSlamStatusVersion);
  append_integer(output, static_cast<uint8_t>(status.state));
  append_integer(output, static_cast<uint8_t>(status.fatal_error ? 1 : 0));
  append_integer(output, status.scans_accepted);
  append_integer(output, status.scans_rejected);
  append_integer(output, static_cast<uint32_t>(message_size));
  for (const unsigned char value : status.message.substr(0, message_size))
    output.push_back(static_cast<std::byte>(value));
  return output;
}

}  // namespace atlas::ui::wire
