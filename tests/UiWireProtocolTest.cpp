#include <atlas/sensors/LidarFrame.hpp>
#include <atlas/ui/WireProtocol.hpp>
#include <bit>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace {

template <typename Integer>
Integer read_little_endian(const std::vector<std::byte>& bytes, const size_t offset) {
  Integer value = 0;
  for (size_t index = 0; index < sizeof(Integer); ++index) {
    value |= static_cast<Integer>(std::to_integer<uint8_t>(bytes[offset + index]))
             << static_cast<Integer>(index * 8U);
  }
  return value;
}

}  // namespace

int main() {
  const std::vector<atlas::sensors::LidarPoint> points{
      {.x_m = 1.25F, .y_m = -2.5F, .z_m = 3.75F, .reflectivity = 42, .tag = 8},
      {.x_m = 4.0F, .y_m = 5.0F, .z_m = 6.0F, .reflectivity = 255, .tag = 0},
  };
  const auto message = atlas::ui::wire::encode_point_cloud(points, 7, 123456789ULL);

  assert(message.size() ==
         atlas::ui::wire::kHeaderSize + points.size() * atlas::ui::wire::kPointSize);
  assert(read_little_endian<uint32_t>(message, 0) == atlas::ui::wire::kMagic);
  assert(read_little_endian<uint16_t>(message, 4) == atlas::ui::wire::kProtocolVersion);
  assert(read_little_endian<uint16_t>(message, 6) ==
         static_cast<uint16_t>(atlas::ui::wire::MessageType::PointCloud));
  assert(read_little_endian<uint32_t>(message, 8) == points.size() * atlas::ui::wire::kPointSize);
  assert(read_little_endian<uint32_t>(message, 12) == 7);
  assert(read_little_endian<uint64_t>(message, 16) == 123456789ULL);
  assert(read_little_endian<uint32_t>(message, 24) == points.size());
  assert(read_little_endian<uint32_t>(message, 28) == 0);
  assert(read_little_endian<uint32_t>(message, 32) == std::bit_cast<uint32_t>(1.25F));
  assert(std::to_integer<uint8_t>(message[44]) == 42);
  assert(std::to_integer<uint8_t>(message[45]) == 8);

  const auto map_message = atlas::ui::wire::encode_point_cloud(
      points, 8, 123456789ULL, atlas::ui::wire::kPointCloudWorldFrame);
  assert(read_little_endian<uint32_t>(map_message, 28) == atlas::ui::wire::kPointCloudWorldFrame);
  assert(map_message.size() == message.size());
  const auto empty_map = atlas::ui::wire::encode_point_cloud(
      {}, 9, 0, atlas::ui::wire::kPointCloudWorldFrame);
  assert(empty_map.size() == atlas::ui::wire::kHeaderSize);
  assert(read_little_endian<uint32_t>(empty_map, 24) == 0);
  assert(read_little_endian<uint32_t>(empty_map, 28) == atlas::ui::wire::kPointCloudWorldFrame);

  const atlas::sensors::SensorWarning warning{
      .sensor_id = 23,
      .timestamp = {.nanoseconds = 987654321ULL},
      .code = atlas::sensors::SensorWarningCode::ExcessiveSpeed,
      .severity = atlas::sensors::SensorWarningSeverity::Caution,
      .measured_value = 2.75F,
      .threshold = 2.5F,
  };
  const auto warning_message = atlas::ui::wire::encode_sensor_warning(warning, 8);
  assert(warning_message.size() ==
         atlas::ui::wire::kHeaderSize + atlas::ui::wire::kSensorWarningSize);
  assert(read_little_endian<uint16_t>(warning_message, 6) ==
         static_cast<uint16_t>(atlas::ui::wire::MessageType::SensorWarning));
  assert(read_little_endian<uint32_t>(warning_message, 8) == atlas::ui::wire::kSensorWarningSize);
  assert(read_little_endian<uint32_t>(warning_message, 28) == 23);
  assert(read_little_endian<uint16_t>(warning_message, 32) ==
         static_cast<uint16_t>(atlas::sensors::SensorWarningCode::ExcessiveSpeed));
  assert(read_little_endian<uint32_t>(warning_message, 36) == std::bit_cast<uint32_t>(2.75F));
  assert(read_little_endian<uint32_t>(warning_message, 40) == std::bit_cast<uint32_t>(2.5F));

  const atlas::ui::wire::SlamStatus lost_status{
      .timestamp_ns = 321987654ULL,
      .state = atlas::ui::wire::SlamTrackingState::Lost,
      .fatal_error = false,
      .scans_accepted = 42,
      .scans_rejected = 3,
      .message = "Tracking gap too large; reset SLAM",
  };
  const auto status_message = atlas::ui::wire::encode_slam_status(lost_status, 10);
  assert(read_little_endian<uint16_t>(status_message, 6) ==
         static_cast<uint16_t>(atlas::ui::wire::MessageType::SensorStatus));
  assert(read_little_endian<uint32_t>(status_message, 28) == atlas::ui::wire::kSensorStatusSlam);
  assert(read_little_endian<uint64_t>(status_message, 16) == lost_status.timestamp_ns);
  assert(read_little_endian<uint16_t>(status_message, 32) == atlas::ui::wire::kSlamStatusVersion);
  assert(std::to_integer<uint8_t>(status_message[34]) == 3);
  assert(std::to_integer<uint8_t>(status_message[35]) == 0);
  assert(read_little_endian<uint64_t>(status_message, 36) == 42);
  assert(read_little_endian<uint64_t>(status_message, 44) == 3);
  assert(read_little_endian<uint32_t>(status_message, 52) == lost_status.message.size());
  assert(status_message.size() == 56 + lost_status.message.size());
  for (size_t i = 0; i < lost_status.message.size(); ++i)
    assert(std::to_integer<uint8_t>(status_message[56 + i]) == static_cast<uint8_t>(lost_status.message[i]));
  for (uint8_t state = 0; state <= 3; ++state) {
    auto status = lost_status;
    status.state = static_cast<atlas::ui::wire::SlamTrackingState>(state);
    status.fatal_error = true;
    status.message.clear();
    const auto encoded = atlas::ui::wire::encode_slam_status(status, 11);
    assert(encoded.size() == 56);
    assert(std::to_integer<uint8_t>(encoded[34]) == state);
    assert(std::to_integer<uint8_t>(encoded[35]) == 1);
  }
  auto long_status = lost_status;
  long_status.message = std::string(atlas::ui::wire::kMaximumSlamStatusMessageBytes - 1, 'x') + "á";
  const auto bounded_status = atlas::ui::wire::encode_slam_status(long_status, 12);
  assert(read_little_endian<uint32_t>(bounded_status, 52) == atlas::ui::wire::kMaximumSlamStatusMessageBytes - 1);
  return 0;
}
