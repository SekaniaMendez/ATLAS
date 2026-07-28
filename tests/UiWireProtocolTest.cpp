#include <atlas/sensors/LidarFrame.hpp>
#include <atlas/ui/WireProtocol.hpp>
#include <bit>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>
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
  assert(read_little_endian<uint32_t>(message, 32) == std::bit_cast<uint32_t>(1.25F));
  assert(std::to_integer<uint8_t>(message[44]) == 42);
  assert(std::to_integer<uint8_t>(message[45]) == 8);
  return 0;
}
