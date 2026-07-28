#pragma once

#include <atlas/sensors/LidarFrame.hpp>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace atlas::ui::wire {

inline constexpr uint32_t kMagic = 0x534C5441U;  // "ATLS" in little-endian bytes.
inline constexpr uint16_t kProtocolVersion = 1;
inline constexpr size_t kHeaderSize = 32;
inline constexpr size_t kPointSize = 16;

enum class MessageType : uint16_t {
  Hello = 1,
  SensorStatus = 2,
  PointCloud = 10,
  SlamPose = 11,
  Control = 20,
  ArkitPose = 30,
};

struct MessageHeader {
  uint16_t version = kProtocolVersion;
  MessageType type = MessageType::Hello;
  uint32_t payload_size = 0;
  uint32_t sequence = 0;
  uint64_t timestamp_ns = 0;
  uint32_t element_count = 0;
  uint32_t flags = 0;
};

[[nodiscard]] std::vector<std::byte> encode_point_cloud(
    std::span<const atlas::sensors::LidarPoint> points, uint32_t sequence, uint64_t timestamp_ns);

}  // namespace atlas::ui::wire
