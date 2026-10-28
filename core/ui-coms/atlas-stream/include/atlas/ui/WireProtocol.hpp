#pragma once

#include <atlas/sensors/LidarFrame.hpp>
#include <atlas/sensors/SensorWarning.hpp>
#include <atlas/ui/wire_protocol_config.hpp>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace atlas::ui::wire {

inline constexpr uint32_t kMagic = 0x534C5441U;  // "ATLS" in little-endian bytes.
inline constexpr uint16_t kProtocolVersion = 1;
inline constexpr size_t kHeaderSize = 32;
inline constexpr size_t kPointSize = 16;
inline constexpr size_t kSensorWarningSize = 12;
/// PointCloud flags only; other message types have their own flag semantics.
inline constexpr uint32_t kPointCloudWorldFrame = 1U << 0U;
/// SensorStatus is a reserved, ignored type in legacy viewers. A subtype keeps
/// SLAM telemetry compatible with those clients without using warning banners.
inline constexpr uint32_t kSensorStatusSlam = 1;
inline constexpr uint16_t kSlamStatusVersion = 1;
inline constexpr size_t kSlamStatusFixedSize = 24;

enum class SlamTrackingState : uint8_t { Uninitialized = 0, Initializing = 1, Tracking = 2, Lost = 3 };

struct SlamStatus {
  uint64_t timestamp_ns = 0;
  SlamTrackingState state = SlamTrackingState::Uninitialized;
  bool fatal_error = false;
  uint64_t scans_accepted = 0;
  uint64_t scans_rejected = 0;
  std::string message;
};

enum class MessageType : uint16_t {
  Hello = 1,
  SensorStatus = 2,
  SensorWarning = 3,
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
    std::span<const atlas::sensors::LidarPoint> points, uint32_t sequence, uint64_t timestamp_ns,
    uint32_t flags = 0);

[[nodiscard]] std::vector<std::byte> encode_sensor_warning(
    const atlas::sensors::SensorWarning& warning, uint32_t sequence);

[[nodiscard]] std::vector<std::byte> encode_slam_status(const SlamStatus& status, uint32_t sequence);

}  // namespace atlas::ui::wire
