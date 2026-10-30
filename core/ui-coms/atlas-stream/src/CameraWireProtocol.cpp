#include <atlas/ui/CameraWireProtocol.hpp>
#include <bit>
#include <stdexcept>

namespace atlas::ui::camera_wire {
namespace {
template<class T> T read(std::span<const std::byte> bytes, size_t offset) {
  T result = 0;
  for (size_t i = 0; i < sizeof(T); ++i) result |= T(std::to_integer<uint8_t>(bytes[offset+i])) << (8*i);
  return result;
}
template<class T> void append(std::vector<std::byte>& out, T value) {
  for (size_t i = 0; i < sizeof(T); ++i) out.push_back(std::byte((value >> (8*i)) & 255));
}
constexpr uint32_t magic = 0x52435441;
}
std::optional<uint32_t> payload_size(std::span<const std::byte> header) {
  if (header.size() != header_size || read<uint32_t>(header, 0) != magic ||
      read<uint32_t>(header, 4) != 1) return {};
  const auto size = read<uint32_t>(header, 8);
  if (size < metadata_size || size > maximum_payload) return {};
  return size;
}
std::optional<sensors::CameraRgbFrame> decode(std::span<const std::byte> payload) {
  if (payload.size() < metadata_size || payload.size() > maximum_payload) return {};
  sensors::CameraRgbFrame frame;
  frame.width = read<uint32_t>(payload, 0); frame.height = read<uint32_t>(payload, 4);
  const uint64_t pixels = uint64_t(frame.width) * frame.height;
  const auto flags = read<uint32_t>(payload, 8);
  if (!pixels || pixels > maximum_pixels || payload.size() != metadata_size + pixels * 3 || (flags & ~3u)) return {};
  frame.calibration_verified = flags & 1; frame.tracking_normal = flags & 2;
  frame.timestamp_ns = read<uint64_t>(payload, 12);
  size_t offset = 20;
  const auto values = [&](auto& array) {
    for (auto& v : array) { v = std::bit_cast<double>(read<uint64_t>(payload, offset)); offset += 8; }
  };
  values(frame.intrinsics); values(frame.camera_from_lidar_rotation); values(frame.camera_from_lidar_translation);
  frame.rgb.resize(pixels * 3);
  for (size_t i = 0; i < frame.rgb.size(); ++i) frame.rgb[i] = std::to_integer<uint8_t>(payload[offset+i]);
  return frame;
}
std::vector<std::byte> encode(const sensors::CameraRgbFrame& frame) {
  const uint64_t pixels = uint64_t(frame.width) * frame.height;
  if (!pixels || pixels > maximum_pixels || frame.rgb.size() != pixels * 3)
    throw std::invalid_argument("Invalid camera RGB dimensions");
  std::vector<std::byte> out;
  out.reserve(header_size + metadata_size + frame.rgb.size());
  append(out, magic); append(out, uint32_t(1)); append(out, uint32_t(metadata_size + frame.rgb.size()));
  append(out, frame.width); append(out, frame.height);
  append(out, uint32_t(frame.calibration_verified | (frame.tracking_normal << 1)));
  append(out, frame.timestamp_ns);
  for (double v : frame.intrinsics) append(out, std::bit_cast<uint64_t>(v));
  for (double v : frame.camera_from_lidar_rotation) append(out, std::bit_cast<uint64_t>(v));
  for (double v : frame.camera_from_lidar_translation) append(out, std::bit_cast<uint64_t>(v));
  for (auto v : frame.rgb) out.push_back(std::byte(v));
  return out;
}
}
