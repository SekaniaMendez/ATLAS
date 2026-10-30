#pragma once
#include <atlas/sensors/CameraRgbFrame.hpp>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace atlas::ui::camera_wire {
inline constexpr size_t header_size = 12, metadata_size = 148;
inline constexpr uint32_t maximum_pixels = 640 * 480;
inline constexpr size_t maximum_payload = metadata_size + maximum_pixels * 3;
// ATCR, version 1, little endian. Separate uplink on the existing duplex socket.
std::optional<uint32_t> payload_size(std::span<const std::byte> header);
std::optional<sensors::CameraRgbFrame> decode(std::span<const std::byte> payload);
std::vector<std::byte> encode(const sensors::CameraRgbFrame& frame);
}
