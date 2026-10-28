#pragma once
#include "BinaryIO.hpp"
#include <atlas/storage/Capture.hpp>

namespace atlas::storage {
binary::Encoder encode_event(const CaptureEvent& event);
CaptureEvent decode_event(std::span<const uint8_t> bytes);
}
