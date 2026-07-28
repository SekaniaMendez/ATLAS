#pragma once

#include <cstdint>

namespace atlas::timesync {

enum class ClockSource : uint8_t {
  Unsynchronized = 0,
  Ptp = 1,
  Gps = 2,
  Unknown = 255,
};

struct SensorTimestamp {
  uint64_t nanoseconds = 0;
  ClockSource source = ClockSource::Unknown;

  [[nodiscard]] bool globally_synchronized() const noexcept {
    return source == ClockSource::Ptp || source == ClockSource::Gps;
  }
};

}  // namespace atlas::timesync
