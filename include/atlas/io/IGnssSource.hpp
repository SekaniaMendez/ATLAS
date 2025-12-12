#pragma once
#include <cstdint>
#include <optional>
#include <string>

namespace atlas::io {

enum class GnssFixType : uint8_t { None = 0, Single, DGNSS, FloatRTK, FixedRTK, PPP };

struct GnssFix {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m = 0.0;
  GnssFixType type = GnssFixType::None;

  // Optional extras you can fill later
  double hdop = 0.0;
  uint8_t sats = 0;
  uint64_t unix_time_ms = 0;  // if you parse ZDA or similar later
};

class IGnssSource {
 public:
  virtual ~IGnssSource() = default;

  // Start streaming/reading GNSS data
  virtual bool start() = 0;

  // Stop reading
  virtual void stop() = 0;

  // Latest fix if available
  virtual std::optional<GnssFix> latest_fix() const = 0;

  // For logging/debug
  virtual std::string name() const = 0;
};

}  // namespace atlas::io