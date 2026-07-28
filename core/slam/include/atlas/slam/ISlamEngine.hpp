#pragma once

#include <atlas/sensors/ISensorDataSink.hpp>
#include <atlas/timesync/SensorTimestamp.hpp>
#include <optional>

namespace atlas::slam {

enum class TrackingState {
  Uninitialized,
  Initializing,
  Tracking,
  Lost,
};

struct PoseEstimate {
  atlas::timesync::SensorTimestamp timestamp;
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  TrackingState state = TrackingState::Uninitialized;
};

class ISlamEngine : public atlas::sensors::ISensorDataSink {
 public:
  ~ISlamEngine() override = default;

  virtual void reset() = 0;
  [[nodiscard]] virtual std::optional<PoseEstimate> latest_pose() const = 0;
};

}  // namespace atlas::slam
