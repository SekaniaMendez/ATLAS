#pragma once

#include <atlas/imu/ImuSample.hpp>
#include <atlas/sensors/LidarFrame.hpp>
#include <atlas/sensors/SensorStatus.hpp>

namespace atlas::sensors {

class ISensorDataSink {
 public:
  virtual ~ISensorDataSink() = default;

  virtual void on_lidar_frame(const LidarFrame& /*frame*/) {}
  virtual void on_imu_sample(const atlas::imu::ImuSample& /*sample*/) {}
  virtual void on_sensor_status(const SensorStatus& /*status*/) {}
};

}  // namespace atlas::sensors
