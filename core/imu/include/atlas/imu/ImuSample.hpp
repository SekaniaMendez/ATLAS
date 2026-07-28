#pragma once

#include <atlas/timesync/SensorTimestamp.hpp>
#include <cstdint>

namespace atlas::imu {

struct ImuSample {
  uint32_t sensor_id = 0;
  atlas::timesync::SensorTimestamp timestamp;
  float gyro_x_rad_s = 0.0F;
  float gyro_y_rad_s = 0.0F;
  float gyro_z_rad_s = 0.0F;
  float accel_x_m_s2 = 0.0F;
  float accel_y_m_s2 = 0.0F;
  float accel_z_m_s2 = 0.0F;
};

}  // namespace atlas::imu
