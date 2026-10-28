#pragma once

#include <string>

namespace atlas::livox {

struct LivoxSourceConfig {
  std::string config_path;
  bool enable_point_cloud = true;
  bool enable_imu = true;
};

}  // namespace atlas::livox
