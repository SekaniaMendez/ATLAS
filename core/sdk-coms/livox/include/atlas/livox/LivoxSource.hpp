#pragma once

#include <atlas/sensors/SensorDataBus.hpp>
#include <cstdint>
#include <memory>
#include <string>

namespace atlas::livox {

struct LivoxSourceConfig {
  std::string config_path;
  bool enable_point_cloud = true;
  bool enable_imu = true;
};

struct LivoxSourceStats {
  uint64_t point_packets = 0;
  uint64_t points = 0;
  uint64_t imu_packets = 0;
  uint64_t imu_samples = 0;
  uint64_t rejected_packets = 0;
};

class LivoxSource {
 public:
  LivoxSource(atlas::sensors::SensorDataBus& output, LivoxSourceConfig config);
  ~LivoxSource();

  LivoxSource(const LivoxSource&) = delete;
  LivoxSource& operator=(const LivoxSource&) = delete;
  LivoxSource(LivoxSource&&) = delete;
  LivoxSource& operator=(LivoxSource&&) = delete;

  [[nodiscard]] bool start();
  void stop() noexcept;

  [[nodiscard]] bool running() const noexcept;
  [[nodiscard]] LivoxSourceStats stats() const noexcept;
  [[nodiscard]] std::string last_error() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace atlas::livox
