#pragma once

#include <atlas/sensors/ISensorDataSink.hpp>
#include <cstdint>
#include <memory>

namespace atlas::sensors {

class SensorDataBus {
 public:
  using SubscriptionId = uint64_t;

  SensorDataBus();
  ~SensorDataBus();

  SensorDataBus(const SensorDataBus&) = delete;
  SensorDataBus& operator=(const SensorDataBus&) = delete;

  [[nodiscard]] SubscriptionId subscribe(std::shared_ptr<ISensorDataSink> sink);
  void unsubscribe(SubscriptionId subscription_id);

  void publish(const LidarFrame& frame) const noexcept;
  void publish(const atlas::imu::ImuSample& sample) const noexcept;
  void publish(const SensorStatus& status) const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace atlas::sensors
