#include <algorithm>
#include <atlas/sensors/SensorDataBus.hpp>
#include <atomic>
#include <mutex>
#include <utility>
#include <vector>

namespace atlas::sensors {

class SensorDataBus::Impl {
 public:
  using Entry = std::pair<SubscriptionId, std::shared_ptr<ISensorDataSink>>;

  [[nodiscard]] SubscriptionId subscribe(std::shared_ptr<ISensorDataSink> sink) {
    if (!sink) {
      return 0;
    }

    const auto id = next_id_.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard lock(mutex_);
    sinks_.emplace_back(id, std::move(sink));
    return id;
  }

  void unsubscribe(const SubscriptionId id) {
    std::lock_guard lock(mutex_);
    std::erase_if(sinks_, [id](const Entry& entry) { return entry.first == id; });
  }

  [[nodiscard]] std::vector<std::shared_ptr<ISensorDataSink>> snapshot() const {
    std::lock_guard lock(mutex_);
    std::vector<std::shared_ptr<ISensorDataSink>> result;
    result.reserve(sinks_.size());
    for (const auto& [id, sink] : sinks_) {
      static_cast<void>(id);
      result.push_back(sink);
    }
    return result;
  }

 private:
  mutable std::mutex mutex_;
  std::vector<Entry> sinks_;
  std::atomic<SubscriptionId> next_id_{1};
};

SensorDataBus::SensorDataBus() : impl_(std::make_unique<Impl>()) {}
SensorDataBus::~SensorDataBus() = default;

SensorDataBus::SubscriptionId SensorDataBus::subscribe(std::shared_ptr<ISensorDataSink> sink) {
  return impl_->subscribe(std::move(sink));
}

void SensorDataBus::unsubscribe(const SubscriptionId subscription_id) {
  impl_->unsubscribe(subscription_id);
}

void SensorDataBus::publish(const LidarFrame& frame) const noexcept {
  for (const auto& sink : impl_->snapshot()) {
    try {
      sink->on_lidar_frame(frame);
    } catch (...) {
    }
  }
}

void SensorDataBus::publish(const atlas::imu::ImuSample& sample) const noexcept {
  for (const auto& sink : impl_->snapshot()) {
    try {
      sink->on_imu_sample(sample);
    } catch (...) {
    }
  }
}

void SensorDataBus::publish(const SensorStatus& status) const noexcept {
  for (const auto& sink : impl_->snapshot()) {
    try {
      sink->on_sensor_status(status);
    } catch (...) {
    }
  }
}

}  // namespace atlas::sensors
