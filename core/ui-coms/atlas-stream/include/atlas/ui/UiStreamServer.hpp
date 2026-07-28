#pragma once

#include <atlas/sensors/ISensorDataSink.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace atlas::ui {

struct UiStreamServerConfig {
  uint16_t port = 47777;
  std::string service_name = "ATLAS";
  std::chrono::milliseconds frame_interval{50};
  size_t max_points_per_frame = 10'000;
  size_t max_pending_points = 40'000;
  float maximum_distance_m = 50.0F;
};

struct UiStreamServerStats {
  uint64_t frames_sent = 0;
  uint64_t points_sent = 0;
  uint64_t dropped_points = 0;
  bool client_connected = false;
};

class UiStreamServer final : public atlas::sensors::ISensorDataSink {
 public:
  explicit UiStreamServer(UiStreamServerConfig config = {});
  ~UiStreamServer() override;

  UiStreamServer(const UiStreamServer&) = delete;
  UiStreamServer& operator=(const UiStreamServer&) = delete;

  [[nodiscard]] bool start();
  void stop() noexcept;
  [[nodiscard]] bool running() const noexcept;
  [[nodiscard]] std::string last_error() const;
  [[nodiscard]] UiStreamServerStats stats() const noexcept;

  void on_lidar_frame(const atlas::sensors::LidarFrame& frame) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace atlas::ui
