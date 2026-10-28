#pragma once

#include <atlas/sensors/ISensorDataSink.hpp>
#include <atlas/ui/ui_stream_server_config.hpp>
#include <atlas/ui/WireProtocol.hpp>
#include <cstdint>
#include <memory>
#include <string>

namespace atlas::ui {

struct UiStreamServerStats {
  uint64_t frames_sent = 0;
  uint64_t points_sent = 0;
  uint64_t dropped_points = 0;
  uint64_t warnings_sent = 0;
  uint64_t slam_statuses_sent = 0;
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
  void on_sensor_warning(const atlas::sensors::SensorWarning& warning) override;
  void publish_slam_status(const wire::SlamStatus& status);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace atlas::ui
