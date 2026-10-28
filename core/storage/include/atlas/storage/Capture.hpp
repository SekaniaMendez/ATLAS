#pragma once

#include <atlas/sensors/ISensorDataSink.hpp>
#include <atlas/storage/capture_config.hpp>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <variant>

namespace atlas::storage {
using SensorEvent = std::variant<sensors::LidarFrame, imu::ImuSample, sensors::SensorStatus,
                                 sensors::SensorWarning, sensors::CameraFrame>;
struct CaptureEvent {
  uint64_t sequence = 0;
  uint64_t arrival_offset_ns = 0; // Mac steady clock relative to session start
  SensorEvent data;
  void dispatch(sensors::ISensorDataSink& sink) const;
};
struct CaptureStats {
  uint64_t accepted_events = 0;
  uint64_t written_events = 0;
  size_t pending_bytes = 0; // includes in-flight writer record
  size_t peak_pending_bytes = 0;
  bool failed = false;
  std::string error;
};

/// Bounded asynchronous raw recorder. Directory must not exist. Metadata is
/// an opaque UTF-8 manifest supplied by the app, including calibration/config.
/// Overflow/I/O failure permanently fails this recording (never silently drops).
/// Call close() after stopping producers; it drains and writes the completion
/// marker, or throws. Destruction only makes a best-effort close.
class CaptureRecorder final : public sensors::ISensorDataSink {
 public:
  CaptureRecorder(const std::filesystem::path& directory, std::string metadata,
                  size_t queue_budget_bytes = kDefaultCaptureQueueBudgetBytes);
  ~CaptureRecorder() override;
  CaptureRecorder(const CaptureRecorder&) = delete;
  CaptureRecorder& operator=(const CaptureRecorder&) = delete;
  void on_lidar_frame(const sensors::LidarFrame&) override;
  void on_imu_sample(const imu::ImuSample&) override;
  void on_sensor_status(const sensors::SensorStatus&) override;
  void on_sensor_warning(const sensors::SensorWarning&) override;
  void on_camera_frame(const sensors::CameraFrame&) override;
  [[nodiscard]] CaptureStats stats() const;
  void close();
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/// Streaming reader: memory is bounded by one record. next() returns nullopt
/// only after a valid completion marker; truncated/incomplete recordings throw.
/// Arrival order is preserved; independent sensor clocks are never interleaved
/// by their raw timestamp values. Caller chooses replay speed using offsets.
class CaptureReader {
 public:
  explicit CaptureReader(const std::filesystem::path& directory);
  ~CaptureReader();
  [[nodiscard]] const std::string& metadata() const;
  [[nodiscard]] std::optional<CaptureEvent> next();
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace atlas::storage
