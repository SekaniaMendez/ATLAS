#pragma once
#include <atlas/geometry/LiveGeometry.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace atlas::ui {
/// One in-flight extraction and one replaceable pending map. Callbacks only
/// enqueue results; they must not reenter this worker or perform network I/O.
class LiveGeometryWorker {
 public:
  using Handler = std::function<void(geometry::GeometrySnapshot, uint64_t, float)>;
  using Extractor = std::function<geometry::GeometrySnapshot(std::span<const sensors::LidarPoint>, uint64_t)>;
  LiveGeometryWorker(std::chrono::milliseconds interval, Handler handler, Extractor extractor = {});
  ~LiveGeometryWorker();
  void start();
  void stop();
  void set_tracking(bool tracking);
  void submit(const sensors::LidarFrame& frame);
  void reset();
  uint64_t errors() const noexcept { return errors_.load(); }
 private:
  void run();
  void clear_locked(uint64_t timestamp);
  void emit_locked(geometry::GeometrySnapshot snapshot, uint64_t revision, float ms);
  std::chrono::milliseconds interval_;
  Handler handler_;
  Extractor extractor_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread thread_;
  bool running_ = false, tracking_ = false, pending_ = false;
  bool latest_truncated_ = false, has_source_timestamp_ = false;
  uint64_t last_source_timestamp_ = 0;
  sensors::LidarFrame latest_;
  uint64_t epoch_ = 0, revision_ = 0;
  std::atomic_uint64_t errors_{0};
};
}  // namespace atlas::ui
