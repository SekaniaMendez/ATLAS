#include "LiveGeometryWorker.hpp"
#include <algorithm>
#include <stdexcept>
#include <utility>

namespace atlas::ui {
LiveGeometryWorker::LiveGeometryWorker(std::chrono::milliseconds interval, Handler handler, Extractor extractor)
    : interval_(interval), handler_(std::move(handler)), extractor_(std::move(extractor)) {
  if (interval.count() <= 0 || !handler_) throw std::invalid_argument("Invalid live geometry worker");
  if (!extractor_) extractor_ = [](auto points, uint64_t timestamp) {
    return geometry::extract_geometry(points, timestamp);
  };
}
LiveGeometryWorker::~LiveGeometryWorker() { stop(); }
void LiveGeometryWorker::start() {
  std::lock_guard lock(mutex_);
  if (running_) return;
  running_ = true;
  thread_ = std::thread([this] { run(); });
}
void LiveGeometryWorker::stop() {
  {
    std::lock_guard lock(mutex_);
    running_ = false; ++epoch_; pending_ = false;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}
void LiveGeometryWorker::emit_locked(geometry::GeometrySnapshot snapshot, uint64_t revision, float ms) {
  try { handler_(std::move(snapshot), revision, ms); }
  catch (const std::exception&) { ++errors_; }
}
void LiveGeometryWorker::clear_locked(uint64_t timestamp) {
  ++epoch_; ++revision_; pending_ = false; latest_ = {};
  geometry::GeometrySnapshot empty;
  empty.source_timestamp_ns = timestamp;
  emit_locked(std::move(empty), revision_, 0);
}
void LiveGeometryWorker::reset() {
  std::lock_guard lock(mutex_);
  tracking_ = false;
  has_source_timestamp_ = false;
  clear_locked(0);
  cv_.notify_all();
}
void LiveGeometryWorker::set_tracking(bool tracking) {
  std::lock_guard lock(mutex_);
  if (tracking_ == tracking) return;
  tracking_ = tracking;
  if (!tracking_) clear_locked(0);
  cv_.notify_all();
}
void LiveGeometryWorker::submit(const sensors::LidarFrame& frame) {
  std::lock_guard lock(mutex_);
  if (!running_ || !tracking_) return;
  if (has_source_timestamp_ && (frame.timestamp.nanoseconds < last_source_timestamp_ ||
      (frame.timestamp.nanoseconds == last_source_timestamp_ && !frame.points.empty()))) return;
  has_source_timestamp_ = true;
  last_source_timestamp_ = frame.timestamp.nanoseconds;
  if (frame.points.empty()) {
    clear_locked(frame.timestamp.nanoseconds);
  } else {
    // The application already bounds map snapshots. Bound this API too, using
    // distributed sampling so accidental larger callers cannot grow the queue.
    constexpr size_t limit = 50'000;
    latest_truncated_ = frame.points.size() >= limit;
    latest_ = {};
    latest_.timestamp = frame.timestamp;
    const auto stride = std::max<size_t>(1, (frame.points.size() + limit - 1) / limit);
    latest_.points.reserve(std::min(limit, frame.points.size()));
    for (size_t i = 0; i < frame.points.size(); i += stride) latest_.points.push_back(frame.points[i]);
    ++revision_; pending_ = true;
  }
  cv_.notify_all();
}
void LiveGeometryWorker::run() {
  auto next = std::chrono::steady_clock::now();
  std::unique_lock lock(mutex_);
  while (running_) {
    cv_.wait(lock, [this] { return !running_ || (tracking_ && pending_); });
    if (!running_) break;
    if (std::chrono::steady_clock::now() < next) {
      cv_.wait_until(lock, next, [this] { return !running_ || !tracking_ || !pending_; });
      continue;
    }
    auto frame = std::move(latest_);
    pending_ = false;
    const auto epoch = epoch_, revision = revision_;
    const bool truncated = latest_truncated_;
    lock.unlock();
    const auto started = std::chrono::steady_clock::now();
    geometry::GeometrySnapshot result;
    try { result = extractor_(frame.points, frame.timestamp.nanoseconds); }
    catch (const std::exception&) { ++errors_; result.source_timestamp_ns = frame.timestamp.nanoseconds; }
    const auto finished = std::chrono::steady_clock::now();
    result.truncated = result.truncated || truncated;
    const float ms = std::chrono::duration<float, std::milli>(finished - started).count();
    lock.lock();
    // A clear, tracking loss or reconnect invalidates an in-flight calculation.
    if (running_ && tracking_ && epoch == epoch_) emit_locked(std::move(result), revision, ms);
    next = started + interval_;
  }
}
}  // namespace atlas::ui
