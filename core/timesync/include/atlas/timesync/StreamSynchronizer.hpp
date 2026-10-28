#pragma once
#include <atlas/sensors/CameraFrame.hpp>
#include <atlas/sensors/LidarFrame.hpp>
#include <atlas/imu/ImuSample.hpp>
#include <cmath>
#include <deque>
#include <optional>
#include <stdexcept>

namespace atlas::timesync {
/// An explicitly established correspondence, not inferred from network arrival.
/// t_ref = reference_anchor_ns + rate * (t_source - source_anchor_ns).
/// All configured streams must target the SAME reference clock/session.
struct ClockAlignment {
  uint32_t sensor_id = 0;
  ClockSource source = ClockSource::Unknown;
  uint64_t source_anchor_ns = 0, reference_anchor_ns = 0;
  double rate = 1.0;
  uint64_t uncertainty_ns = 0;
  [[nodiscard]] std::optional<uint64_t> map(uint64_t source_ns) const noexcept {
    if (!std::isfinite(rate) || rate <= 0) return std::nullopt;
    const bool positive = source_ns >= source_anchor_ns;
    const auto difference = positive ? source_ns - source_anchor_ns : source_anchor_ns - source_ns;
    // Subtract integer anchors BEFORE floating conversion; preserve nanosecond
    // differences even on platforms where long double has double precision.
    uint64_t delta = difference;
    if (rate != 1.0) {
      const long double magnitude = std::round(static_cast<long double>(difference) * rate);
      if (!std::isfinite(magnitude) || magnitude >= 18446744073709551616.0L) return std::nullopt;
      delta = static_cast<uint64_t>(magnitude);
    }
    if (positive) return delta > UINT64_MAX - reference_anchor_ns ? std::nullopt
        : std::optional<uint64_t>(reference_anchor_ns + delta);
    return delta > reference_anchor_ns ? std::nullopt : std::optional<uint64_t>(reference_anchor_ns - delta);
  }
};

enum class SyncPushResult { Accepted, Unaligned, WrongClock, OutOfOrder, QueueFull, InvalidTime, Oversized };
template <typename T> struct TimedSample { uint64_t time_ns; T sample; };

/// Independent bounded synchronizer per sensor stream. Strictly increasing
/// timestamps; late/duplicate samples are rejected and counted by the caller.
/// Changing alignment clears this stream. Orchestrator resets ALL streams when
/// one alignment changes, so samples from different clock models never mix.
template <typename T> class StreamSynchronizer {
 public:
  explicit StreamSynchronizer(size_t capacity) : capacity_(capacity) {
    if (!capacity) throw std::invalid_argument("Synchronizer capacity must be positive");
  }
  void set_alignment(std::optional<ClockAlignment> alignment) {
    if (alignment && (!std::isfinite(alignment->rate) || alignment->rate <= 0))
      throw std::invalid_argument("Clock rate must be finite and positive");
    alignment_ = alignment; clear();
  }
  [[nodiscard]] const std::optional<ClockAlignment>& alignment() const { return alignment_; }
  SyncPushResult push(const T& value) {
    if (!alignment_) return SyncPushResult::Unaligned;
    if (value.sensor_id != alignment_->sensor_id || value.timestamp.source != alignment_->source)
      return SyncPushResult::WrongClock;
    const auto time = alignment_->map(value.timestamp.nanoseconds);
    if (!time) return SyncPushResult::InvalidTime;
    if (last_time_ && *time <= *last_time_) return SyncPushResult::OutOfOrder;
    if (queue_.size() == capacity_) return SyncPushResult::QueueFull;
    queue_.push_back({*time, value}); last_time_ = time; return SyncPushResult::Accepted;
  }
  void clear() { queue_.clear(); last_time_.reset(); }
  [[nodiscard]] const std::deque<TimedSample<T>>& samples() const { return queue_; }
  void pop_front() { if (!queue_.empty()) queue_.pop_front(); }
 private:
  size_t capacity_;
  std::optional<ClockAlignment> alignment_;
  std::optional<uint64_t> last_time_;
  std::deque<TimedSample<T>> queue_;
};

using LidarSynchronizer = StreamSynchronizer<sensors::LidarFrame>;
using ImuSynchronizer = StreamSynchronizer<imu::ImuSample>;
using CameraSynchronizer = StreamSynchronizer<sensors::CameraFrame>;
}  // namespace atlas::timesync
