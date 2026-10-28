#include <atlas/timesync/SynchronizationOrchestrator.hpp>
#include <algorithm>

namespace atlas::timesync {
SynchronizationOrchestrator::SynchronizationOrchestrator(SynchronizationConfig config)
    : config_(config), lidar_(config.lidar_capacity), imu_(config.imu_capacity), camera_(config.camera_capacity) {
  if (config.imu_capacity < 2 || !config.max_points_per_packet || !config.max_imu_gap_ns ||
      !config.max_camera_metadata_bytes) throw std::invalid_argument("Invalid synchronization limits");
}

void SynchronizationOrchestrator::configure_clocks(std::optional<ClockAlignment> lidar,
    std::optional<ClockAlignment> imu, std::optional<ClockAlignment> camera) {
  // Validate all models before replacing any active stream.
  LidarSynchronizer next_lidar(config_.lidar_capacity); next_lidar.set_alignment(lidar);
  ImuSynchronizer next_imu(config_.imu_capacity); next_imu.set_alignment(imu);
  CameraSynchronizer next_camera(config_.camera_capacity); next_camera.set_alignment(camera);
  std::lock_guard lock(mutex_);
  stats_.reset_discarded += lidar_.samples().size() + imu_.samples().size() + camera_.samples().size();
  lidar_ = std::move(next_lidar); imu_ = std::move(next_imu); camera_ = std::move(next_camera); ++epoch_;
}

SyncPushResult SynchronizationOrchestrator::account(SyncPushResult result) {
  if (result == SyncPushResult::Accepted) ++stats_.accepted;
  else { ++stats_.rejected; stats_.last_rejection = result; }
  return result;
}
SyncPushResult SynchronizationOrchestrator::push(const sensors::LidarFrame& value) {
  std::lock_guard lock(mutex_);
  if (value.points.size() > config_.max_points_per_packet) return account(SyncPushResult::Oversized);
  if (value.duration_ns > UINT64_MAX - value.timestamp.nanoseconds ||
      std::any_of(value.points.begin(), value.points.end(), [&](const auto& p) { return p.offset_ns > value.duration_ns; }))
    return account(SyncPushResult::InvalidTime);
  if (lidar_.alignment() && !lidar_.alignment()->map(value.timestamp.nanoseconds + value.duration_ns))
    return account(SyncPushResult::InvalidTime);
  return account(lidar_.push(value));
}
SyncPushResult SynchronizationOrchestrator::push(const imu::ImuSample& value) {
  std::lock_guard lock(mutex_); return account(imu_.push(value));
}
SyncPushResult SynchronizationOrchestrator::push(const sensors::CameraFrame& value) {
  std::lock_guard lock(mutex_);
  if (value.image_path.size() > config_.max_camera_metadata_bytes ||
      value.calibration_json.size() > config_.max_camera_metadata_bytes - value.image_path.size())
    return account(SyncPushResult::Oversized);
  return account(camera_.push(value));
}

std::optional<SynchronizedPacket> SynchronizationOrchestrator::poll(uint64_t now, bool eof) {
  std::lock_guard lock(mutex_);
  if (lidar_.samples().empty()) return std::nullopt;
  const auto& scan = lidar_.samples().front();
  const auto begin = scan.time_ns;
  const auto end = *lidar_.alignment()->map(scan.sample.timestamp.nanoseconds + scan.sample.duration_ns);
  const auto middle = begin + (end - begin) / 2;
  const bool expired = eof || (now >= end && now - end >= config_.max_wait_ns);
  while (imu_.samples().size() > 1 && imu_.samples()[1].time_ns <= begin) imu_.pop_front();
  while (camera_.samples().size() > 1 && camera_.samples()[1].time_ns <= middle) camera_.pop_front();

  const auto& samples = imu_.samples();
  const auto after = std::lower_bound(samples.begin(), samples.end(), end,
      [](const auto& s, uint64_t t) { return s.time_ns < t; });
  const bool brackets = !samples.empty() && samples.front().time_ns <= begin && after != samples.end();
  const auto& cameras = camera_.samples();
  const bool camera_past_center = !cameras.empty() && cameras.back().time_ns >= middle;
  if (!expired && (!brackets || (config_.require_camera && !camera_past_center))) return std::nullopt;

  SynchronizedPacket result;
  result.epoch = epoch_; result.begin_ns = begin; result.end_ns = end; result.lidar = scan.sample;
  if (brackets) {
    result.imu.assign(samples.begin(), std::next(after)); result.imu_complete = true;
    for (size_t i = 1; i < result.imu.size(); ++i)
      if (result.imu[i].time_ns - result.imu[i - 1].time_ns > config_.max_imu_gap_ns) result.imu_complete = false;
  } else result.imu.assign(samples.begin(), after);
  uint64_t best_distance = UINT64_MAX;
  for (const auto& camera : cameras) {
    const auto distance = camera.time_ns > middle ? camera.time_ns - middle : middle - camera.time_ns;
    if (distance <= config_.camera_tolerance_ns && distance < best_distance) {
      result.camera = camera; best_distance = distance;
    }
  }
  result.camera_matched = result.camera.has_value();
  const auto lidar_uncertainty = lidar_.alignment()->uncertainty_ns;
  const auto imu_uncertainty = imu_.alignment() ? imu_.alignment()->uncertainty_ns : UINT64_MAX;
  const auto camera_uncertainty = config_.require_camera
      ? (camera_.alignment() ? camera_.alignment()->uncertainty_ns : UINT64_MAX) : 0;
  result.clock_uncertainty_ns = lidar_uncertainty;
  for (const auto uncertainty : {imu_uncertainty, camera_uncertainty}) {
    if (uncertainty > UINT64_MAX - result.clock_uncertainty_ns) result.clock_uncertainty_ns = UINT64_MAX;
    else result.clock_uncertainty_ns += uncertainty;
  }
  result.clocks_precise = result.clock_uncertainty_ns <= config_.max_clock_uncertainty_ns;
  result.ready_for_estimation = result.imu_complete && result.clocks_precise &&
      (!config_.require_camera || result.camera_matched);
  lidar_.pop_front(); ++stats_.emitted;
  if (!result.ready_for_estimation) ++stats_.incomplete;
  return result;
}
SynchronizationStats SynchronizationOrchestrator::stats() const { std::lock_guard lock(mutex_); return stats_; }
}  // namespace atlas::timesync
