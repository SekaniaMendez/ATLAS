#include <atlas/slam/VoxelSlamEngine.hpp>
#include <atlas/storage/VoxelMapArchive.hpp>
#include "ImuTracker.hpp"
#include "VoxelRegistration.hpp"
#include "TemporalCanvas.hpp"
#include "CameraColorizer.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <variant>

namespace atlas::slam {
namespace {
using Vector = Eigen::Vector3d;
using Rotation = Eigen::Quaterniond;
Rotation rotation(const PoseEstimate& p) { return {p.qw, p.qx, p.qy, p.qz}; }
Vector position(const PoseEstimate& p) { return {p.x_m, p.y_m, p.z_m}; }
void assign(PoseEstimate& p, const Vector& t, const Rotation& r) {
  p.x_m = t.x(); p.y_m = t.y(); p.z_m = t.z();
  p.qx = r.x(); p.qy = r.y(); p.qz = r.z(); p.qw = r.w();
}
bool positive(double value) { return std::isfinite(value) && value > 0; }
std::string registration_failure_message(const RegistrationResult& result,
                                         const VoxelRegistrationConfig& config) {
  switch (result.failure) {
    case RegistrationFailure::InsufficientCorrespondences: {
      const auto required = std::max(config.minimum_correspondences,
          static_cast<size_t>(std::ceil(result.query_count * config.minimum_correspondence_ratio)));
      return "Insufficient map support: " + std::to_string(result.correspondences) + "/" +
          std::to_string(result.query_count) + " correspondences; require " + std::to_string(required);
    }
    case RegistrationFailure::DegenerateGeometry:
      return "Degenerate geometry: observe surfaces in more directions";
    case RegistrationFailure::DidNotConverge:
      return "Scan registration did not converge after " + std::to_string(result.iterations) + " iterations";
    case RegistrationFailure::ExcessiveCorrection:
      return "Scan registration correction exceeds configured translation or rotation limit";
    case RegistrationFailure::ExcessiveResidual:
      return "Scan registration residual exceeds configured limit: RMSE=" +
          std::to_string(result.rmse_m) + " m";
    case RegistrationFailure::InsufficientPoints:
      return "Insufficient usable points for scan registration";
    case RegistrationFailure::EmptyMap:
      return "Scan registration reference map is empty";
    case RegistrationFailure::InvalidInput:
      return "Invalid pose prediction for scan registration";
    case RegistrationFailure::NumericalFailure:
      return "Scan registration numerical solver failed";
    case RegistrationFailure::None:
      return "Scan registration rejected without a result";
  }
  return "Scan registration rejected";
}
void validate(const VoxelSlamConfig& c) {
  if (!c.scan_duration_ns || c.scan_duration_ns > c.max_scan_duration_ns ||
      c.max_scan_duration_ns > UINT32_MAX || c.minimum_scan_points < 6 ||
      c.maximum_scan_points < c.minimum_scan_points || c.maximum_scan_points > 1'000'000 ||
      !positive(c.input_voxel_size_m) || !positive(c.maximum_range_m) ||
      !std::isfinite(c.minimum_range_m) || c.minimum_range_m < 0 ||
      c.maximum_range_m <= c.minimum_range_m || !c.max_queue_bytes || !c.max_pending_scans ||
      !c.max_canvas_blocks || !c.max_matching_blocks || !c.max_trajectory_poses ||
      !c.imu_initialization_ns || c.minimum_initialization_imu_samples < 2 ||
      !c.maximum_imu_gap_ns || c.imu_history_ns < c.imu_initialization_ns ||
      c.imu_history_ns - c.imu_initialization_ns < c.max_scan_duration_ns ||
      c.maximum_imu_samples < c.minimum_initialization_imu_samples ||
      !positive(c.gravity_m_s2) || !positive(c.stationary_gyro_limit_rad_s) ||
      !positive(c.maximum_initialization_gyro_bias_rad_s) || !positive(c.stationary_gyro_stddev_rad_s) ||
      !positive(c.stationary_accel_tolerance_m_s2) || !positive(c.stationary_accel_stddev_m_s2) ||
      !positive(c.maximum_prediction_interval_s) || !positive(c.maximum_speed_m_s) ||
      !positive(c.matching_keyframe_translation_m) || !positive(c.matching_keyframe_rotation_rad) ||
      (c.temporal_canvas.enabled && c.canvas.voxel_size_m > c.matching_map.voxel_size_m) ||
      !std::isfinite(c.velocity_smoothing) || c.velocity_smoothing < 0 || c.velocity_smoothing > 1)
    throw std::invalid_argument("Invalid VoxelSlamConfig: sizes, durations and thresholds must be bounded and positive");
  double norm = 0;
  for (double v : c.imu_to_lidar_rotation_xyzw) norm += v * v;
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)
    throw std::invalid_argument("IMU-to-LiDAR rotation must be a finite unit quaternion");
}
class NewFile {
 public:
  explicit NewFile(const std::filesystem::path& path) {
    file_ = std::fopen(path.string().c_str(), "wx");
    if (!file_) throw std::runtime_error("Cannot create new output file: " + path.string());
  }
  ~NewFile() { if (file_) std::fclose(file_); }
  FILE* get() const { return file_; }
  void close() {
    const bool failed = std::ferror(file_);
    const int result = std::fclose(file_);
    file_ = nullptr;
    if (failed || result) throw std::runtime_error("Failed to write SLAM output file");
  }
 private:
  FILE* file_ = nullptr;
};
}

class VoxelSlamEngine::Impl {
 public:
  explicit Impl(VoxelSlamConfig config)
      : config_(std::move(config)), canvas_(config_.canvas), matching_(config_.matching_map),
        registration_(config_.registration), imu_(config_),
        temporal_(config_.temporal_canvas, config_.max_canvas_blocks, config_.max_matching_blocks),
        colorizer_(config_.camera_fusion) {
    validate(config_);
    worker_ = std::thread([this] { run(); });
  }
  ~Impl() { finish(); }
  using Event = std::variant<sensors::LidarFrame, imu::ImuSample>;
  struct QueuedEvent { Event value; size_t bytes; };
  struct TimedPoint { sensors::LidarPoint point; uint64_t time; };
  struct Scan {
    uint64_t begin = 0, end = 0;
    std::vector<TimedPoint> points;
  };

  void enqueue(const sensors::LidarFrame& frame) {
    if (frame.points.size() > config_.maximum_scan_points) {
      rejected_input_.fetch_add(1); return;
    }
    const size_t bytes = sizeof(QueuedEvent) + frame.points.size() * sizeof(sensors::LidarPoint);
    std::lock_guard lock(queue_mutex_);
    if (!can_enqueue(bytes)) return;
    try { queue_.push_back({frame, bytes}); }
    catch (...) { overflow_.store(true); rejected_input_.fetch_add(1); wake_.notify_one(); return; }
    queued_bytes_ += bytes; peak_queued_bytes_ = std::max(peak_queued_bytes_, queued_bytes_);
    wake_.notify_one();
  }
  void enqueue(const imu::ImuSample& sample) {
    std::lock_guard lock(queue_mutex_);
    const size_t bytes = sizeof(QueuedEvent);
    if (!can_enqueue(bytes)) return;
    try { queue_.push_back({sample, bytes}); }
    catch (...) { overflow_.store(true); rejected_input_.fetch_add(1); wake_.notify_one(); return; }
    queued_bytes_ += bytes; peak_queued_bytes_ = std::max(peak_queued_bytes_, queued_bytes_);
    wake_.notify_one();
  }
  void wait_until_idle() {
    std::unique_lock lock(queue_mutex_);
    idle_.wait(lock, [this] { return queue_.empty() && !busy_; });
  }
  void finish() {
    std::lock_guard finish_lock(finish_mutex_);
    {
      std::lock_guard lock(queue_mutex_);
      stopping_ = true;
    }
    wake_.notify_all();
    if (worker_.joinable()) worker_.join();
  }
  SlamStats stats() const {
    SlamStats result;
    {
      std::lock_guard lock(state_mutex_);
      result = stats_;
    }
    {
      std::lock_guard lock(queue_mutex_);
      result.queued_bytes = queued_bytes_;
      result.peak_queued_bytes = peak_queued_bytes_;
    }
    result.rejected_events += rejected_input_.load();
    return result;
  }
  std::optional<PoseEstimate> pose() const {
    std::lock_guard lock(state_mutex_);
    auto pose = pose_;
    if (pose) pose->state = stats_.state;
    return pose;
  }
  std::vector<PoseEstimate> trajectory() const {
    std::lock_guard lock(state_mutex_); return trajectory_;
  }
  std::vector<MapPoint> points(size_t maximum) const {
    std::lock_guard lock(state_mutex_);
    std::vector<MapPoint> result;
    if (!maximum) return result;
    const size_t count = canvas_.voxel_count();
    const size_t stride = count / maximum + (count % maximum != 0);
    result.reserve(std::min(maximum, count));
    size_t index = 0;
    for (const auto& [key, block] : canvas_.blocks()) {
      (void)key;
      for (const auto& voxel : block.voxels) {
        if (voxel.samples.empty()) continue;
        if (index++ % std::max<size_t>(1, stride) == 0 && result.size() < maximum)
          result.push_back(voxel.samples.back());
      }
    }
    return result;
  }
  void save(const std::filesystem::path& path) const {
    require_finished();
    std::lock_guard lock(state_mutex_);
    if (!canvas_.voxel_count()) throw std::runtime_error("Cannot save an empty SLAM map");
    storage::VoxelMapArchive::save(path, canvas_);
    write_session_report(path / "slam_session.txt");
  }
  void session_report(const std::filesystem::path& path) const {
    require_finished();
    std::lock_guard lock(state_mutex_);
    write_session_report(path);
  }
  // Caller holds state_mutex_; keep standalone and archive metadata identical.
  void write_session_report(const std::filesystem::path& path) const {
    NewFile metadata(path);
    std::fprintf(metadata.get(),
        "algorithm=ATLAS voxel scan-to-map prototype\n"
        "frame=local metric; initial position and yaw arbitrary\n"
        "georeferenced=false\nloop_closure=false\n"
        "imu_mode=%s\nclock_source=%u\n"
        "canvas_voxel_size_m=%.9g\nmatching_voxel_size_m=%.9g\n"
        "input_voxel_size_m=%.9g\nscan_duration_ns=%llu\n"
        "matching_keyframe_translation_m=%.9g\nmatching_keyframe_rotation_rad=%.9g\n"
        "scans_accepted=%llu\nscans_rejected=%llu\ntracking_state=%u\nfatal_error=%u\n"
        "status=%s\n",
        config_.use_imu ? "gyro prediction and rotational deskew; stationary gravity initialization"
                        : "disabled; constant-velocity translation prediction",
        static_cast<unsigned>(clock_), config_.canvas.voxel_size_m, config_.matching_map.voxel_size_m,
        config_.input_voxel_size_m, static_cast<unsigned long long>(config_.scan_duration_ns),
        config_.matching_keyframe_translation_m, config_.matching_keyframe_rotation_rad,
        static_cast<unsigned long long>(stats_.scans_accepted),
        static_cast<unsigned long long>(stats_.scans_rejected),
        static_cast<unsigned>(stats_.state), static_cast<unsigned>(stats_.fatal_error),
        stats_.message.c_str());
    std::fprintf(metadata.get(),
        "temporal_canvas_enabled=%u\nconfirmation_scans=%u\nclearing_scans=%u\n"
        "temporal_evidence_window_ns=%llu\ncandidate_lifetime_ns=%llu\n"
        "ray_tolerance_m=%.9g\nsurface_margin_m=%.9g\nmaximum_rays_per_scan=%zu\n"
        "candidate_voxels=%llu\ncleared_voxels=%llu\nexpired_candidate_voxels=%llu\n"
        "temporal_skipped_scans=%llu\nprovisional_matching_voxels=%llu\ncontradicted_candidate_voxels=%llu\n",
        static_cast<unsigned>(config_.temporal_canvas.enabled), config_.temporal_canvas.confirmation_scans,
        config_.temporal_canvas.clearing_scans,
        static_cast<unsigned long long>(config_.temporal_canvas.evidence_window_ns),
        static_cast<unsigned long long>(config_.temporal_canvas.candidate_lifetime_ns),
        config_.temporal_canvas.ray_tolerance_m, config_.temporal_canvas.surface_margin_m,
        config_.temporal_canvas.maximum_rays_per_scan,
        static_cast<unsigned long long>(stats_.candidate_voxels),
        static_cast<unsigned long long>(stats_.cleared_voxels),
        static_cast<unsigned long long>(stats_.expired_candidate_voxels),
        static_cast<unsigned long long>(stats_.temporal_skipped_scans),
        static_cast<unsigned long long>(stats_.provisional_matching_voxels),
        static_cast<unsigned long long>(stats_.contradicted_candidate_voxels));
    std::fprintf(metadata.get(),
        "camera_clock_alignment_verified=%u\nphone_to_lidar_offset_ns=%lld\n"
        "camera_clock_uncertainty_ns=%llu\ncamera_lidar_clock=%u\n"
        "colored_points=%llu\ncolor_conflicts=%llu\n",
        static_cast<unsigned>(config_.camera_fusion.clock_alignment_verified),
        static_cast<long long>(config_.camera_fusion.phone_to_lidar_offset_ns),
        static_cast<unsigned long long>(config_.camera_fusion.clock_uncertainty_ns),
        static_cast<unsigned>(config_.camera_fusion.lidar_clock),
        static_cast<unsigned long long>(stats_.colored_points),
        static_cast<unsigned long long>(stats_.color_conflicts));
    metadata.close();
  }
  void ply(const std::filesystem::path& path) const {
    require_finished();
    std::lock_guard lock(state_mutex_);
    if (!canvas_.voxel_count()) throw std::runtime_error("Cannot export an empty SLAM map");
    NewFile file(path);
    std::fprintf(file.get(), "ply\nformat ascii 1.0\ncomment ATLAS local SLAM prototype; one measured point per voxel\n"
                 "element vertex %zu\nproperty double x\nproperty double y\nproperty double z\n"
                 "property uchar reflectivity\nproperty uchar red\nproperty uchar green\n"
                 "property uchar blue\nproperty uchar has_color\nend_header\n", canvas_.voxel_count());
    for (const auto& [key, block] : canvas_.blocks()) {
      (void)key;
      for (const auto& voxel : block.voxels) {
        if (voxel.samples.empty()) continue;
        const auto& p = voxel.samples.back();
        std::fprintf(file.get(), "%.9f %.9f %.9f %u %u %u %u %u\n", p.x_m, p.y_m, p.z_m,
                     static_cast<unsigned>(p.reflectivity),
                     static_cast<unsigned>(p.has_color ? p.rgb[0] : p.reflectivity),
                     static_cast<unsigned>(p.has_color ? p.rgb[1] : p.reflectivity),
                     static_cast<unsigned>(p.has_color ? p.rgb[2] : p.reflectivity),
                     static_cast<unsigned>(p.has_color));
      }
    }
    file.close();
  }
  void csv(const std::filesystem::path& path) const {
    require_finished();
    std::lock_guard lock(state_mutex_);
    NewFile file(path);
    std::fprintf(file.get(), "timestamp_ns,x_m,y_m,z_m,qx,qy,qz,qw\n");
    for (const auto& p : trajectory_)
      std::fprintf(file.get(), "%llu,%.9f,%.9f,%.9f,%.12f,%.12f,%.12f,%.12f\n",
                   static_cast<unsigned long long>(p.timestamp.nanoseconds), p.x_m, p.y_m, p.z_m,
                   p.qx, p.qy, p.qz, p.qw);
    file.close();
  }
  VoxelSlamConfig config() const { return config_; }
  bool camera(sensors::CameraRgbFrame frame) { return colorizer_.submit(std::move(frame)); }

 private:
  bool can_enqueue(size_t bytes) {
    if (stopping_ || overflow_.load()) { rejected_input_.fetch_add(1); return false; }
    if (bytes > config_.max_queue_bytes || queued_bytes_ > config_.max_queue_bytes - bytes) {
      overflow_.store(true); rejected_input_.fetch_add(1); wake_.notify_one(); return false;
    }
    return true;
  }
  void require_finished() const {
    std::lock_guard lock(queue_mutex_);
    if (!finished_) throw std::logic_error("Stop producers and finish SLAM before exporting");
  }
  void run() noexcept {
    for (;;) {
      std::optional<QueuedEvent> event;
      {
        std::unique_lock lock(queue_mutex_);
        wake_.wait(lock, [this] { return stopping_ || !queue_.empty() || overflow_.load(); });
        if (queue_.empty() && stopping_) break;
        if (queue_.empty() && overflow_.load()) {
          lock.unlock();
          { std::lock_guard state(state_mutex_); fail("Input queue overflow; restart with a slower stream/replay"); }
          lock.lock();
          wake_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
          if (queue_.empty() && stopping_) break;
        }
        if (queue_.empty()) continue;
        event = std::move(queue_.front()); queue_.pop_front(); busy_ = true;
      }
      try {
        std::lock_guard lock(state_mutex_);
        if (overflow_.load()) fail("Input queue overflow; incomplete sensor stream");
        if (!stats_.fatal_error) {
          std::visit([this](const auto& value) { consume(value); }, event->value);
          drain(false);
        }
      } catch (const std::exception& error) {
        std::lock_guard lock(state_mutex_); fail(error.what());
      } catch (...) {
        std::lock_guard lock(state_mutex_); fail("Unknown SLAM worker failure");
      }
      {
        std::lock_guard lock(queue_mutex_);
        queued_bytes_ -= event->bytes; busy_ = false;
        if (queue_.empty()) idle_.notify_all();
      }
    }
    try {
      std::lock_guard lock(state_mutex_);
      if (overflow_.load()) fail("Input queue overflow; incomplete sensor stream");
      if (!stats_.fatal_error) {
        seal();
        drain(true);
      }
    } catch (const std::exception& error) {
      std::lock_guard lock(state_mutex_); fail(error.what());
    } catch (...) {
      std::lock_guard lock(state_mutex_); fail("SLAM finalization failed");
    }
    {
      std::lock_guard lock(queue_mutex_);
      finished_ = true; busy_ = false; idle_.notify_all();
    }
  }
  void fail(std::string message) {
    stats_.fatal_error = true; stats_.state = TrackingState::Lost; stats_.message = std::move(message);
  }
  void reject_scan(std::string message) {
    ++stats_.scans_rejected;
    if (pose_ && tracking_failure_cause_.empty()) tracking_failure_cause_ = message;
    stats_.state = pose_ ? TrackingState::Lost : TrackingState::Initializing;
    stats_.message = std::move(message);
  }
  bool identity(uint32_t id, timesync::ClockSource clock) {
    if (clock == timesync::ClockSource::Unknown) {
      ++stats_.rejected_events; stats_.message = "Unknown sensor clock rejected"; return false;
    }
    if (!sensor_id_) { sensor_id_ = id; clock_ = clock; }
    if (*sensor_id_ != id) {
      ++stats_.rejected_events; stats_.message = "Different sensor_id rejected"; return false;
    }
    if (clock_ != clock) {
      ++stats_.rejected_events; fail("Sensor clock changed; reset SLAM before continuing"); return false;
    }
    return true;
  }
  void consume(const imu::ImuSample& sample) {
    if (!config_.use_imu) return;
    if (!identity(sample.sensor_id, sample.timestamp.source)) return;
    if (!imu_.add(sample)) { ++stats_.rejected_events; return; }
    latest_measurement_ns_ = std::max(latest_measurement_ns_, sample.timestamp.nanoseconds);
    ++stats_.imu_samples;
    if (!pose_) {
      stats_.state = TrackingState::Initializing;
      stats_.message = imu_.ready() ? "IMU initialized; waiting for a usable scan"
                                   : "Keep sensor still for gyro bias and gravity initialization";
    }
  }
  void consume(const sensors::LidarFrame& frame) {
    if (!identity(frame.sensor_id, frame.timestamp.source)) return;
    const uint64_t begin = frame.timestamp.nanoseconds;
    if (last_lidar_time_ && begin < *last_lidar_time_ &&
        *last_lidar_time_ - begin > config_.max_scan_duration_ns) {
      ++stats_.rejected_events;
      fail("Sensor timestamp moved backwards; reset SLAM after a device restart");
      return;
    }
    if ((last_lidar_time_ && begin <= *last_lidar_time_) ||
        frame.duration_ns > config_.max_scan_duration_ns ||
        begin > UINT64_MAX - frame.duration_ns) { ++stats_.rejected_events; return; }
    for (const auto& p : frame.points)
      if (p.offset_ns > frame.duration_ns) { ++stats_.rejected_events; return; }
    last_lidar_time_ = begin;
    const uint64_t end = begin + frame.duration_ns;
    latest_measurement_ns_ = std::max(latest_measurement_ns_, end);
    if (!assembling_.points.empty() && end - assembling_.begin > config_.max_scan_duration_ns) seal();
    if (assembling_.points.empty()) assembling_.begin = begin;
    assembling_.end = std::max(assembling_.end, end);
    const double min2 = config_.minimum_range_m * config_.minimum_range_m;
    const double max2 = config_.maximum_range_m * config_.maximum_range_m;
    const size_t previous_size = assembling_.points.size();
    for (const auto& p : frame.points) {
      const double d2 = static_cast<double>(p.x_m) * p.x_m + static_cast<double>(p.y_m) * p.y_m +
                        static_cast<double>(p.z_m) * p.z_m;
      if (!std::isfinite(d2) || d2 < min2 || d2 > max2) continue;
      if (assembling_.points.size() == config_.maximum_scan_points) {
        fail("Scan point budget exceeded; reduce scan duration or increase maximum_scan_points");
        assembling_ = {}; return;
      }
      assembling_.points.push_back({p, begin + p.offset_ns});
    }
    if (!frame.points.empty() && assembling_.points.size() == previous_size) {
      ++stats_.rejected_events;
      stats_.message = "LiDAR packet contains no valid points within configured range";
    }
    if (end - assembling_.begin >= config_.scan_duration_ns) seal();
  }
  void seal() {
    if (assembling_.points.empty()) { assembling_ = {}; return; }
    if (pending_.size() == config_.max_pending_scans) {
      ++stats_.scans_processed;
      reject_scan("Scan queue full while waiting for IMU coverage");
      pending_.pop_front();
    }
    pending_.push_back(std::move(assembling_)); assembling_ = {};
  }
  bool short_final_imu_tail(const Scan& scan) const {
    if (!config_.use_imu || !imu_.ready() || !pose_) return false;
    const uint64_t last_imu = imu_.latest_time();
    const uint64_t last_pose = pose_->timestamp.nanoseconds;
    if (last_imu >= scan.end || scan.end - last_imu > config_.maximum_imu_gap_ns ||
        scan.end <= last_pose ||
        static_cast<double>(scan.end - last_pose) * 1e-9 > config_.maximum_prediction_interval_s)
      return false;
    // EOF may fall just before a scan boundary, leaving both that scan and a
    // short successor pending. Only missing end coverage is benign: verify the
    // available prefix, including the interval since the last accepted pose,
    // so an internal gap or missing history cannot be hidden by finalization.
    return imu_.path(std::min(scan.begin, last_pose), last_imu).has_value();
  }
  void drain(bool final) {
    while (!pending_.empty()) {
      const auto& scan = pending_.front();
      if (!final && config_.camera_fusion.clock_alignment_verified &&
          (latest_measurement_ns_ < scan.end ||
           latest_measurement_ns_ - scan.end < config_.camera_fusion.maximum_wait_ns)) return;
      if (config_.use_imu && imu_.ready() && imu_.latest_time() < scan.end && !final) return;
      if (final && short_final_imu_tail(scan)) {
        ++stats_.scans_processed; ++stats_.scans_rejected;
        if (stats_.state != TrackingState::Lost)
          stats_.message = "Final scan discarded: IMU ends before scan; last complete pose retained";
        pending_.pop_front();
        continue;
      }
      process(scan); pending_.pop_front();
      if (stats_.fatal_error) { pending_.clear(); return; }
    }
  }
  std::vector<sensors::LidarPoint> deskew(const Scan& scan, const PoseEstimate& prediction,
                                       const std::vector<ImuTracker::Knot>& path) const {
    std::vector<sensors::LidarPoint> result;
    result.reserve(scan.points.size());
    std::unordered_set<VoxelKey, VoxelKeyHash> cells;
    cells.reserve(scan.points.size());
    const Rotation end_inverse = path.empty() ? Rotation::Identity() : path.back().rotation.conjugate();
    const Vector local_velocity = rotation(prediction).conjugate() * velocity_;
    for (const auto& timed : scan.points) {
      Vector point(timed.point.x_m, timed.point.y_m, timed.point.z_m);
      if (!path.empty()) point = end_inverse * ImuTracker::at(path, timed.time) * point;
      point += local_velocity * (-static_cast<double>(scan.end - timed.time) * 1e-9);
      const Vector index = (point / config_.input_voxel_size_m).array().floor().matrix();
      if ((index.array() < INT32_MIN).any() || (index.array() > INT32_MAX).any()) continue;
      const VoxelKey key{static_cast<int32_t>(index.x()), static_cast<int32_t>(index.y()),
                         static_cast<int32_t>(index.z())};
      if (!cells.insert(key).second) continue;
      auto p = timed.point;
      p.x_m = static_cast<float>(point.x()); p.y_m = static_cast<float>(point.y());
      p.z_m = static_cast<float>(point.z());
      // Preserve acquisition time relative to the assembled scan for free-space rays.
      p.offset_ns = static_cast<uint32_t>(timed.time - scan.begin);
      result.push_back(p);
    }
    return result;
  }
  std::vector<MapPoint> world_points(const std::vector<sensors::LidarPoint>& points,
                                    const PoseEstimate& pose) const {
    std::vector<MapPoint> result; result.reserve(points.size());
    const auto r = rotation(pose); const auto t = position(pose);
    for (const auto& p : points) {
      const Vector w = r * Vector(p.x_m, p.y_m, p.z_m) + t;
      result.push_back({.x_m = w.x(), .y_m = w.y(), .z_m = w.z(), .reflectivity = p.reflectivity,
                        .rgb = p.rgb, .has_color = p.has_color, .color_timestamp_ns = p.color_timestamp_ns});
    }
    return result;
  }
  bool fits(const SparseVoxelMap& map, const std::vector<MapPoint>& points, size_t limit) const {
    std::unordered_set<BlockKey, BlockKeyHash> additions;
    for (const auto& p : points) {
      if (map.config().bounds && !map.config().bounds->contains(p)) continue;
      const auto key = map.voxel_key_for(p);
      if (!key) return false;
      const auto block = SparseVoxelMap::address_for(*key).block;
      if (!map.find_block(block)) additions.insert(block);
      if (map.block_count() + additions.size() > limit) return false;
    }
    return true;
  }
  void insert(SparseVoxelMap& map, const std::vector<MapPoint>& points) {
    for (const auto& p : points) (void)map.insert(p);
    (void)map.take_changes();
  }
  void process(const Scan& scan) {
    ++stats_.scans_processed;
    if (scan.points.size() < config_.minimum_scan_points) { reject_scan("Insufficient scan points"); return; }
    if (config_.use_imu && !imu_.ready()) {
      reject_scan("Keep sensor still: waiting for IMU initialization"); return;
    }
    if (config_.use_imu && scan.begin < imu_.initialized_at()) {
      reject_scan("Scan precedes stationary IMU initialization"); return;
    }
    if (scan.end < scan.begin || scan.end - scan.begin > config_.max_scan_duration_ns) {
      reject_scan("Invalid scan duration"); return;
    }
    std::vector<ImuTracker::Knot> path;
    if (config_.use_imu) {
      auto covered = imu_.path(scan.begin, scan.end);
      if (!covered) { reject_scan("Missing IMU coverage or excessive IMU gap"); return; }
      path = std::move(*covered);
    }
    PoseEstimate predicted;
    predicted.timestamp = {scan.end, clock_};
    predicted.state = TrackingState::Tracking;
    if (pose_) {
      if (scan.end <= pose_->timestamp.nanoseconds) { reject_scan("Non-increasing scan time"); return; }
      const double dt = static_cast<double>(scan.end - pose_->timestamp.nanoseconds) * 1e-9;
      if (dt > config_.maximum_prediction_interval_s) {
        std::string message = "Tracking gap too large; reset SLAM to start a new local map";
        if (!tracking_failure_cause_.empty()) message += "; first rejection: " + tracking_failure_cause_;
        reject_scan(std::move(message)); return;
      }
      Rotation r = rotation(*pose_);
      if (config_.use_imu) {
        const auto between = imu_.path(pose_->timestamp.nanoseconds, scan.end);
        if (!between) { reject_scan("Missing IMU between accepted scans"); return; }
        r = (r * between->back().rotation).normalized();
      }
      assign(predicted, position(*pose_) + velocity_ * dt, r);
    } else if (config_.use_imu) {
      Rotation r = imu_.initial_rotation();
      if (scan.end >= imu_.initialized_at()) {
        const auto between = imu_.path(imu_.initialized_at(), scan.end);
        if (!between) { reject_scan("Missing initial IMU coverage"); return; }
        r = (r * between->back().rotation).normalized();
      }
      assign(predicted, Vector::Zero(), r);
    }
    auto points = deskew(scan, predicted, path);
    if (points.size() < config_.minimum_scan_points) { reject_scan("Insufficient downsampled scan points"); return; }
    const bool seed = !pose_;
    if (seed) {
      auto initial = world_points(points, predicted);
      if (!fits(matching_, initial, config_.max_matching_blocks)) {
        fail("Matching map block budget exceeded"); return;
      }
      insert(matching_, initial);
    }
    const auto aligned = registration_.align(points, predicted, matching_);
    stats_.last_rmse_m = aligned.rmse_m;
    stats_.last_correspondences = aligned.correspondences;
    if (!aligned.accepted) {
      if (seed) matching_.clear();
      reject_scan(registration_failure_message(aligned, config_.registration));
      return;
    }
    PoseEstimate accepted = aligned.pose;
    accepted.timestamp = {scan.end, clock_};
    accepted.state = TrackingState::Tracking;
    // The seed tests geometric observability, not independent tracking accuracy.
    if (seed) accepted = predicted;
    Vector new_velocity = Vector::Zero();
    if (pose_) {
      const double dt = static_cast<double>(scan.end - pose_->timestamp.nanoseconds) * 1e-9;
      new_velocity = (position(accepted) - position(*pose_)) / dt;
      if (new_velocity.norm() > config_.maximum_speed_m_s) {
        reject_scan("Estimated speed exceeds configured motion limit"); return;
      }
    }
    stats_.colored_points += colorizer_.colorize(points, accepted.timestamp, scan.begin);
    const auto transformed = world_points(points, accepted);
    const bool new_keyframe = !matching_pose_ ||
        (position(accepted) - position(*matching_pose_)).norm() >= config_.matching_keyframe_translation_m ||
        rotation(accepted).angularDistance(rotation(*matching_pose_)) >= config_.matching_keyframe_rotation_rad;
    if (!fits(canvas_, transformed, config_.max_canvas_blocks) ||
        ((new_keyframe || config_.temporal_canvas.enabled) &&
         !fits(matching_, transformed, config_.max_matching_blocks)) ||
        trajectory_.size() >= config_.max_trajectory_poses) {
      if (seed) matching_.clear();
      fail("SLAM map or trajectory budget exceeded; save and start a new session"); return;
    }
    bool temporal_paused = false;
    if (config_.temporal_canvas.enabled) {
      std::vector<RayObservation> rays;
      rays.reserve(transformed.size());
      const Rotation correction = rotation(accepted) * rotation(predicted).conjugate();
      for (size_t i = 0; i < points.size(); ++i) {
        const double dt = static_cast<double>(scan.end - scan.begin - points[i].offset_ns) * 1e-9;
        const Vector origin = position(accepted) - correction * velocity_ * dt;
        rays.push_back({{.x_m = origin.x(), .y_m = origin.y(), .z_m = origin.z()}, transformed[i]});
      }
      const bool reliable = seed ||
          (aligned.rmse_m <= config_.temporal_canvas.maximum_pose_rmse_m &&
           aligned.condition_number <= config_.temporal_canvas.maximum_pose_condition_number);
      temporal_.update(scan.end, rays, reliable, canvas_, matching_);
      temporal_paused = !reliable;
      const auto temporal_stats = temporal_.stats();
      stats_.candidate_voxels = temporal_stats.candidate_cells;
      stats_.provisional_matching_voxels = temporal_stats.provisional_matching_cells;
      stats_.cleared_voxels = temporal_stats.cleared_cells;
      stats_.contradicted_candidate_voxels = temporal_stats.contradicted_candidates;
      stats_.expired_candidate_voxels = temporal_stats.expired_candidates;
      stats_.temporal_skipped_scans = temporal_stats.skipped_scans;
      stats_.color_conflicts = temporal_stats.color_conflicts;
      (void)canvas_.take_changes();
      (void)matching_.take_changes();
    } else {
      insert(canvas_, transformed);
      if (new_keyframe && !seed) insert(matching_, transformed);
    }
    if (new_keyframe) matching_pose_ = accepted;
    velocity_ = seed ? Vector::Zero().eval()
                    : ((1.0 - config_.velocity_smoothing) * velocity_ +
                        config_.velocity_smoothing * new_velocity).eval();
    pose_ = accepted; trajectory_.push_back(accepted);
    ++stats_.scans_accepted;
    stats_.map_voxels = canvas_.voxel_count(); stats_.map_blocks = canvas_.block_count();
    stats_.state = TrackingState::Tracking;
    tracking_failure_cause_.clear();
    stats_.message = seed ? "Local map initialized; initial yaw and position are arbitrary"
                         : "Tracking against voxel map";
    if (seed && config_.temporal_canvas.enabled)
      stats_.message = "Local reference initialized; canvas awaiting temporal confirmation; origin/yaw arbitrary";
    if (temporal_paused)
      stats_.message = "Pose accepted; temporal map frozen: registration quality below mapping threshold";
  }
  VoxelSlamConfig config_;
  mutable std::mutex state_mutex_, queue_mutex_;
  std::mutex finish_mutex_;
  std::condition_variable wake_, idle_;
  std::deque<QueuedEvent> queue_;
  size_t queued_bytes_ = 0, peak_queued_bytes_ = 0;
  bool stopping_ = false, busy_ = false, finished_ = false;
  std::atomic_bool overflow_{false};
  std::atomic_uint64_t rejected_input_{0};
  std::thread worker_;
  SparseVoxelMap canvas_, matching_;
  VoxelRegistration registration_;
  ImuTracker imu_;
  TemporalCanvas temporal_;
  CameraColorizer colorizer_;
  Scan assembling_;
  std::deque<Scan> pending_;
  std::optional<uint32_t> sensor_id_;
  timesync::ClockSource clock_ = timesync::ClockSource::Unknown;
  std::optional<uint64_t> last_lidar_time_;
  std::optional<PoseEstimate> pose_;
  std::optional<PoseEstimate> matching_pose_;
  std::vector<PoseEstimate> trajectory_;
  Vector velocity_ = Vector::Zero();
  SlamStats stats_;
  uint64_t latest_measurement_ns_ = 0;
  /// Retain the first rejection in an uninterrupted tracking loss. Later gap
  /// guards must not hide the event that actually caused tracking to stop.
  std::string tracking_failure_cause_;
};

VoxelSlamEngine::VoxelSlamEngine(VoxelSlamConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}
VoxelSlamEngine::~VoxelSlamEngine() = default;
void VoxelSlamEngine::on_lidar_frame(const sensors::LidarFrame& v) { impl_->enqueue(v); }
bool VoxelSlamEngine::submit_camera_image(sensors::CameraRgbFrame frame) { return impl_->camera(std::move(frame)); }
void VoxelSlamEngine::on_imu_sample(const imu::ImuSample& v) { impl_->enqueue(v); }
void VoxelSlamEngine::reset() {
  const auto config = impl_->config();
  impl_->finish(); impl_ = std::make_unique<Impl>(config);
}
void VoxelSlamEngine::wait_until_idle() { impl_->wait_until_idle(); }
void VoxelSlamEngine::finish() { impl_->finish(); }
std::optional<PoseEstimate> VoxelSlamEngine::latest_pose() const { return impl_->pose(); }
SlamStats VoxelSlamEngine::stats() const { return impl_->stats(); }
std::vector<MapPoint> VoxelSlamEngine::map_points(size_t max_points) const { return impl_->points(max_points); }
std::vector<PoseEstimate> VoxelSlamEngine::trajectory() const { return impl_->trajectory(); }
void VoxelSlamEngine::save_map(const std::filesystem::path& path) const { impl_->save(path); }
void VoxelSlamEngine::export_ply(const std::filesystem::path& path) const { impl_->ply(path); }
void VoxelSlamEngine::export_trajectory(const std::filesystem::path& path) const { impl_->csv(path); }
void VoxelSlamEngine::export_session_report(const std::filesystem::path& path) const { impl_->session_report(path); }
}  // namespace atlas::slam
