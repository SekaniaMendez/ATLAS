#pragma once

#include <atlas/imu/ImuSample.hpp>
#include <atlas/slam/voxel_slam_config.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <deque>
#include <optional>
#include <vector>

namespace atlas::slam {

inline Eigen::Quaterniond rotation_increment(const Eigen::Vector3d& angle) {
  const double norm = angle.norm();
  return norm > 1e-12 ? Eigen::Quaterniond(Eigen::AngleAxisd(norm, angle / norm))
                      : Eigen::Quaterniond::Identity();
}

/// Gyro-only relative attitude. Accelerometer is used exclusively for stationary
/// gravity initialization, never double-integrated into an unbounded position.
class ImuTracker {
 public:
  struct Knot {
    uint64_t time_ns;
    Eigen::Quaterniond rotation;
  };
  explicit ImuTracker(const VoxelSlamConfig& config)
      : config_(config),
        imu_to_lidar_(config.imu_to_lidar_rotation_xyzw[3],
                      config.imu_to_lidar_rotation_xyzw[0],
                      config.imu_to_lidar_rotation_xyzw[1],
                      config.imu_to_lidar_rotation_xyzw[2]) {
    imu_to_lidar_.normalize();
  }
  bool add(const imu::ImuSample& sample) {
    const Eigen::Vector3d gyro = imu_to_lidar_ *
        Eigen::Vector3d(sample.gyro_x_rad_s, sample.gyro_y_rad_s, sample.gyro_z_rad_s);
    const Eigen::Vector3d accel = imu_to_lidar_ *
        Eigen::Vector3d(sample.accel_x_m_s2, sample.accel_y_m_s2, sample.accel_z_m_s2);
    if (!gyro.allFinite() || !accel.allFinite() ||
        (!samples_.empty() && sample.timestamp.nanoseconds <= samples_.back().time_ns)) return false;
    samples_.push_back({sample.timestamp.nanoseconds, gyro, accel});
    if (!ready_) initialize();
    while (samples_.size() > config_.maximum_imu_samples ||
           (samples_.size() > 2 && samples_.back().time_ns - samples_[1].time_ns >
                                       config_.imu_history_ns)) samples_.pop_front();
    return true;
  }
  bool ready() const { return ready_; }
  uint64_t initialized_at() const { return initialized_at_; }
  uint64_t latest_time() const { return samples_.empty() ? 0 : samples_.back().time_ns; }
  const Eigen::Quaterniond& initial_rotation() const { return initial_rotation_; }

  std::optional<std::vector<Knot>> path(uint64_t start, uint64_t end) const {
    if (!ready_ || end < start || samples_.empty() || samples_.front().time_ns > start ||
        samples_.back().time_ns < end) return std::nullopt;
    auto first = std::upper_bound(samples_.begin(), samples_.end(), start,
        [](uint64_t t, const Sample& s) { return t < s.time_ns; });
    if (first == samples_.begin()) return std::nullopt;
    --first;
    std::vector<Knot> result{{start, Eigen::Quaterniond::Identity()}};
    uint64_t time = start;
    auto current = first;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    while (time < end) {
      const auto next = std::next(current);
      if (next == samples_.end()) return std::nullopt;
      const uint64_t gap = next->time_ns - current->time_ns;
      if (gap > config_.maximum_imu_gap_ns) return std::nullopt;
      const uint64_t stop = std::min(end, next->time_ns);
      const double a = static_cast<double>(time - current->time_ns) / static_cast<double>(gap);
      const double b = static_cast<double>(stop - current->time_ns) / static_cast<double>(gap);
      const Eigen::Vector3d rate = current->gyro +
          ((a + b) * 0.5) * (next->gyro - current->gyro) - bias_;
      q = (q * rotation_increment(rate * (static_cast<double>(stop - time) * 1e-9))).normalized();
      result.push_back({stop, q});
      time = stop;
      current = next;
    }
    return result;
  }
  static Eigen::Quaterniond at(const std::vector<Knot>& path, uint64_t time) {
    const auto upper = std::lower_bound(path.begin(), path.end(), time,
        [](const Knot& k, uint64_t t) { return k.time_ns < t; });
    if (upper == path.begin()) return upper->rotation;
    if (upper == path.end()) return path.back().rotation;
    const auto& lower = *std::prev(upper);
    const double alpha = static_cast<double>(time - lower.time_ns) /
                         static_cast<double>(upper->time_ns - lower.time_ns);
    return lower.rotation.slerp(alpha, upper->rotation);
  }

 private:
  struct Sample {
    uint64_t time_ns;
    Eigen::Vector3d gyro, accel;
  };
  void initialize() {
    if (samples_.size() < config_.minimum_initialization_imu_samples) return;
    const uint64_t end = samples_.back().time_ns;
    auto begin = samples_.end() - 1;
    while (begin != samples_.begin() && end - begin->time_ns < config_.imu_initialization_ns) --begin;
    if (end - begin->time_ns < config_.imu_initialization_ns ||
        static_cast<size_t>(samples_.end() - begin) < config_.minimum_initialization_imu_samples) return;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero(), accel = Eigen::Vector3d::Zero();
    for (auto it = begin; it != samples_.end(); ++it) {
      if (std::abs(it->accel.norm() - config_.gravity_m_s2) >
              config_.stationary_accel_tolerance_m_s2 ||
          (it != begin && it->time_ns - std::prev(it)->time_ns > config_.maximum_imu_gap_ns)) return;
      gyro += it->gyro; accel += it->accel;
    }
    const double count = static_cast<double>(samples_.end() - begin);
    gyro /= count; accel /= count;
    // A stable nonzero gyro offset is what this window estimates. Gate its
    // magnitude separately from motion/noise, so bias does not reject itself.
    // The absolute bound still rejects strong constant rotation, which has no
    // variance and could otherwise be absorbed into the bias estimate.
    if (gyro.norm() > config_.maximum_initialization_gyro_bias_rad_s) return;
    double gyro_variance = 0, accel_variance = 0;
    for (auto it = begin; it != samples_.end(); ++it) {
      const Eigen::Vector3d residual = it->gyro - gyro;
      if (residual.norm() > config_.stationary_gyro_limit_rad_s) return;
      gyro_variance += residual.squaredNorm();
      accel_variance += (it->accel - accel).squaredNorm();
    }
    if (std::sqrt(gyro_variance / count) > config_.stationary_gyro_stddev_rad_s ||
        std::sqrt(accel_variance / count) > config_.stationary_accel_stddev_m_s2 ||
        accel.norm() < config_.gravity_m_s2 * 0.5) return;
    bias_ = gyro;
    initial_rotation_ = Eigen::Quaterniond::FromTwoVectors(accel, Eigen::Vector3d::UnitZ()).normalized();
    initialized_at_ = end;
    ready_ = true;
  }
  VoxelSlamConfig config_;
  Eigen::Quaterniond imu_to_lidar_;
  std::deque<Sample> samples_;
  Eigen::Vector3d bias_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond initial_rotation_ = Eigen::Quaterniond::Identity();
  uint64_t initialized_at_ = 0;
  bool ready_ = false;
};

}  // namespace atlas::slam
