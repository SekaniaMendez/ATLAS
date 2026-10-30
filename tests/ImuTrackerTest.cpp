#include "ImuTracker.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using atlas::slam::ImuTracker;
using atlas::slam::VoxelSlamConfig;
constexpr uint64_t start_ns = 1'000'000'000;
constexpr uint64_t period_ns = 5'000'000;
const Eigen::Vector3d measured_bias(0.0296, -0.0117, 0.0168);

void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

atlas::imu::ImuSample sample(uint64_t timestamp_ns, const Eigen::Vector3d& gyro,
                            const Eigen::Vector3d& accel = {0, 0, 9.80665}) {
  atlas::imu::ImuSample result;
  result.timestamp = {timestamp_ns, atlas::timesync::ClockSource::Unsynchronized};
  result.gyro_x_rad_s = static_cast<float>(gyro.x());
  result.gyro_y_rad_s = static_cast<float>(gyro.y());
  result.gyro_z_rad_s = static_cast<float>(gyro.z());
  result.accel_x_m_s2 = static_cast<float>(accel.x());
  result.accel_y_m_s2 = static_cast<float>(accel.y());
  result.accel_z_m_s2 = static_cast<float>(accel.z());
  return result;
}

void stable_bias_is_estimated_and_removed() {
  const VoxelSlamConfig config;
  check(measured_bias.norm() > config.stationary_gyro_limit_rad_s,
        "regression must exercise bias above the old raw-gyro gate");
  ImuTracker tracker(config);
  const double roll = 0.2;
  const Eigen::Vector3d gravity(0, std::sin(roll) * config.gravity_m_s2,
                                std::cos(roll) * config.gravity_m_s2);
  // Deterministic noise comparable to the captured stationary sensor. Test
  // both the statistical acceptance and bias removal in later gyro integration.
  for (size_t i = 0; i <= 100; ++i) {
    const double sign = i % 2 == 0 ? 1 : -1;
    tracker.add(sample(start_ns + i * period_ns,
        measured_bias + sign * Eigen::Vector3d(0.0025, -0.0038, 0.0018),
        gravity + Eigen::Vector3d(sign * 0.13, 0, 0)));
    if (i < 100) check(!tracker.ready(), "accepted an incomplete initialization window");
  }
  check(tracker.ready(), "stable measured bias prevented initialization");
  check(tracker.initialized_at() == start_ns + 100 * period_ns,
        "stationary initialization timestamp is incorrect");
  const Eigen::Quaterniond expected(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  check(tracker.initial_rotation().angularDistance(expected) < 0.001,
        "stationary accelerometer did not recover gravity alignment");
  for (size_t i = 101; i <= 300; ++i)
    tracker.add(sample(start_ns + i * period_ns, measured_bias, gravity));
  const auto path = tracker.path(start_ns + 101 * period_ns, start_ns + 300 * period_ns);
  check(path.has_value(), "initialized IMU did not cover the subsequent stationary interval");
  check(path->back().rotation.angularDistance(Eigen::Quaterniond::Identity()) < 0.001,
        "estimated constant bias caused spurious rotation");
}

void variable_motion_and_outliers_are_not_bias() {
  ImuTracker varying(VoxelSlamConfig{}), outliers(VoxelSlamConfig{});
  for (size_t i = 0; i < 400; ++i) {
    // Every sample is below 0.03 rad/s; the independent dispersion gate must
    // nevertheless reject this varying rotation.
    varying.add(sample(start_ns + i * period_ns, {i % 2 == 0 ? 0.02 : -0.02, 0, 0}));
    // Sparse peaks have RMS below 0.01 but exceed the individual residual gate.
    outliers.add(sample(start_ns + i * period_ns, {i % 20 == 0 ? 0.04 : 0, 0, 0}));
    check(!varying.ready(), "varying motion was calibrated as gyro bias");
    check(!outliers.ready(), "sparse rotation peaks were calibrated as gyro bias");
  }
}

void sustained_rotation_is_rejected_then_a_stationary_window_recovers() {
  ImuTracker tracker(VoxelSlamConfig{});
  for (size_t i = 0; i < 200; ++i) {
    tracker.add(sample(start_ns + i * period_ns, {0, 0, 0.2}));
    check(!tracker.ready(), "strong constant yaw was absorbed into gyro bias");
  }
  for (size_t i = 200; i <= 300; ++i) {
    tracker.add(sample(start_ns + i * period_ns, measured_bias));
    if (i < 300) check(!tracker.ready(), "accepted a window still containing strong rotation");
  }
  check(tracker.ready(), "initialization did not recover once the sensor stopped");
  check(tracker.initialized_at() == start_ns + 300 * period_ns,
        "initialization retained samples from the moving interval");
}

void acceleration_and_timestamp_guards_remain_active() {
  ImuTracker accelerating(VoxelSlamConfig{}), wrong_gravity(VoxelSlamConfig{}), gaps(VoxelSlamConfig{});
  uint64_t gap_time = start_ns;
  for (size_t i = 0; i < 400; ++i) {
    accelerating.add(sample(start_ns + i * period_ns, measured_bias,
                           {i % 2 == 0 ? 0.22 : -0.22, 0, 9.80665}));
    wrong_gravity.add(sample(start_ns + i * period_ns, measured_bias, {0, 0, 8.5}));
    gap_time += i % 20 == 0 ? 40'000'000 : period_ns;
    gaps.add(sample(gap_time, measured_bias));
    check(!accelerating.ready(), "varying acceleration was accepted as stationary gravity");
    check(!wrong_gravity.ready(), "non-gravitational acceleration initialized attitude");
    check(!gaps.ready(), "IMU gaps were accepted during stationary initialization");
  }
}
}  // namespace

int main() {
  try {
    stable_bias_is_estimated_and_removed();
    variable_motion_and_outliers_are_not_bias();
    sustained_rotation_is_rejected_then_a_stationary_window_recovers();
    acceleration_and_timestamp_guards_remain_active();
    std::cout << "IMU bias, motion rejection and initialization guard tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
