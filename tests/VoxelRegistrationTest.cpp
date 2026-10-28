#include "../core/slam/src/VoxelRegistration.hpp"
#include "StorageTestSupport.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace {

using namespace atlas;
std::vector<sensors::LidarPoint> observe(const std::vector<slam::MapPoint>& world,
                                       const Eigen::Quaterniond& rotation,
                                       const Eigen::Vector3d& translation) {
  std::vector<sensors::LidarPoint> result;
  for (const auto& point : world) {
    const Eigen::Vector3d sensor = rotation.conjugate() *
        (Eigen::Vector3d(point.x_m, point.y_m, point.z_m) - translation);
    result.push_back({.x_m = static_cast<float>(sensor.x()),
                      .y_m = static_cast<float>(sensor.y()),
                      .z_m = static_cast<float>(sensor.z())});
  }
  return result;
}

}  // namespace

int main() {
  try {
    std::vector<slam::MapPoint> room, floor;
    for (int a = -15; a <= 15; ++a) {
      for (int b = -15; b <= 15; ++b) {
        const double x = 0.2 * a, y = 0.2 * b;
        room.push_back({x, y, -1.4});
        floor.push_back({x, y, -1.4});
        room.push_back({3.0, x, y + 1.6});
        room.push_back({x, 3.0, y + 1.6});
      }
    }
    slam::SparseVoxelMap map({.voxel_size_m = 0.2, .max_samples_per_voxel = 8});
    for (const auto& point : room) map.insert(point);
    const auto revision_before = map.revision();
    const Eigen::Vector3d translation(0.12, -0.08, 0.06);
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(
        0.025, Eigen::Vector3d(1.0, -2.0, 3.0).normalized()));
    auto scan = observe(room, rotation, translation);
    // Unrelated distant observations exercise correspondence rejection.
    for (size_t i = 0; i < scan.size(); i += 13) scan[i].z_m += 20.0F;
    const slam::VoxelRegistration registration;
    const auto result = registration.align(scan, {}, map);
    REQUIRE(result.accepted);
    REQUIRE(result.failure == slam::RegistrationFailure::None);
    REQUIRE(result.query_count == 2500);
    REQUIRE(!result.degenerate);
    REQUIRE(result.correspondences >= 40);
    REQUIRE(result.rmse_m < 0.005);
    const Eigen::Vector3d recovered(result.pose.x_m, result.pose.y_m, result.pose.z_m);
    const Eigen::Quaterniond recovered_rotation(
        result.pose.qw, result.pose.qx, result.pose.qy, result.pose.qz);
    REQUIRE((recovered - translation).norm() < 0.005);
    REQUIRE(recovered_rotation.angularDistance(rotation) < 0.002);
    REQUIRE(map.revision() == revision_before);
    // Stationary scans must not gradually destroy planar support when each
    // voxel's recent-sample ring fills with repeated observations.
    for (int repeat = 0; repeat < 12; ++repeat)
      for (const auto& point : room) map.insert(point);
    const auto repeated = registration.align(scan, {}, map);
    REQUIRE(repeated.accepted);
    REQUIRE(repeated.correspondences >= result.correspondences * 9 / 10);
    const Eigen::Vector3d repeated_position(repeated.pose.x_m, repeated.pose.y_m, repeated.pose.z_m);
    REQUIRE((repeated_position - translation).norm() < 0.005);
    const slam::VoxelRegistration limited({.max_translation_correction_m = 0.02});
    const auto too_far = limited.align(scan, {}, map);
    REQUIRE(!too_far.accepted);
    REQUIRE(too_far.failure == slam::RegistrationFailure::ExcessiveCorrection);
    REQUIRE(too_far.pose.x_m == 0.0);
    const slam::VoxelRegistration one_iteration({.max_iterations = 1});
    const auto unconverged = one_iteration.align(scan, {}, map);
    REQUIRE(!unconverged.accepted);
    REQUIRE(unconverged.failure == slam::RegistrationFailure::DidNotConverge);

    auto outside_map = scan;
    for (auto& point : outside_map) point.x_m += 30.0F;
    const auto unsupported = registration.align(outside_map, {}, map);
    REQUIRE(!unsupported.accepted);
    REQUIRE(unsupported.failure == slam::RegistrationFailure::InsufficientCorrespondences);
    REQUIRE(unsupported.query_count == 2500);
    REQUIRE(unsupported.correspondences == 0);

    slam::SparseVoxelMap wall_map({.voxel_size_m = 0.2, .max_samples_per_voxel = 8});
    for (const auto& point : floor) wall_map.insert(point);
    const auto wall_scan = observe(floor, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    const auto degenerate = registration.align(wall_scan, {}, wall_map);
    REQUIRE(!degenerate.accepted);
    REQUIRE(degenerate.degenerate);
    REQUIRE(degenerate.failure == slam::RegistrationFailure::DegenerateGeometry);
    REQUIRE(degenerate.pose.x_m == 0.0);

    slam::PoseEstimate invalid_pose;
    invalid_pose.qw = std::numeric_limits<double>::quiet_NaN();
    REQUIRE(registration.align(scan, invalid_pose, map).failure == slam::RegistrationFailure::InvalidInput);
    REQUIRE(registration.align({}, {}, map).failure == slam::RegistrationFailure::InsufficientPoints);
    const slam::SparseVoxelMap empty_map;
    REQUIRE(registration.align(scan, {}, empty_map).failure == slam::RegistrationFailure::EmptyMap);
    rejects([] { slam::VoxelRegistration invalid({.neighbor_radius_voxels = 9}); });
    rejects([] { slam::VoxelRegistration invalid({.huber_delta_m = 0.0}); });
    rejects([] {
      slam::VoxelRegistration invalid({
          .max_condition_number = std::numeric_limits<double>::infinity()});
    });
    std::cout << "Registration recovers six-axis motion, rejects a single plane and preserves the map\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
