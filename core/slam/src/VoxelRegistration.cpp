#include "VoxelRegistration.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace atlas::slam {
namespace {

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

struct Transform {
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

struct Neighbor {
  double squared_distance;
  Eigen::Vector3d point;
};

struct Linearization {
  Matrix6 information = Matrix6::Zero();
  Vector6 gradient = Vector6::Zero();
  size_t count = 0;
  double squared_error = 0.0;
};

bool positive_finite(double value) { return std::isfinite(value) && value > 0.0; }

bool valid_config(const VoxelRegistrationConfig& c) {
  return c.max_iterations > 0 && c.max_iterations <= 100 &&
      c.max_query_points >= c.minimum_correspondences && c.max_query_points <= 1000000 &&
      c.minimum_correspondences >= 6 &&
      c.minimum_plane_neighbors >= 3 && c.nearest_neighbors >= c.minimum_plane_neighbors &&
      c.nearest_neighbors <= 512 &&
      c.neighbor_radius_voxels > 0 && c.neighbor_radius_voxels <= 8 &&
      positive_finite(c.max_correspondence_distance_m) &&
      positive_finite(c.max_plane_residual_m) &&
      positive_finite(c.max_planarity_ratio) && c.max_planarity_ratio < 1.0 &&
      positive_finite(c.minimum_plane_spread_m) &&
      positive_finite(c.huber_delta_m) &&
      positive_finite(c.convergence_translation_m) &&
      positive_finite(c.convergence_rotation_rad) &&
      positive_finite(c.max_translation_correction_m) &&
      positive_finite(c.max_rotation_correction_rad) &&
      c.max_rotation_correction_rad <= 3.14159265358979323846 &&
      positive_finite(c.max_rmse_m) &&
      positive_finite(c.minimum_correspondence_ratio) && c.minimum_correspondence_ratio <= 1.0 &&
      positive_finite(c.max_condition_number) && c.max_condition_number >= 1.0 &&
      positive_finite(c.minimum_information_eigenvalue) &&
      std::isfinite(c.max_correspondence_distance_m * c.max_correspondence_distance_m) &&
      std::isfinite(c.minimum_plane_spread_m * c.minimum_plane_spread_m);
}

std::optional<Transform> transform_of(const PoseEstimate& pose) {
  Transform transform{Eigen::Quaterniond(pose.qw, pose.qx, pose.qy, pose.qz),
                      Eigen::Vector3d(pose.x_m, pose.y_m, pose.z_m)};
  if (!transform.translation.allFinite() || !transform.rotation.coeffs().allFinite() ||
      !positive_finite(transform.rotation.squaredNorm())) return std::nullopt;
  transform.rotation.normalize();
  return transform;
}

PoseEstimate pose_of(const Transform& transform, const PoseEstimate& reference) {
  auto pose = reference;
  pose.x_m = transform.translation.x();
  pose.y_m = transform.translation.y();
  pose.z_m = transform.translation.z();
  pose.qx = transform.rotation.x();
  pose.qy = transform.rotation.y();
  pose.qz = transform.rotation.z();
  pose.qw = transform.rotation.w();
  return pose;
}

Linearization linearize(const std::vector<Eigen::Vector3d>& points,
                        const Transform& transform, const SparseVoxelMap& map,
                        const VoxelRegistrationConfig& config, double lever_arm) {
  Linearization result;
  const double max_distance_squared =
      config.max_correspondence_distance_m * config.max_correspondence_distance_m;
  const double minimum_spread_squared =
      config.minimum_plane_spread_m * config.minimum_plane_spread_m;
  std::vector<Neighbor> neighbors;
  for (const auto& point : points) {
    // A left rotation increment is applied around the sensor origin, not around
    // the project origin, so distant map coordinates cannot spoil conditioning.
    const Eigen::Vector3d rotated = transform.rotation * point;
    const Eigen::Vector3d world = rotated + transform.translation;
    const auto key = map.voxel_key_for({world.x(), world.y(), world.z()});
    if (!key) continue;
    neighbors.clear();
    for (const auto& cell : map.neighbors(*key, config.neighbor_radius_voxels)) {
      std::optional<Neighbor> representative;
      for (const auto& sample : cell.voxel->samples) {
        const Eigen::Vector3d candidate(sample.x_m, sample.y_m, sample.z_m);
        const double distance = (candidate - world).squaredNorm();
        if (std::isfinite(distance) && distance <= max_distance_squared &&
            (!representative || distance < representative->squared_distance))
          representative = Neighbor{distance, candidate};
      }
      // A repeatedly observed point can occupy every slot in a voxel's ring.
      // Counting those duplicates as k independent neighbors collapses the fit
      // onto a point/line. One measured representative per cell keeps spatial
      // support without averaging incompatible surfaces across a cell.
      if (representative) neighbors.push_back(*representative);
    }
    if (neighbors.size() < config.minimum_plane_neighbors) continue;
    const auto count = std::min(config.nearest_neighbors, neighbors.size());
    if (neighbors.size() > count) {
      std::nth_element(neighbors.begin(), neighbors.begin() + static_cast<std::ptrdiff_t>(count),
                       neighbors.end(), [](const Neighbor& a, const Neighbor& b) {
                         return a.squared_distance < b.squared_distance;
                       });
    }
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < count; ++i) center += neighbors[i].point;
    center /= static_cast<double>(count);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < count; ++i) {
      const Eigen::Vector3d difference = neighbors[i].point - center;
      covariance.noalias() += difference * difference.transpose();
    }
    covariance /= static_cast<double>(count);
    if (!covariance.allFinite()) continue;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    if (eigen.info() != Eigen::Success ||
        eigen.eigenvalues()[1] < minimum_spread_squared ||
        eigen.eigenvalues()[0] > config.max_planarity_ratio * eigen.eigenvalues()[1]) continue;
    const Eigen::Vector3d normal = eigen.eigenvectors().col(0);
    bool consistent_plane = true;
    for (size_t i = 0; i < count; ++i) {
      if (std::abs(normal.dot(neighbors[i].point - center)) > config.max_plane_residual_m) {
        consistent_plane = false;
        break;
      }
    }
    const double residual = normal.dot(world - center);
    if (!consistent_plane || !std::isfinite(residual) ||
        std::abs(residual) > config.max_plane_residual_m) continue;
    const double weight = std::abs(residual) <= config.huber_delta_m
        ? 1.0 : config.huber_delta_m / std::abs(residual);
    Vector6 jacobian;
    jacobian.head<3>() = rotated.cross(normal) / lever_arm;
    jacobian.tail<3>() = normal;
    result.information.noalias() += weight * jacobian * jacobian.transpose();
    result.gradient.noalias() += weight * jacobian * residual;
    result.squared_error += residual * residual;
    ++result.count;
  }
  return result;
}

bool inspect(const Linearization& linearization, size_t query_count,
             const VoxelRegistrationConfig& config, RegistrationResult& result,
             Eigen::SelfAdjointEigenSolver<Matrix6>& eigen) {
  result.correspondences = linearization.count;
  result.rmse_m = linearization.count
      ? std::sqrt(linearization.squared_error / static_cast<double>(linearization.count)) : 0.0;
  if (linearization.count < config.minimum_correspondences ||
      static_cast<double>(linearization.count) <
          static_cast<double>(query_count) * config.minimum_correspondence_ratio) {
    result.failure = RegistrationFailure::InsufficientCorrespondences;
    return false;
  }
  if (!linearization.information.allFinite() || !linearization.gradient.allFinite()) {
    result.failure = RegistrationFailure::NumericalFailure;
    return false;
  }
  eigen.compute(linearization.information / static_cast<double>(linearization.count));
  if (eigen.info() != Eigen::Success) {
    result.failure = RegistrationFailure::NumericalFailure;
    return false;
  }
  const double smallest = eigen.eigenvalues()[0];
  const double largest = eigen.eigenvalues()[5];
  result.condition_number = smallest > 0.0 ? largest / smallest
                                          : std::numeric_limits<double>::infinity();
  result.degenerate = smallest < config.minimum_information_eigenvalue ||
      !std::isfinite(result.condition_number) || result.condition_number > config.max_condition_number;
  if (result.degenerate) result.failure = RegistrationFailure::DegenerateGeometry;
  return !result.degenerate;
}

}  // namespace

VoxelRegistration::VoxelRegistration(VoxelRegistrationConfig config) : config_(config) {
  if (!valid_config(config_)) throw std::invalid_argument("Invalid voxel registration configuration");
}

RegistrationResult VoxelRegistration::align(std::span<const sensors::LidarPoint> points,
                                           const PoseEstimate& initial,
                                           const SparseVoxelMap& map) const {
  RegistrationResult result;
  result.pose = initial;
  const auto predicted = transform_of(initial);
  if (!predicted) {
    result.failure = RegistrationFailure::InvalidInput;
    return result;
  }
  if (map.voxel_count() == 0) {
    result.failure = RegistrationFailure::EmptyMap;
    return result;
  }
  if (points.size() < config_.minimum_correspondences) {
    result.failure = RegistrationFailure::InsufficientPoints;
    return result;
  }

  std::vector<Eigen::Vector3d> queries;
  const auto desired = std::min(points.size(), config_.max_query_points);
  queries.reserve(desired);
  double squared_range_sum = 0.0;
  for (size_t i = 0; i < desired; ++i) {
    // Spread a bounded sample over the entire scan, including temporal coverage.
    const auto index = static_cast<size_t>(
        (static_cast<long double>(i) * static_cast<long double>(points.size())) / desired);
    const auto& point = points[index];
    const Eigen::Vector3d value(point.x_m, point.y_m, point.z_m);
    if (!value.allFinite()) continue;
    squared_range_sum += value.squaredNorm();
    queries.push_back(value);
  }
  result.query_count = queries.size();
  if (queries.size() < config_.minimum_correspondences) {
    result.failure = RegistrationFailure::InsufficientPoints;
    return result;
  }
  const double lever_arm = std::max(1.0, std::sqrt(squared_range_sum / queries.size()));
  if (!std::isfinite(lever_arm)) {
    result.failure = RegistrationFailure::NumericalFailure;
    return result;
  }

  auto candidate = *predicted;
  bool converged = false;
  for (size_t iteration = 0; iteration < config_.max_iterations; ++iteration) {
    result.iterations = iteration + 1;
    const auto system = linearize(queries, candidate, map, config_, lever_arm);
    Eigen::SelfAdjointEigenSolver<Matrix6> eigen;
    if (!inspect(system, queries.size(), config_, result, eigen)) return result;
    // The rank test above is performed without a prior or damping term. An IMU
    // prediction must not turn an unobservable plane into six measured axes.
    const Vector6 mean_gradient = system.gradient / static_cast<double>(system.count);
    const Vector6 step = -eigen.eigenvectors() *
        (eigen.eigenvectors().transpose() * mean_gradient).cwiseQuotient(eigen.eigenvalues());
    if (!step.allFinite()) {
      result.failure = RegistrationFailure::NumericalFailure;
      return result;
    }
    const Eigen::Vector3d angular_step = step.head<3>() / lever_arm;
    const Eigen::Vector3d translation_step = step.tail<3>();
    const double angle = angular_step.norm();
    if (angle > config_.max_rotation_correction_rad ||
        translation_step.norm() > config_.max_translation_correction_m) {
      result.failure = RegistrationFailure::ExcessiveCorrection;
      return result;
    }
    if (angle > 0.0) {
      candidate.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angle, angular_step / angle)) *
          candidate.rotation;
      candidate.rotation.normalize();
    }
    candidate.translation += translation_step;
    if (!candidate.translation.allFinite()) {
      result.failure = RegistrationFailure::NumericalFailure;
      return result;
    }
    if ((candidate.translation - predicted->translation).norm() > config_.max_translation_correction_m ||
        candidate.rotation.angularDistance(predicted->rotation) > config_.max_rotation_correction_rad) {
      result.failure = RegistrationFailure::ExcessiveCorrection;
      return result;
    }
    if (translation_step.norm() <= config_.convergence_translation_m &&
        angle <= config_.convergence_rotation_rad) {
      converged = true;
      break;
    }
  }
  if (!converged) {
    result.failure = RegistrationFailure::DidNotConverge;
    return result;
  }
  // Validate at the resulting pose, with freshly associated neighbors.
  const auto final_system = linearize(queries, candidate, map, config_, lever_arm);
  Eigen::SelfAdjointEigenSolver<Matrix6> final_eigen;
  if (!inspect(final_system, queries.size(), config_, result, final_eigen)) return result;
  if (!std::isfinite(result.rmse_m) || result.rmse_m > config_.max_rmse_m) {
    result.failure = RegistrationFailure::ExcessiveResidual;
    return result;
  }
  result.pose = pose_of(candidate, initial);
  result.accepted = true;
  return result;
}

}  // namespace atlas::slam
