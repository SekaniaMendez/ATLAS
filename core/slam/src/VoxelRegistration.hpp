#pragma once

#include <atlas/sensors/LidarFrame.hpp>
#include <atlas/slam/ISlamEngine.hpp>
#include <atlas/slam/SparseVoxelMap.hpp>
#include <atlas/slam/voxel_registration_config.hpp>
#include <cstddef>
#include <span>

namespace atlas::slam {

enum class RegistrationFailure {
  None,
  InvalidInput,
  EmptyMap,
  InsufficientPoints,
  InsufficientCorrespondences,
  DegenerateGeometry,
  NumericalFailure,
  ExcessiveCorrection,
  DidNotConverge,
  ExcessiveResidual,
};

struct RegistrationResult {
  /// The original prediction is returned on rejection; partial updates are not
  /// exposed as an accepted motion estimate.
  PoseEstimate pose;
  bool accepted = false;
  bool degenerate = false;
  RegistrationFailure failure = RegistrationFailure::None;
  /// Finite points actually selected for matching, before correspondence tests.
  size_t query_count = 0;
  size_t correspondences = 0;
  double rmse_m = 0.0;
  double condition_number = 0.0;
  size_t iterations = 0;
};

/// Geometric registration only. Points must already share the end-of-scan
/// sensor frame, and the map must remain unchanged throughout align().
/// It does not modify the map or fuse an IMU prior into its observability test.
class VoxelRegistration {
 public:
  explicit VoxelRegistration(VoxelRegistrationConfig config = {});
  [[nodiscard]] RegistrationResult align(std::span<const sensors::LidarPoint> points,
                                         const PoseEstimate& initial,
                                         const SparseVoxelMap& map) const;

 private:
  VoxelRegistrationConfig config_;
};

}  // namespace atlas::slam
