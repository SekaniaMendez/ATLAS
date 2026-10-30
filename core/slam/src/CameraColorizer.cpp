#include "CameraColorizer.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace atlas::slam {
CameraColorizer::CameraColorizer(CameraFusionConfig config) : config_(config) {
  if (!config.maximum_frames || !config.maximum_image_pixels || !config.depth_cell_pixels ||
      !config.maximum_time_error_ns || config.maximum_time_error_ns > 1'000'000'000 ||
      config.maximum_wait_ns > 1'000'000'000 || !std::isfinite(config.occlusion_tolerance_m) ||
      config.occlusion_tolerance_m < 0)
    throw std::invalid_argument("Invalid camera fusion configuration");
}
bool CameraColorizer::submit(sensors::CameraRgbFrame frame) {
  const uint64_t pixels = uint64_t(frame.width) * frame.height;
  if (!frame.width || !frame.height || pixels > config_.maximum_image_pixels ||
      frame.rgb.size() != pixels * 3 || !frame.timestamp_ns) return false;
  for (double v : frame.intrinsics) if (!std::isfinite(v)) return false;
  if (frame.intrinsics[0] <= 0 || frame.intrinsics[1] <= 0 ||
      frame.intrinsics[2] < 0 || frame.intrinsics[2] >= frame.width ||
      frame.intrinsics[3] < 0 || frame.intrinsics[3] >= frame.height) return false;
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> r(frame.camera_from_lidar_rotation.data());
  const Eigen::Map<const Eigen::Vector3d> t(frame.camera_from_lidar_translation.data());
  if (!r.allFinite() || !t.allFinite() || t.norm() > 5 ||
      (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() > 1e-4 ||
      std::abs(r.determinant() - 1) > 1e-4) return false;
  auto owned = std::make_shared<const sensors::CameraRgbFrame>(std::move(frame));
  std::lock_guard lock(mutex_);
  if (!frames_.empty() && owned->timestamp_ns <= frames_.back()->timestamp_ns) return false;
  frames_.push_back(std::move(owned));
  while (frames_.size() > config_.maximum_frames) frames_.pop_front();
  return true;
}

size_t CameraColorizer::colorize(std::span<sensors::LidarPoint> points, timesync::SensorTimestamp time, uint64_t scan_begin_ns) const {
  for (auto& point : points) { point.has_color = false; point.color_timestamp_ns = 0; }
  if (!config_.clock_alignment_verified || time.source != config_.lidar_clock ||
      time.source == timesync::ClockSource::Unknown ||
      config_.clock_uncertainty_ns > config_.maximum_time_error_ns) return 0;
  std::shared_ptr<const sensors::CameraRgbFrame> image;
  uint64_t image_time_ns = 0;
  uint64_t best = config_.maximum_time_error_ns - config_.clock_uncertainty_ns + 1;
  {
    std::lock_guard lock(mutex_);
    for (const auto& frame : frames_) {
      if (!frame->calibration_verified || !frame->tracking_normal) continue;
      // Offset arithmetic is checked without signed overflow, including INT64_MIN.
      const auto offset = config_.phone_to_lidar_offset_ns;
      uint64_t aligned = frame->timestamp_ns;
      if (offset >= 0) {
        if (aligned > UINT64_MAX - uint64_t(offset)) continue;
        aligned += uint64_t(offset);
      } else {
        const uint64_t magnitude = uint64_t(-(offset + 1)) + 1;
        if (aligned < magnitude) continue;
        aligned -= magnitude;
      }
      const uint64_t distance = aligned > time.nanoseconds ? aligned - time.nanoseconds : time.nanoseconds - aligned;
      if (distance < best) { best = distance; image = frame; image_time_ns = aligned; }
    }
  }
  if (!image) return 0;
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> r(image->camera_from_lidar_rotation.data());
  const Eigen::Map<const Eigen::Vector3d> t(image->camera_from_lidar_translation.data());
  struct Projection { size_t pixel, cell; double depth; };
  std::vector<Projection> projected(points.size(), {SIZE_MAX, 0, 0});
  const size_t cell_size = config_.depth_cell_pixels;
  const size_t columns = (image->width + cell_size - 1) / cell_size;
  std::vector<double> depth(columns * ((image->height + cell_size - 1) / cell_size),
                            std::numeric_limits<double>::infinity());
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& p = points[i];
    if (scan_begin_ns) {
      if (scan_begin_ns > UINT64_MAX - p.offset_ns) continue;
      const auto acquired = scan_begin_ns + p.offset_ns;
      const auto distance = acquired > image_time_ns ? acquired - image_time_ns : image_time_ns - acquired;
      if (distance > config_.maximum_time_error_ns - config_.clock_uncertainty_ns) continue;
    }
    const Eigen::Vector3d camera = r * Eigen::Vector3d(p.x_m, p.y_m, p.z_m) + t;
    const double z = -camera.z();
    if (!camera.allFinite() || z <= 0.05) continue;
    const double u = image->intrinsics[0] * camera.x() / z + image->intrinsics[2];
    const double v = image->intrinsics[3] - image->intrinsics[1] * camera.y() / z;
    if (!(u >= 0 && v >= 0 && u < image->width && v < image->height)) continue;
    const size_t x = static_cast<size_t>(u), y = static_cast<size_t>(v);
    const size_t cell = (y / cell_size) * columns + x / cell_size;
    projected[i] = {y * image->width + x, cell, z};
    depth[cell] = std::min(depth[cell], z);
  }
  size_t colored = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& p = projected[i];
    if (p.pixel == SIZE_MAX || p.depth > depth[p.cell] + config_.occlusion_tolerance_m) continue;
    std::copy_n(image->rgb.data() + p.pixel * 3, 3, points[i].rgb.begin());
    points[i].has_color = true;
    points[i].color_timestamp_ns = image->timestamp_ns;
    ++colored;
  }
  return colored;
}
}
