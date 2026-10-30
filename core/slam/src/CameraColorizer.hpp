#pragma once
#include <atlas/sensors/CameraRgbFrame.hpp>
#include <atlas/sensors/LidarFrame.hpp>
#include <atlas/slam/camera_fusion_config.hpp>
#include <deque>
#include <memory>
#include <mutex>
#include <span>

namespace atlas::slam {
class CameraColorizer {
 public:
  explicit CameraColorizer(CameraFusionConfig config);
  bool submit(sensors::CameraRgbFrame frame);
  size_t colorize(std::span<sensors::LidarPoint> points, timesync::SensorTimestamp scan_end,
                  uint64_t scan_begin_ns = 0) const;
 private:
  CameraFusionConfig config_;
  mutable std::mutex mutex_;
  std::deque<std::shared_ptr<const sensors::CameraRgbFrame>> frames_;
};
}
