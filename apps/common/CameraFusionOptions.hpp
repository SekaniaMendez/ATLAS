#pragma once
#include <atlas/slam/camera_fusion_config.hpp>
#include <charconv>
#include <optional>
#include <stdexcept>
#include <string>

namespace atlas::apps {
struct CameraFusionOptions {
  std::optional<int64_t> offset;
  std::optional<uint64_t> uncertainty;
  std::optional<timesync::ClockSource> clock;
  template<class T> static T number(const std::string& text) {
    T value{};
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size())
      throw std::runtime_error("Invalid camera clock value: " + text);
    return value;
  }
  bool parse(const std::string& option, int& index, int argc, char** argv) {
    if (option != "--camera-offset-ns" && option != "--camera-uncertainty-ns" && option != "--camera-lidar-clock") return false;
    if (++index >= argc) throw std::runtime_error("Missing value for " + option);
    const std::string value = argv[index];
    if (option == "--camera-offset-ns") offset = number<int64_t>(value);
    else if (option == "--camera-uncertainty-ns") uncertainty = number<uint64_t>(value);
    else if (value == "unsynchronized") clock = timesync::ClockSource::Unsynchronized;
    else if (value == "ptp") clock = timesync::ClockSource::Ptp;
    else if (value == "gps") clock = timesync::ClockSource::Gps;
    else throw std::runtime_error("Camera LiDAR clock must be unsynchronized, ptp or gps");
    return true;
  }
  slam::CameraFusionConfig config(bool slam_enabled) const {
    slam::CameraFusionConfig result;
    if (!offset && !uncertainty && !clock) return result;
    if (!slam_enabled || !offset || !uncertainty || !clock)
      throw std::runtime_error("Camera alignment requires --slam and all three --camera-* options");
#ifndef ATLAS_HAS_UI_STREAM
    throw std::runtime_error("Camera fusion requires a build with ATLAS_BUILD_UI_STREAM=ON");
#endif
    if (*uncertainty > result.maximum_time_error_ns)
      throw std::runtime_error("Camera clock uncertainty exceeds the 15 ms fusion budget");
    result.clock_alignment_verified = true;
    result.phone_to_lidar_offset_ns = *offset;
    result.clock_uncertainty_ns = *uncertainty;
    result.lidar_clock = *clock;
    return result;
  }
};
}
