#include <atlas/storage/Capture.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
namespace fs = std::filesystem;
constexpr uint64_t kSecond = 1'000'000'000ULL;
struct Options {
  fs::path input, output;
  std::optional<uint32_t> sensor;
  uint64_t start = 0, duration = 5 * kSecond;
  double min_range = 0.2;
  bool inspect = false, keep_low_confidence = false, imu_only = false;
};
uint64_t seconds(const std::string& text, bool allow_zero) {
  size_t parsed = 0;
  const auto value = std::stold(text, &parsed);
  const auto rounded_ns = std::round(value * kSecond);
  if (parsed != text.size() || !std::isfinite(value) || value < 0 ||
      (!allow_zero && value == 0) || rounded_ns >= std::ldexp(1.0L, 64))
    throw std::runtime_error("Invalid seconds: " + text);
  const auto ns = static_cast<uint64_t>(rounded_ns);
  if (!allow_zero && ns == 0) throw std::runtime_error("Duration must be at least one nanosecond");
  return ns;
}
Options arguments(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help") {
      std::cout << "atlas_calibration_export --input RAW_DIRECTORY --inspect\n"
                   "atlas_calibration_export --input RAW_DIRECTORY --output NEW_DIRECTORY\n"
                   "  [--start-s 0] [--duration-s 5] [--sensor-id ID]\n"
                   "  [--min-range-m 0.2] [--keep-low-confidence] [--imu-only]\n"
                   "Window is [start, start+duration), relative to the first LiDAR packet.\n"
                   "Reads the complete capture and validates its completion marker.\n";
      std::exit(0);
    }
    if (arg == "--inspect") { opt.inspect = true; continue; }
    if (arg == "--imu-only") { opt.imu_only = true; continue; }
    if (arg == "--keep-low-confidence") { opt.keep_low_confidence = true; continue; }
    if (i + 1 == argc) throw std::runtime_error("Missing value for " + arg);
    const std::string value = argv[++i];
    if (arg == "--input") opt.input = value;
    else if (arg == "--output") opt.output = value;
    else if (arg == "--start-s") opt.start = seconds(value, true);
    else if (arg == "--duration-s") opt.duration = seconds(value, false);
    else if (arg == "--sensor-id") {
      size_t parsed = 0;
      const auto number = std::stoull(value, &parsed);
      if (value.empty() || value[0] == '-' || parsed != value.size() ||
          number > std::numeric_limits<uint32_t>::max()) throw std::runtime_error("Invalid sensor ID");
      opt.sensor = static_cast<uint32_t>(number);
    } else if (arg == "--min-range-m") {
      size_t parsed = 0;
      opt.min_range = std::stod(value, &parsed);
      if (parsed != value.size() || !std::isfinite(opt.min_range) || opt.min_range < 0 ||
          opt.min_range > 10000) throw std::runtime_error("Invalid minimum range");
    } else throw std::runtime_error("Unknown argument: " + arg);
  }
  if (opt.input.empty() || (!opt.inspect && opt.output.empty()))
    throw std::runtime_error("--input and either --output or --inspect are required; see --help");
  if (opt.inspect && !opt.output.empty()) throw std::runtime_error("--inspect does not write an output directory");
  if (opt.duration > std::numeric_limits<uint64_t>::max() - opt.start)
    throw std::runtime_error("Interval overflow");
  return opt;
}
struct Origin {
  uint64_t timestamp = 0;
  uint32_t sensor = 0;
  atlas::timesync::ClockSource clock = atlas::timesync::ClockSource::Unknown;
};
Origin origin(const Options& opt) {
  atlas::storage::CaptureReader reader(opt.input);
  while (const auto event = reader.next()) {
    const auto* frame = std::get_if<atlas::sensors::LidarFrame>(&event->data);
    if (frame && (!opt.sensor || frame->sensor_id == *opt.sensor))
      return {frame->timestamp.nanoseconds, frame->sensor_id, frame->timestamp.source};
  }
  throw std::runtime_error("No LiDAR packets found for the requested sensor");
}
struct Bin {
  uint64_t packets = 0, points = 0, imu_samples = 0;
  double gyro_norm_squared_sum = 0, gyro_norm_max = 0;
};
struct Stats {
  uint64_t events = 0, packets = 0, raw_points = 0, selected_points = 0, imu_samples = 0;
  uint64_t rejected_nonfinite = 0, rejected_range = 0, rejected_tag = 0;
  uint64_t mismatched_imu_clock = 0, invalid_imu = 0, last_timestamp = 0;
  uint64_t lidar_timestamp_regressions = 0, imu_timestamp_regressions = 0;
  std::optional<uint64_t> selected_first_timestamp, selected_last_timestamp;
  std::map<uint64_t, Bin> bins;
  std::string manifest;
};
using PointConsumer = std::function<void(const atlas::sensors::LidarPoint&, uint64_t)>;
using ImuConsumer = std::function<void(const atlas::imu::ImuSample&)>;
Stats scan(const Options& opt, const Origin& ref,
           const PointConsumer& point_consumer = {}, const ImuConsumer& imu_consumer = {}) {
  if (ref.timestamp > std::numeric_limits<uint64_t>::max() - opt.start - opt.duration)
    throw std::runtime_error("Absolute interval overflow");
  const auto begin = ref.timestamp + opt.start;
  const auto end = begin + opt.duration;
  Stats stats;
  std::optional<uint64_t> previous_lidar, previous_imu;
  stats.last_timestamp = ref.timestamp;
  atlas::storage::CaptureReader reader(opt.input);
  stats.manifest = reader.metadata();
  auto bin = [&](uint64_t timestamp) -> Bin& {
    const auto second = (timestamp - ref.timestamp) / kSecond;
    if (stats.bins.size() >= 1'000'000 && !stats.bins.contains(second))
      throw std::runtime_error("Too many time bins; inspect capture timestamps");
    return stats.bins[second];
  };
  while (const auto event = reader.next()) {
    ++stats.events;
    if (const auto* frame = std::get_if<atlas::sensors::LidarFrame>(&event->data)) {
      if (frame->sensor_id != ref.sensor) continue;
      if (frame->timestamp.source != ref.clock || frame->timestamp.nanoseconds < ref.timestamp)
        throw std::runtime_error("LiDAR clock changed or timestamp precedes the first packet; split the recording by clock epoch");
      if (previous_lidar && frame->timestamp.nanoseconds < *previous_lidar)
        ++stats.lidar_timestamp_regressions;
      previous_lidar = frame->timestamp.nanoseconds;
      ++stats.packets;
      ++bin(frame->timestamp.nanoseconds).packets;
      for (const auto& point : frame->points) {
        if (frame->timestamp.nanoseconds > std::numeric_limits<uint64_t>::max() - point.offset_ns)
          throw std::runtime_error("Point timestamp overflow");
        const auto stamp = frame->timestamp.nanoseconds + point.offset_ns;
        stats.last_timestamp = std::max(stats.last_timestamp, stamp);
        ++stats.raw_points;
        ++bin(stamp).points;
        if (stamp < begin || stamp >= end) continue;
        const auto range2 = double(point.x_m) * point.x_m + double(point.y_m) * point.y_m +
                            double(point.z_m) * point.z_m;
        if (!std::isfinite(range2)) { ++stats.rejected_nonfinite; continue; }
        if (range2 < opt.min_range * opt.min_range) { ++stats.rejected_range; continue; }
        // MID-360: adhesion, small particles, other; 2 means low confidence.
        const bool low = (point.tag & 3U) == 2U || ((point.tag >> 2U) & 3U) == 2U ||
                         ((point.tag >> 4U) & 3U) == 2U;
        if (!opt.keep_low_confidence && low) { ++stats.rejected_tag; continue; }
        ++stats.selected_points;
        if (!stats.selected_first_timestamp || stamp < *stats.selected_first_timestamp)
          stats.selected_first_timestamp = stamp;
        if (!stats.selected_last_timestamp || stamp > *stats.selected_last_timestamp)
          stats.selected_last_timestamp = stamp;
        if (point_consumer) point_consumer(point, stamp);
      }
    } else if (const auto* imu = std::get_if<atlas::imu::ImuSample>(&event->data)) {
      if (imu->sensor_id != ref.sensor) continue;
      if (imu->timestamp.source != ref.clock) { ++stats.mismatched_imu_clock; continue; }
      if (previous_imu && imu->timestamp.nanoseconds < *previous_imu)
        ++stats.imu_timestamp_regressions;
      previous_imu = imu->timestamp.nanoseconds;
      if (!std::isfinite(imu->gyro_x_rad_s) || !std::isfinite(imu->gyro_y_rad_s) ||
          !std::isfinite(imu->gyro_z_rad_s) || !std::isfinite(imu->accel_x_m_s2) ||
          !std::isfinite(imu->accel_y_m_s2) || !std::isfinite(imu->accel_z_m_s2)) {
        ++stats.invalid_imu; continue;
      }
      if (imu->timestamp.nanoseconds >= ref.timestamp) {
        auto& entry = bin(imu->timestamp.nanoseconds);
        ++entry.imu_samples;
        const auto norm2 = double(imu->gyro_x_rad_s) * imu->gyro_x_rad_s +
                           double(imu->gyro_y_rad_s) * imu->gyro_y_rad_s +
                           double(imu->gyro_z_rad_s) * imu->gyro_z_rad_s;
        entry.gyro_norm_squared_sum += norm2;
        entry.gyro_norm_max = std::max(entry.gyro_norm_max, std::sqrt(norm2));
      }
      if (imu->timestamp.nanoseconds >= begin && imu->timestamp.nanoseconds < end) {
        ++stats.imu_samples;
        if (imu_consumer) imu_consumer(*imu);
      }
    }
  }
  return stats;
}
std::string quoted(const std::string& text) {
  std::ostringstream result;
  result << '"';
  for (const unsigned char c : text) {
    if (c == '"' || c == '\\') result << '\\' << c;
    else if (c < 0x20) result << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c);
    else result << c;
  }
  result << '"';
  return result.str();
}
const char* clockName(atlas::timesync::ClockSource clock) {
  switch (clock) {
    case atlas::timesync::ClockSource::Unsynchronized: return "unsynchronized";
    case atlas::timesync::ClockSource::Ptp: return "ptp";
    case atlas::timesync::ClockSource::Gps: return "gps";
    default: return "unknown";
  }
}
void metadata(std::ostream& out, const Options& opt, const Origin& ref, const Stats& stats) {
  out << std::setprecision(17)
      << "{\n  \"schema\": \"atlas.calibration.raw_export.v1\",\n  \"capture_complete\": true,\n"
      << "  \"source\": " << quoted(fs::absolute(opt.input).string()) << ",\n"
      << "  \"source_capture\": " << quoted(fs::absolute(opt.input).string()) << ",\n"
      << "  \"coordinate_frame\": \"lidar_sensor\",\n  \"length_unit\": \"metre\",\n"
      << "  \"units\": \"m\",\n"
      << "  \"point_cloud_exported\": " << ((!opt.inspect && !opt.imu_only) ? "true" : "false") << ",\n"
      << "  \"deskewed\": false,\n  \"stationarity_verified\": false,\n"
      << "  \"extrinsic_calibration_verified\": false,\n  \"clock_alignment_verified\": false,\n"
      << "  \"sensor_id\": " << ref.sensor << ",\n  \"clock\": " << quoted(clockName(ref.clock)) << ",\n"
      << "  \"clock_source\": " << unsigned(ref.clock) << ",\n"
      << "  \"first_lidar_timestamp_ns\": " << ref.timestamp << ",\n"
      << "  \"last_lidar_timestamp_ns\": " << stats.last_timestamp << ",\n"
      << "  \"lidar_span_s\": " << double(stats.last_timestamp - ref.timestamp) / kSecond << ",\n"
      << "  \"requested_start_offset_ns\": " << opt.start << ",\n"
      << "  \"requested_duration_ns\": " << opt.duration << ",\n"
      << "  \"interval_begin_ns_inclusive\": " << ref.timestamp + opt.start << ",\n"
      << "  \"interval_end_ns_exclusive\": " << ref.timestamp + opt.start + opt.duration << ",\n"
      << "  \"minimum_range_m\": " << opt.min_range << ",\n"
      << "  \"filter_mid360_low_confidence\": " << (opt.keep_low_confidence ? "false" : "true") << ",\n"
      << "  \"capture_events\": " << stats.events << ",\n  \"lidar_packets\": " << stats.packets << ",\n"
      << "  \"raw_points\": " << stats.raw_points << ",\n  \"selected_points\": " << stats.selected_points << ",\n"
      << "  \"selected_imu_samples\": " << stats.imu_samples << ",\n"
      << "  \"rejected_nonfinite\": " << stats.rejected_nonfinite << ",\n"
      << "  \"rejected_range\": " << stats.rejected_range << ",\n"
      << "  \"rejected_tag\": " << stats.rejected_tag << ",\n"
      << "  \"mismatched_imu_clock\": " << stats.mismatched_imu_clock << ",\n"
      << "  \"invalid_imu_samples\": " << stats.invalid_imu << ",\n"
      << "  \"lidar_timestamp_regressions\": " << stats.lidar_timestamp_regressions << ",\n"
      << "  \"imu_timestamp_regressions\": " << stats.imu_timestamp_regressions << ",\n"
      << "  \"selected_first_timestamp_ns\": " << (stats.selected_first_timestamp ? std::to_string(*stats.selected_first_timestamp) : "null") << ",\n"
      << "  \"selected_last_timestamp_ns\": " << (stats.selected_last_timestamp ? std::to_string(*stats.selected_last_timestamp) : "null") << ",\n"
      << "  \"seconds\": [\n";
  bool first = true;
  for (const auto& [second, entry] : stats.bins) {
    if (!first) out << ",\n";
    first = false;
    out << "    {\"offset_s\":" << second << ",\"packets\":" << entry.packets
        << ",\"points\":" << entry.points << ",\"imu_samples\":" << entry.imu_samples
        << ",\"gyro_norm_rms_rad_s\":";
    if (entry.imu_samples) out << std::sqrt(entry.gyro_norm_squared_sum / entry.imu_samples);
    else out << "null";
    out << ",\"gyro_norm_max_rad_s\":";
    if (entry.imu_samples) out << entry.gyro_norm_max; else out << "null";
    out << '}';
  }
  out << "\n  ]\n}\n";
}
std::ofstream outputFile(const fs::path& path) {
  std::ofstream out(path, std::ios::binary);
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out << std::setprecision(std::numeric_limits<float>::max_digits10);
  return out;
}
void exportWindow(const Options& opt, const Origin& ref, const Stats& stats) {
  if (!opt.imu_only && !stats.selected_points) throw std::runtime_error("No usable points in requested interval; use --inspect");
  if (opt.imu_only && !stats.imu_samples) throw std::runtime_error("No usable IMU samples in requested interval; use --inspect");
  if (stats.mismatched_imu_clock) throw std::runtime_error("IMU and LiDAR use different clocks; cannot export a common calibration interval");
  if (stats.lidar_timestamp_regressions || stats.imu_timestamp_regressions)
    throw std::runtime_error("Sensor timestamps regress; inspect packet ordering or clock resets before calibration");
  if (!fs::create_directory(opt.output)) throw std::runtime_error("Output directory already exists");
  try {
    std::ofstream cloud, stamps;
    auto gyro = outputFile(opt.output / "gyro.csv");
    if (!opt.imu_only) {
      cloud = outputFile(opt.output / "cloud.ply");
      stamps = outputFile(opt.output / "points.csv");
      cloud << "ply\nformat ascii 1.0\ncomment ATLAS raw sensor frame, metres; no deskew\n"
            << "element vertex " << stats.selected_points
            << "\nproperty float x\nproperty float y\nproperty float z\n"
            << "property uchar reflectivity\nproperty uchar tag\nend_header\n";
      stamps << "vertex_index,timestamp_ns,clock_source,reflectivity,tag\n";
    }
    gyro << "timestamp_ns,clock_source,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,accel_x_m_s2,accel_y_m_s2,accel_z_m_s2\n";
    uint64_t index = 0;
    const auto second = scan(opt, ref, [&](const auto& point, uint64_t stamp) {
      if (opt.imu_only) return;
      cloud << point.x_m << ' ' << point.y_m << ' ' << point.z_m << ' '
            << unsigned(point.reflectivity) << ' ' << unsigned(point.tag) << '\n';
      stamps << index++ << ',' << stamp << ',' << unsigned(ref.clock) << ','
             << unsigned(point.reflectivity) << ',' << unsigned(point.tag) << '\n';
    }, [&](const auto& imu) {
      gyro << imu.timestamp.nanoseconds << ',' << unsigned(imu.timestamp.source) << ','
           << imu.gyro_x_rad_s << ',' << imu.gyro_y_rad_s << ',' << imu.gyro_z_rad_s << ','
           << imu.accel_x_m_s2 << ',' << imu.accel_y_m_s2 << ',' << imu.accel_z_m_s2 << '\n';
    });
    if (second.events != stats.events || second.selected_points != stats.selected_points ||
        second.imu_samples != stats.imu_samples || second.last_timestamp != stats.last_timestamp)
      throw std::runtime_error("Capture changed during export");
    if (!opt.imu_only) { cloud.close(); stamps.close(); }
    gyro.close();
    auto manifest = outputFile(opt.output / "source_manifest.txt");
    manifest << stats.manifest;
    manifest.close();
    auto meta = outputFile(opt.output / "metadata.json");
    metadata(meta, opt, ref, stats);
    meta.close();
  } catch (...) {
    std::error_code ignored;
    fs::remove_all(opt.output, ignored);  // Only the new directory created above.
    throw;
  }
}
}  // namespace

int main(int argc, char** argv) {
  try {
    const auto opt = arguments(argc, argv);
    if (!opt.inspect && fs::exists(opt.output)) throw std::runtime_error("Output directory already exists");
    const auto ref = origin(opt);
    const auto stats = scan(opt, ref);
    if (opt.inspect) metadata(std::cout, opt, ref, stats);
    else {
      exportWindow(opt, ref, stats);
      std::cout << "Exported " << (opt.imu_only ? 0 : stats.selected_points) << " sensor-frame points and " << stats.imu_samples
                << " IMU samples to " << fs::absolute(opt.output) << '\n';
      std::cout << "Stationarity, camera extrinsics and phone/LiDAR clock alignment remain unverified.\n";
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "atlas_calibration_export: " << error.what() << '\n';
    return 1;
  }
}
