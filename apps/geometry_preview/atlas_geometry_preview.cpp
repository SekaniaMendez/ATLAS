#include <atlas/ui/UiStreamServer.hpp>
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }

struct Options {
  std::filesystem::path input;
  uint16_t port = 47777;
  double seconds = 0;
  bool exercise_clears = false;
};

unsigned long long integer(const std::string& value) {
  unsigned long long result = 0;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size())
    throw std::runtime_error("Invalid integer: " + value);
  return result;
}

Options options(int argc, char** argv) {
  Options result;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--exercise-clears") result.exercise_clears = true;
    else if ((argument == "--input" || argument == "--port" || argument == "--seconds") && i + 1 < argc) {
      const std::string value = argv[++i];
      if (argument == "--input") result.input = value;
      else if (argument == "--port") {
        const auto number = integer(value);
        if (number < 1 || number > 65535) throw std::runtime_error("Port must be between 1 and 65535");
        result.port = static_cast<uint16_t>(number);
      } else {
        size_t consumed = 0;
        result.seconds = std::stod(value, &consumed);
        if (consumed != value.size() || !std::isfinite(result.seconds) || result.seconds < 0)
          throw std::runtime_error("Seconds must be a finite nonnegative number");
      }
    } else throw std::runtime_error("Unknown option or missing value: " + argument);
  }
  if (result.input.empty()) throw std::runtime_error("--input requires a saved ATLAS ASCII PLY");
  return result;
}

std::vector<atlas::sensors::LidarPoint> read_points(const std::filesystem::path& path) {
  std::ifstream input(path);
  std::string line;
  if (!input || !std::getline(input, line) || line != "ply")
    throw std::runtime_error("Cannot read an ASCII PLY: " + path.string());
  std::vector<std::string> properties;
  size_t count = 0;
  bool ascii = false, ended = false, vertex = false;
  for (size_t n = 0; n < 256 && std::getline(input, line); ++n) {
    if (line.size() > 4096) throw std::runtime_error("PLY header line is too long");
    std::istringstream fields(line);
    std::string word;
    fields >> word;
    if (word == "format") {
      std::string name, version;
      fields >> name >> version;
      ascii = name == "ascii" && version == "1.0";
      if (!ascii) throw std::runtime_error("Only ASCII PLY 1.0 is supported");
    } else if (word == "element") {
      std::string name, number;
      fields >> name >> number;
      const auto elements = integer(number);
      vertex = name == "vertex";
      if (vertex) {
        if (count || elements < 3 || elements > 2'000'000)
          throw std::runtime_error("PLY must contain between 3 and 2,000,000 vertices");
        count = static_cast<size_t>(elements);
      } else if (elements) throw std::runtime_error("Expected a point PLY, not mesh elements");
    } else if (word == "property" && vertex) {
      std::string type, name;
      fields >> type >> name;
      if (type == "list" || name.empty() || properties.size() >= 64 ||
          std::find(properties.begin(), properties.end(), name) != properties.end())
        throw std::runtime_error("Invalid or duplicate scalar PLY vertex property");
      properties.push_back(name);
    } else if (word == "end_header") { ended = true; break; }
  }
  if (!ascii || !ended || !count) throw std::runtime_error("Incomplete ASCII PLY header");
  const auto property = [&](const std::string& name) {
    const auto found = std::find(properties.begin(), properties.end(), name);
    return found == properties.end() ? properties.size() : static_cast<size_t>(found - properties.begin());
  };
  const size_t x = property("x"), y = property("y"), z = property("z"), reflectivity = property("reflectivity");
  if (x == properties.size() || y == properties.size() || z == properties.size())
    throw std::runtime_error("PLY needs x, y and z properties");
  std::vector<atlas::sensors::LidarPoint> points;
  points.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    if (!std::getline(input, line) || line.size() > 4096) throw std::runtime_error("Truncated or invalid PLY data");
    std::istringstream fields(line);
    std::vector<double> values(properties.size());
    for (double& value : values)
      if (!(fields >> value) || !std::isfinite(value)) throw std::runtime_error("Non-finite or malformed PLY value");
    std::string extra;
    if (fields >> extra) throw std::runtime_error("Extra columns in PLY vertex data");
    atlas::sensors::LidarPoint p;
    p.x_m = static_cast<float>(values[x]); p.y_m = static_cast<float>(values[y]); p.z_m = static_cast<float>(values[z]);
    if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m) || !std::isfinite(p.z_m))
      throw std::runtime_error("PLY coordinates exceed Float32 range");
    if (reflectivity < properties.size()) p.reflectivity = static_cast<uint8_t>(std::clamp(values[reflectivity], 0., 255.));
    points.push_back(p);
  }
  while (std::getline(input, line))
    if (line.find_first_not_of(" \t\r") != std::string::npos) throw std::runtime_error("PLY has extra vertex rows");
  return points;
}

uint64_t timestamp() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

int run(const Options& args) {
  const auto points = read_points(args.input);
  atlas::ui::UiStreamServerConfig config;
  config.port = args.port;
  config.service_name = "ATLAS geometry recording";
  config.source_kind = atlas::ui::StreamSourceKind::Recorded;
  config.world_frame = true;
  config.live_geometry = true;
  config.max_pending_points = config.max_points_per_frame = 50'000;
  atlas::ui::UiStreamServer server(config);
  if (!server.start()) throw std::runtime_error(server.last_error());
  std::cout << "[ATLAS GEOMETRY PREVIEW] saved_points=" << points.size()
            << " TCP=" << args.port << " source=" << args.input
            << " (simulation only; no Livox SDK or live acquisition)" << std::endl;
  const auto started = std::chrono::steady_clock::now();
  auto connected_at = started, next = started;
  bool was_connected = false;
  uint64_t sequence = 0;
  while (running) {
    const auto now = std::chrono::steady_clock::now();
    if (args.seconds > 0 && std::chrono::duration<double>(now - started).count() >= args.seconds) break;
    const bool connected = server.stats().client_connected;
    if (connected && !was_connected) connected_at = now;
    was_connected = connected;
    const double elapsed = std::chrono::duration<double>(now - connected_at).count();
    // Six 3-second phases repeat. Starting on connection avoids missing clear
    // phases while a human opens the iPhone app or a test connects its socket.
    const int phase = args.exercise_clears ? static_cast<int>(elapsed / 3) % 6 : 1;
    const char* names[] = {"progressive", "full", "empty", "reappearance", "tracking_lost", "recovered"};
    atlas::ui::wire::SlamStatus status;
    status.timestamp_ns = timestamp();
    status.state = phase == 4 ? atlas::ui::wire::SlamTrackingState::Lost : atlas::ui::wire::SlamTrackingState::Tracking;
    status.scans_accepted = ++sequence;
    status.message = std::string("OFFLINE GEOMETRY PREVIEW: ") + names[phase] + "; saved PLY, not live SLAM";
    server.publish_slam_status(status);
    atlas::sensors::LidarFrame frame;
    frame.timestamp.nanoseconds = status.timestamp_ns;
    if (phase != 2) {
      const size_t take = std::min<size_t>(50'000, points.size());
      frame.points.reserve(take);
      for (size_t i = 0; i < take; ++i) {
        // The progressive stage gradually exposes contiguous source regions.
        if (phase == 0 && static_cast<double>(i) / static_cast<double>(take) >
            std::min(1., .25 + std::fmod(elapsed, 3.) / 4)) continue;
        frame.points.push_back(points[i * points.size() / take]);
      }
    }
    server.on_lidar_frame(frame);
    next += std::chrono::milliseconds(250);
    std::this_thread::sleep_until(next);
  }
  server.stop();
  const auto stats = server.stats();
  std::cout << "[ATLAS GEOMETRY PREVIEW] frames=" << stats.frames_sent
            << " geometry=" << stats.geometry_snapshots_sent << " errors=" << stats.geometry_errors << '\n';
  return stats.geometry_errors == 0 ? 0 : 1;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    std::cout << "atlas_geometry_preview --input SAVED.ply [--port 47777] [--seconds 0] [--exercise-clears]\n"
                 "Streams saved measurements and live geometry to ATLASViewer without any LiDAR.\n"
                 "--exercise-clears cycles progressive/full/empty/reappearance/lost/recovered every 3 seconds.\n"
                 "--seconds 0 runs until Ctrl+C. Input files are never modified.\n";
    return 0;
  }
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  try { return run(options(argc, argv)); }
  catch (const std::exception& error) { std::cerr << "[ATLAS GEOMETRY PREVIEW] " << error.what() << '\n'; return 1; }
}
