#include <atlas/sensors/SensorDataBus.hpp>
#include <atlas/ui/UiStreamServer.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <memory>
#include <numbers>
#include <thread>

namespace {

std::atomic_bool g_running{true};

void on_signal(int) { g_running.store(false); }

atlas::sensors::LidarFrame make_demo_frame(const uint32_t sequence, const float phase) {
  atlas::sensors::LidarFrame frame;
  frame.frame_index = static_cast<uint8_t>(sequence);
  frame.timestamp.nanoseconds =
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count());

  constexpr uint32_t rings = 64;
  constexpr uint32_t points_per_ring = 128;
  frame.points.reserve(rings * points_per_ring);
  for (uint32_t ring = 0; ring < rings; ++ring) {
    const float theta =
        static_cast<float>(ring) / static_cast<float>(rings - 1) * std::numbers::pi_v<float>;
    for (uint32_t sample = 0; sample < points_per_ring; ++sample) {
      const float phi = static_cast<float>(sample) / static_cast<float>(points_per_ring) * 2.0F *
                            std::numbers::pi_v<float> +
                        phase;
      const float radius = 8.0F + 0.6F * std::sin(phi * 5.0F + phase) * std::sin(theta * 3.0F);
      frame.points.push_back({
          .x_m = radius * std::sin(theta) * std::cos(phi),
          .y_m = radius * std::sin(theta) * std::sin(phi),
          .z_m = radius * std::cos(theta),
          .reflectivity = static_cast<uint8_t>((sample * 255U) / points_per_ring),
          .tag = 0,
      });
    }
  }
  return frame;
}

}  // namespace

int main() {
  std::signal(SIGINT, on_signal);

  atlas::sensors::SensorDataBus bus;
  auto stream = std::make_shared<atlas::ui::UiStreamServer>();
  if (!stream->start()) {
    std::cerr << "[ATLAS UI] failed: " << stream->last_error() << '\n';
    return 1;
  }
  const auto subscription = bus.subscribe(stream);
  std::cout << "[ATLAS UI] demo available on TCP 47777 and Bonjour _atlas._tcp\n";
  std::cout << "[ATLAS UI] open ATLAS Viewer on the iPhone (Ctrl+C to stop)\n";

  uint32_t sequence = 0;
  while (g_running.load()) {
    bus.publish(make_demo_frame(sequence, static_cast<float>(sequence) * 0.025F));
    ++sequence;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  bus.unsubscribe(subscription);
  stream->stop();
  const auto stats = stream->stats();
  std::cout << "[ATLAS UI] frames_sent=" << stats.frames_sent
            << " points_sent=" << stats.points_sent << '\n';
  return 0;
}
