#pragma once

#include <chrono>
#include <cstddef>

namespace atlas::apps {

struct SlamAppConfig {
  std::chrono::milliseconds map_snapshot_interval{250};
  std::chrono::milliseconds status_interval{1000};
  std::chrono::milliseconds live_poll_interval{50};
  std::chrono::milliseconds final_snapshot_grace{150};
  size_t maximum_snapshot_points = 50'000;
  size_t replay_batch_events = 100;
};

}  // namespace atlas::apps
