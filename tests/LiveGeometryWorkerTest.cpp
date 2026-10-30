#include "LiveGeometryWorker.hpp"
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>
using namespace atlas;
using namespace std::chrono_literals;
namespace {
void check(bool ok) { if (!ok) throw std::runtime_error("Live worker regression"); }
sensors::LidarFrame frame(uint64_t stamp, bool empty=false) {
  sensors::LidarFrame f; f.timestamp.nanoseconds=stamp;
  if (!empty) f.points.push_back({.x_m=1,.y_m=2,.z_m=3});
  return f;
}
struct Results {
  std::mutex mutex; std::condition_variable cv;
  bool entered=false, release=false;
  std::vector<uint64_t> timestamps, revisions;
  geometry::GeometrySnapshot extract(std::span<const sensors::LidarPoint>, uint64_t timestamp) {
    if (timestamp==1) {
      std::unique_lock lock(mutex); entered=true; cv.notify_all();
      if (!cv.wait_for(lock,2s,[&]{return release;})) throw std::runtime_error("test gate timeout");
    }
    geometry::GeometrySnapshot s; s.source_timestamp_ns=timestamp; return s;
  }
  void arrived(geometry::GeometrySnapshot s, uint64_t revision, float) {
    std::lock_guard lock(mutex); timestamps.push_back(s.source_timestamp_ns); revisions.push_back(revision); cv.notify_all();
  }
  void wait_entered() { std::unique_lock lock(mutex); check(cv.wait_for(lock,2s,[&]{return entered;})); }
  void open() { std::lock_guard lock(mutex); release=true; cv.notify_all(); }
  void wait_count(size_t n) { std::unique_lock lock(mutex); check(cv.wait_for(lock,2s,[&]{return timestamps.size()>=n;})); }
};
void latest_only() {
  Results r;
  ui::LiveGeometryWorker worker(1ms,[&](auto s,auto rev,auto ms){r.arrived(std::move(s),rev,ms);},
      [&](auto p,auto t){return r.extract(p,t);});
  worker.start(); worker.set_tracking(true); worker.submit(frame(1)); r.wait_entered();
  worker.submit(frame(2)); worker.submit(frame(3)); r.open(); r.wait_count(2); worker.stop();
  check(r.timestamps==std::vector<uint64_t>({1,3})); check(r.revisions[1]>r.revisions[0]);
}
void invalidate(bool tracking_loss, bool reconnect) {
  Results r;
  ui::LiveGeometryWorker worker(1ms,[&](auto s,auto rev,auto ms){r.arrived(std::move(s),rev,ms);},
      [&](auto p,auto t){return r.extract(p,t);});
  worker.start(); worker.set_tracking(true); worker.submit(frame(1)); r.wait_entered();
  if (reconnect) worker.reset();
  else if (tracking_loss) worker.set_tracking(false);
  else worker.submit(frame(2,true));
  worker.set_tracking(true); worker.submit(frame(3)); r.open(); r.wait_count(2); worker.stop();
  check(r.timestamps==std::vector<uint64_t>({tracking_loss || reconnect ? 0ULL : 2ULL,3}));
  check(r.revisions[1]>r.revisions[0]);
}
}
int main() {
  latest_only(); invalidate(false,false); invalidate(true,false); invalidate(false,true);
  {
    Results freshness; freshness.release=true;
    ui::LiveGeometryWorker worker(1ms,[&](auto s,auto rev,auto ms){freshness.arrived(std::move(s),rev,ms);},
        [&](auto p,auto t){return freshness.extract(p,t);});
    worker.start(); worker.set_tracking(true); worker.submit(frame(1)); freshness.wait_count(1);
    worker.submit(frame(1)); worker.submit(frame(0));
    {
      std::unique_lock lock(freshness.mutex);
      check(!freshness.cv.wait_for(lock,50ms,[&]{return freshness.timestamps.size()>1;}));
    }
    worker.submit(frame(2)); freshness.wait_count(2); worker.stop();
    check(freshness.timestamps==std::vector<uint64_t>({1,2}));
  }
  Results r;
  ui::LiveGeometryWorker worker(1ms,[&](auto s,auto rev,auto ms){r.arrived(std::move(s),rev,ms);},
      [](auto,auto)->geometry::GeometrySnapshot{throw std::runtime_error("extraction failure");});
  worker.start(); worker.set_tracking(true); worker.submit(frame(2)); r.wait_count(1); worker.stop();
  check(worker.errors()==1 && r.timestamps.front()==2);
  std::cout << "Bounded live worker replacement and invalidation passed\n";
}
