#include "StorageTestSupport.hpp"
#include <atlas/timesync/SynchronizationOrchestrator.hpp>
#include <iostream>

int main() {
  try {
    using namespace atlas;
    using namespace atlas::timesync;
    constexpr uint64_t ms = 1000000;
    ImuSynchronizer stream(2);
    imu::ImuSample sample; sample.sensor_id = 5; sample.timestamp.source = ClockSource::Unsynchronized;
    REQUIRE(stream.push(sample) == SyncPushResult::Unaligned);
    ClockAlignment alignment{.sensor_id = 5, .source = ClockSource::Unsynchronized,
                              .source_anchor_ns = 1000, .reference_anchor_ns = 2000, .rate = 1.001};
    stream.set_alignment(alignment);
    sample.timestamp.nanoseconds = 1000; REQUIRE(stream.push(sample) == SyncPushResult::Accepted);
    REQUIRE(stream.samples().front().time_ns == 2000);
    REQUIRE(stream.push(sample) == SyncPushResult::OutOfOrder);
    sample.timestamp.nanoseconds = 3000; REQUIRE(stream.push(sample) == SyncPushResult::Accepted);
    sample.timestamp.nanoseconds = 4000; REQUIRE(stream.push(sample) == SyncPushResult::QueueFull);
    sample.sensor_id = 6; REQUIRE(stream.push(sample) == SyncPushResult::WrongClock);
    REQUIRE(!ClockAlignment{.source_anchor_ns = 100}.map(0));
    constexpr uint64_t epoch_ns = 1'800'000'000'000'000'000ULL;
    const ClockAlignment precise{.source_anchor_ns = epoch_ns, .reference_anchor_ns = epoch_ns};
    REQUIRE(precise.map(epoch_ns + 1) == epoch_ns + 1);
    REQUIRE(precise.map(epoch_ns - 1) == epoch_ns - 1);
    REQUIRE(ClockAlignment{}.map(epoch_ns + 1) == epoch_ns + 1);
    REQUIRE(!ClockAlignment{.reference_anchor_ns = UINT64_MAX}.map(1));

    SynchronizationOrchestrator sync({.max_imu_gap_ns = 10 * ms, .require_camera = true});
    ClockAlignment lidar_clock{.sensor_id = 1, .source = ClockSource::Unsynchronized};
    ClockAlignment imu_clock{.sensor_id = 1, .source = ClockSource::Unsynchronized};
    ClockAlignment camera_clock{.sensor_id = 2, .source = ClockSource::Unknown,
                                 .source_anchor_ns = 1000 * ms, .reference_anchor_ns = 0};
    sync.configure_clocks(lidar_clock, imu_clock, camera_clock);
    sensors::LidarFrame lidar; lidar.sensor_id = 1; lidar.timestamp = {20 * ms, ClockSource::Unsynchronized};
    lidar.duration_ns = 10 * ms; lidar.points = {{.offset_ns = 10 * ms}};
    REQUIRE(sync.push(lidar) == SyncPushResult::Accepted);
    REQUIRE(!sync.poll(30 * ms));
    sample.sensor_id = 1;
    for (uint64_t t : {15, 20, 25, 30, 35}) { sample.timestamp.nanoseconds = t * ms; REQUIRE(sync.push(sample) == SyncPushResult::Accepted); }
    REQUIRE(!sync.poll(30 * ms)); // required camera absent
    sensors::CameraFrame camera; camera.sensor_id = 2; camera.timestamp = {1026 * ms, ClockSource::Unknown};
    REQUIRE(sync.push(camera) == SyncPushResult::Accepted);
    auto packet = sync.poll(35 * ms);
    REQUIRE(packet && packet->ready_for_estimation && packet->imu_complete && packet->camera_matched);
    REQUIRE(packet->begin_ns == 20 * ms && packet->end_ns == 30 * ms);
    REQUIRE(packet->imu.front().time_ns == 20 * ms && packet->imu.back().time_ns == 30 * ms);
    REQUIRE(packet->camera->time_ns == 26 * ms);
    REQUIRE(packet->camera->sample.timestamp.nanoseconds == 1026 * ms); // original preserved
    REQUIRE(!sync.poll(35 * ms));
    lidar.timestamp.nanoseconds = 100 * ms; REQUIRE(sync.push(lidar) == SyncPushResult::Accepted);
    packet = sync.poll(400 * ms); REQUIRE(packet && !packet->ready_for_estimation && !packet->imu_complete);
    lidar.timestamp.nanoseconds = 500 * ms; REQUIRE(sync.push(lidar) == SyncPushResult::Accepted);
    const auto epoch = packet->epoch;
    sync.configure_clocks(lidar_clock, imu_clock); REQUIRE(sync.stats().reset_discarded > 0);
    REQUIRE(!sync.poll(600 * ms, true));
    REQUIRE(sync.push(lidar) == SyncPushResult::Accepted);
    packet = sync.poll(600 * ms, true); REQUIRE(packet && packet->epoch > epoch && !packet->ready_for_estimation);

    SynchronizationOrchestrator gaps({.max_imu_gap_ns = 10 * ms});
    imu_clock.uncertainty_ns = 6 * ms; gaps.configure_clocks(lidar_clock, imu_clock);
    lidar.timestamp.nanoseconds = 20 * ms; REQUIRE(gaps.push(lidar) == SyncPushResult::Accepted);
    for (uint64_t t : {0, 50}) { sample.timestamp.nanoseconds = t * ms; REQUIRE(gaps.push(sample) == SyncPushResult::Accepted); }
    packet = gaps.poll(60 * ms); REQUIRE(packet && !packet->imu_complete && !packet->clocks_precise);
    lidar.timestamp.nanoseconds = UINT64_MAX; REQUIRE(gaps.push(lidar) == SyncPushResult::InvalidTime);
    REQUIRE(gaps.stats().rejected == 1);
    // Invalid reconfiguration is transactional: previously configured models survive.
    rejects([&] { auto invalid = imu_clock; invalid.rate = 0; gaps.configure_clocks(lidar_clock, invalid); });
    lidar.timestamp.nanoseconds = 100 * ms; REQUIRE(gaps.push(lidar) == SyncPushResult::Accepted);
    std::cout << "Independent clock mapping, ordering, bounded queues, bracketing, timeout and reset passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
