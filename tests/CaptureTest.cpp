#include "StorageTestSupport.hpp"
#include <atlas/storage/Capture.hpp>
#include <atlas/sensors/SensorDataBus.hpp>
#include <atlas/preprocessing/SensorPreprocessor.hpp>
#include <bit>
#include <iostream>
#include <thread>

int main() {
  try {
    using namespace atlas;
    TempDirectory temp;
    const auto path = temp.path / "capture";
    auto recorder = std::make_shared<storage::CaptureRecorder>(path, "calibration=unverified\nclock=raw\n");
    sensors::SensorDataBus bus; const auto subscription = bus.subscribe(recorder);
    sensors::LidarFrame lidar;
    lidar.sensor_id = 42; lidar.packet_sequence = 65535; lidar.frame_index = 255;
    lidar.duration_ns = 500; lidar.timestamp = {123456789012345ULL, timesync::ClockSource::Ptp};
    lidar.points = {{.x_m = -1.2F, .y_m = 3.4F, .z_m = -0.0F, .offset_ns = 500, .reflectivity = 123, .tag = 255}};
    bus.publish(lidar);
    imu::ImuSample imu; imu.sensor_id = 42; imu.timestamp = {123, timesync::ClockSource::Unsynchronized};
    imu.accel_z_m_s2 = 9.81F; imu.gyro_x_rad_s = -0.2F; bus.publish(imu);
    sensors::SensorStatus status; status.sensor_id = 42; status.state = sensors::ConnectionState::Streaming;
    status.message = "captura ñ"; bus.publish(status);
    sensors::SensorWarning warning; warning.sensor_id = 42; warning.measured_value = 2.8F; bus.publish(warning);
    sensors::CameraFrame camera; camera.sensor_id = 7; camera.width = 1920; camera.height = 1080;
    camera.timestamp = {444, timesync::ClockSource::Unknown}; camera.intrinsics[0] = 1200.25;
    camera.world_from_camera[15] = 1; camera.image_path = "000001.jpg"; camera.calibration_json = "{\"verified\":false}";
    bus.publish(camera);
    // Two concurrent sources: all accepted events must survive in writer order.
    std::thread a([&] { for (int i = 0; i < 100; ++i) bus.publish(imu); });
    std::thread b([&] { for (int i = 0; i < 100; ++i) bus.publish(lidar); });
    a.join(); b.join(); bus.unsubscribe(subscription); recorder->close(); recorder->close();
    REQUIRE(recorder->stats().written_events == 205);
    REQUIRE(recorder->stats().pending_bytes == 0 && !recorder->stats().failed);
    storage::CaptureReader reader(path); REQUIRE(reader.metadata() == "calibration=unverified\nclock=raw\n");
    auto event = reader.next(); REQUIRE(event && event->sequence == 0);
    const auto recovered = std::get<sensors::LidarFrame>(event->data);
    REQUIRE(recovered.timestamp.nanoseconds == lidar.timestamp.nanoseconds);
    REQUIRE(recovered.timestamp.source == lidar.timestamp.source && recovered.packet_sequence == 65535);
    REQUIRE(recovered.duration_ns == 500 && recovered.frame_index == 255);
    REQUIRE(recovered.points[0].tag == 255 && recovered.points[0].reflectivity == 123);
    REQUIRE(std::bit_cast<uint32_t>(recovered.points[0].z_m) == std::bit_cast<uint32_t>(-0.0F));
    event = reader.next(); REQUIRE(std::get<imu::ImuSample>(event->data).accel_z_m_s2 == imu.accel_z_m_s2);
    event = reader.next(); REQUIRE(std::get<sensors::SensorStatus>(event->data).message == status.message);
    event = reader.next(); REQUIRE(std::get<sensors::SensorWarning>(event->data).measured_value == 2.8F);
    event = reader.next(); REQUIRE(std::get<sensors::CameraFrame>(event->data).calibration_json == camera.calibration_json);
    REQUIRE(std::get<sensors::CameraFrame>(event->data).intrinsics == camera.intrinsics);
    struct CameraSink final : sensors::ISensorDataSink {
      sensors::CameraFrame last; size_t count = 0;
      void on_camera_frame(const sensors::CameraFrame& value) override { last = value; ++count; }
    };
    sensors::SensorDataBus output;
    auto camera_sink = std::make_shared<CameraSink>(); const auto camera_id = output.subscribe(camera_sink);
    preprocessing::SensorPreprocessor preprocessor(output);
    event->dispatch(preprocessor);
    REQUIRE(camera_sink->count == 1 && camera_sink->last.intrinsics == camera.intrinsics);
    output.unsubscribe(camera_id);
    size_t count = 5; uint64_t last = event->arrival_offset_ns;
    while ((event = reader.next())) { REQUIRE(event->sequence == count++); REQUIRE(event->arrival_offset_ns >= last); last = event->arrival_offset_ns; }
    REQUIRE(count == 205 && !reader.next());
    rejects([&] { storage::CaptureRecorder duplicate(path, ""); });

    const auto overflow_path = temp.path / "overflow";
    storage::CaptureRecorder overflow(overflow_path, "", 1);
    overflow.on_lidar_frame(lidar); REQUIRE(overflow.stats().failed);
    rejects([&] { overflow.close(); });
    rejects([&] { storage::CaptureReader incomplete(overflow_path); while (incomplete.next()) {} });

    // A complete header but missing footer must never look like successful EOF.
    const auto file = path / "events.atlas";
    std::filesystem::resize_file(file, std::filesystem::file_size(file) - 4);
    rejects([&] { storage::CaptureReader truncated(path); while (truncated.next()) {} });
    std::fstream corrupt(file, std::ios::binary | std::ios::in | std::ios::out);
    corrupt.seekp(16); corrupt.put('X'); corrupt.close();
    rejects([&] { storage::CaptureReader invalid(path); });
    std::cout << "Capture roundtrip, concurrent producers, overflow, truncation and CRC passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
