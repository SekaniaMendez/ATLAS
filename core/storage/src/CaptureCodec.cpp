#include "CaptureCodec.hpp"
#include <type_traits>

namespace atlas::storage {
namespace {
using binary::Encoder;
using binary::Decoder;
void timestamp(Encoder& e, const timesync::SensorTimestamp& t) { e.u64(t.nanoseconds); e.u8(static_cast<uint8_t>(t.source)); }
timesync::SensorTimestamp timestamp(Decoder& d) {
  const auto ns = d.u64();
  const auto source = d.u8();
  if (source > 2 && source != 255) throw std::runtime_error("Invalid clock source");
  return {ns, static_cast<timesync::ClockSource>(source)};
}
}

binary::Encoder encode_event(const CaptureEvent& event) {
  Encoder e;
  e.u8(static_cast<uint8_t>(event.data.index() + 1));
  e.u64(event.sequence); e.u64(event.arrival_offset_ns);
  std::visit([&](const auto& value) {
    using T = std::decay_t<decltype(value)>;
    e.u32(value.sensor_id);
    if constexpr (std::is_same_v<T, sensors::LidarFrame>) {
      timestamp(e, value.timestamp); e.u32(value.packet_sequence); e.u8(value.frame_index);
      e.u32(value.duration_ns);
      if (value.points.size() > (binary::kMaxRecordBytes - 64) / 18) throw std::runtime_error("LiDAR frame too large");
      e.u32(static_cast<uint32_t>(value.points.size()));
      for (const auto& p : value.points) {
        e.f32(p.x_m); e.f32(p.y_m); e.f32(p.z_m); e.u32(p.offset_ns); e.u8(p.reflectivity); e.u8(p.tag);
      }
    } else if constexpr (std::is_same_v<T, imu::ImuSample>) {
      timestamp(e, value.timestamp);
      e.f32(value.gyro_x_rad_s); e.f32(value.gyro_y_rad_s); e.f32(value.gyro_z_rad_s);
      e.f32(value.accel_x_m_s2); e.f32(value.accel_y_m_s2); e.f32(value.accel_z_m_s2);
    } else if constexpr (std::is_same_v<T, sensors::SensorStatus>) {
      e.u8(static_cast<uint8_t>(value.state)); e.string(value.driver); e.string(value.serial_number);
      e.string(value.ip_address); e.string(value.message);
    } else if constexpr (std::is_same_v<T, sensors::CameraFrame>) {
      timestamp(e, value.timestamp); e.u32(value.width); e.u32(value.height);
      for (auto v : value.intrinsics) e.f64(v);
      for (auto v : value.world_from_camera) e.f64(v);
      e.string(value.image_path); e.string(value.calibration_json);
    } else {
      timestamp(e, value.timestamp); e.u32(static_cast<uint32_t>(value.code));
      e.u8(static_cast<uint8_t>(value.severity)); e.f32(value.measured_value); e.f32(value.threshold);
    }
  }, event.data);
  return e;
}

CaptureEvent decode_event(std::span<const uint8_t> bytes) {
  Decoder d{bytes};
  const auto type = d.u8();
  CaptureEvent event;
  event.sequence = d.u64(); event.arrival_offset_ns = d.u64();
  const auto sensor = d.u32();
  switch (type) {
    case 1: {
      sensors::LidarFrame v; v.sensor_id = sensor; v.timestamp = timestamp(d);
      const auto sequence = d.u32();
      if (sequence > UINT16_MAX) throw std::runtime_error("Invalid packet sequence");
      v.packet_sequence = static_cast<uint16_t>(sequence); v.frame_index = d.u8(); v.duration_ns = d.u32();
      const auto count = d.u32();
      if (count > d.remaining() / 18) throw std::runtime_error("Invalid point count");
      v.points.resize(count);
      for (auto& p : v.points) { p.x_m = d.f32(); p.y_m = d.f32(); p.z_m = d.f32(); p.offset_ns = d.u32(); p.reflectivity = d.u8(); p.tag = d.u8(); }
      event.data = std::move(v); break;
    }
    case 2: {
      imu::ImuSample v; v.sensor_id = sensor; v.timestamp = timestamp(d);
      v.gyro_x_rad_s = d.f32(); v.gyro_y_rad_s = d.f32(); v.gyro_z_rad_s = d.f32();
      v.accel_x_m_s2 = d.f32(); v.accel_y_m_s2 = d.f32(); v.accel_z_m_s2 = d.f32();
      event.data = v; break;
    }
    case 3: {
      sensors::SensorStatus v; v.sensor_id = sensor;
      const auto state = d.u8(); if (state > 3) throw std::runtime_error("Invalid sensor state");
      v.state = static_cast<sensors::ConnectionState>(state); v.driver = d.string();
      v.serial_number = d.string(); v.ip_address = d.string(); v.message = d.string(); event.data = std::move(v); break;
    }
    case 4: {
      sensors::SensorWarning v; v.sensor_id = sensor; v.timestamp = timestamp(d);
      const auto code = d.u32(); const auto severity = d.u8();
      if (code < 1 || code > 3 || severity > 2) throw std::runtime_error("Invalid sensor warning");
      v.code = static_cast<sensors::SensorWarningCode>(code); v.severity = static_cast<sensors::SensorWarningSeverity>(severity);
      v.measured_value = d.f32(); v.threshold = d.f32(); event.data = v; break;
    }
    case 5: {
      sensors::CameraFrame v; v.sensor_id = sensor; v.timestamp = timestamp(d);
      v.width = d.u32(); v.height = d.u32();
      for (auto& x : v.intrinsics) x = d.f64();
      for (auto& x : v.world_from_camera) x = d.f64();
      v.image_path = d.string(); v.calibration_json = d.string(); event.data = std::move(v); break;
    }
    default: throw std::runtime_error("Unknown capture event type");
  }
  d.finish(); return event;
}

void CaptureEvent::dispatch(sensors::ISensorDataSink& sink) const {
  std::visit([&](const auto& value) {
    using T = std::decay_t<decltype(value)>;
    if constexpr (std::is_same_v<T, sensors::LidarFrame>) sink.on_lidar_frame(value);
    else if constexpr (std::is_same_v<T, imu::ImuSample>) sink.on_imu_sample(value);
    else if constexpr (std::is_same_v<T, sensors::SensorStatus>) sink.on_sensor_status(value);
    else if constexpr (std::is_same_v<T, sensors::CameraFrame>) sink.on_camera_frame(value);
    else sink.on_sensor_warning(value);
  }, data);
}
}  // namespace atlas::storage
