#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#include <algorithm>
#include <atlas/imu/ImuSample.hpp>
#include <atlas/livox/LivoxSource.hpp>
#include <atlas/sensors/LidarFrame.hpp>
#include <atlas/sensors/SensorStatus.hpp>
#include <atlas/timesync/SensorTimestamp.hpp>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <numbers>
#include <string>
#include <utility>

namespace atlas::livox {
namespace {

constexpr float kMillimetersToMeters = 0.001F;
constexpr float kCentimetersToMeters = 0.01F;
constexpr float kStandardGravityMps2 = 9.80665F;
constexpr uint32_t kTimeIntervalUnitNs = 100;

[[nodiscard]] uint64_t decode_little_endian_u64(const uint8_t bytes[8]) noexcept {
  uint64_t value = 0;
  for (uint32_t index = 0; index < 8; ++index) {
    value |= static_cast<uint64_t>(bytes[index]) << (index * 8U);
  }
  return value;
}

[[nodiscard]] atlas::timesync::ClockSource clock_source(const uint8_t type) noexcept {
  switch (type) {
    case 0:
      return atlas::timesync::ClockSource::Unsynchronized;
    case 1:
      return atlas::timesync::ClockSource::Ptp;
    case 2:
      return atlas::timesync::ClockSource::Gps;
    default:
      return atlas::timesync::ClockSource::Unknown;
  }
}

[[nodiscard]] atlas::timesync::SensorTimestamp timestamp_of(
    const LivoxLidarEthernetPacket& packet) noexcept {
  return {
      .nanoseconds = decode_little_endian_u64(packet.timestamp),
      .source = clock_source(packet.time_type),
  };
}

[[nodiscard]] uint32_t point_offset_ns(const uint32_t index, const uint32_t count,
                                       const uint32_t duration_ns) noexcept {
  if (count < 2) {
    return 0;
  }
  return static_cast<uint32_t>((static_cast<uint64_t>(duration_ns) * index) / (count - 1U));
}

template <typename RawPoint>
[[nodiscard]] uint32_t safe_point_count(const LivoxLidarEthernetPacket& packet) noexcept {
  constexpr auto header_size = offsetof(LivoxLidarEthernetPacket, data);
  if (packet.length < header_size) {
    return 0;
  }
  const auto available = static_cast<size_t>(packet.length) - header_size;
  const auto capacity = available / sizeof(RawPoint);
  return static_cast<uint32_t>(std::min<size_t>(packet.dot_num, capacity));
}

[[nodiscard]] std::string fixed_string(const char* value, const size_t capacity) {
  size_t length = 0;
  while (length < capacity && value[length] != '\0') {
    ++length;
  }
  return std::string(value, length);
}

}  // namespace

class LivoxSource::Impl {
 public:
  Impl(atlas::sensors::SensorDataBus& output, LivoxSourceConfig config)
      : output_(output), config_(std::move(config)) {}

  ~Impl() { stop(); }

  [[nodiscard]] bool start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
      return true;
    }

    publish_status(0, atlas::sensors::ConnectionState::Connecting, "initializing Livox SDK");
    const char* config_path = config_.config_path.empty() ? nullptr : config_.config_path.c_str();
    if (!LivoxLidarSdkInit(config_path)) {
      set_error("LivoxLidarSdkInit failed");
      running_.store(false);
      publish_status(0, atlas::sensors::ConnectionState::Error, last_error());
      return false;
    }

    SetLivoxLidarPointCloudCallBack(&Impl::point_cloud_callback, this);
    SetLivoxLidarImuDataCallback(&Impl::imu_callback, this);
    SetLivoxLidarInfoCallback(&Impl::info_callback, this);
    SetLivoxLidarInfoChangeCallback(&Impl::info_change_callback, this);
    sdk_initialized_.store(true);
    return true;
  }

  void stop() noexcept {
    if (!running_.exchange(false)) {
      return;
    }
    if (sdk_initialized_.exchange(false)) {
      LivoxLidarSdkUninit();
    }
    publish_status(active_handle_.load(), atlas::sensors::ConnectionState::Disconnected,
                   "Livox source stopped");
    active_handle_.store(0);
  }

  [[nodiscard]] bool running() const noexcept { return running_.load(); }

  [[nodiscard]] LivoxSourceStats stats() const noexcept {
    return {
        .point_packets = point_packets_.load(),
        .points = points_.load(),
        .imu_packets = imu_packets_.load(),
        .imu_samples = imu_samples_.load(),
        .rejected_packets = rejected_packets_.load(),
    };
  }

  [[nodiscard]] std::string last_error() const {
    std::lock_guard lock(error_mutex_);
    return last_error_;
  }

 private:
  static void point_cloud_callback(const uint32_t handle, const uint8_t /*device_type*/,
                                   LivoxLidarEthernetPacket* packet, void* client_data) noexcept {
    if (client_data != nullptr) {
      static_cast<Impl*>(client_data)->on_point_cloud(handle, packet);
    }
  }

  static void imu_callback(const uint32_t handle, const uint8_t /*device_type*/,
                           LivoxLidarEthernetPacket* packet, void* client_data) noexcept {
    if (client_data != nullptr) {
      static_cast<Impl*>(client_data)->on_imu(handle, packet);
    }
  }

  static void info_callback(const uint32_t handle, const uint8_t /*device_type*/, const char* info,
                            void* client_data) noexcept {
    if (client_data != nullptr && info != nullptr) {
      static_cast<Impl*>(client_data)
          ->publish_status(handle, atlas::sensors::ConnectionState::Streaming, info);
    }
  }

  static void info_change_callback(const uint32_t handle, const LivoxLidarInfo* info,
                                   void* client_data) noexcept {
    if (client_data != nullptr) {
      static_cast<Impl*>(client_data)->on_info_change(handle, info);
    }
  }

  static void control_callback(const livox_status status, const uint32_t handle,
                               LivoxLidarAsyncControlResponse* response,
                               void* client_data) noexcept {
    if (client_data == nullptr) {
      return;
    }
    auto& self = *static_cast<Impl*>(client_data);
    if (status != kLivoxLidarStatusSuccess || (response != nullptr && response->ret_code != 0)) {
      self.rejected_packets_.fetch_add(1, std::memory_order_relaxed);
      self.publish_status(handle, atlas::sensors::ConnectionState::Error,
                          "MID-360 rejected a stream control request");
    }
  }

  void on_info_change(const uint32_t handle, const LivoxLidarInfo* info) noexcept {
    if (!running_.load() || info == nullptr || info->dev_type != kLivoxLidarTypeMid360) {
      return;
    }

    uint32_t expected = 0;
    if (!active_handle_.compare_exchange_strong(expected, handle) && expected != handle) {
      return;
    }

    {
      std::lock_guard lock(device_mutex_);
      serial_number_ = fixed_string(info->sn, sizeof(info->sn));
      ip_address_ = fixed_string(info->lidar_ip, sizeof(info->lidar_ip));
    }

    publish_status(handle, atlas::sensors::ConnectionState::Streaming, "MID-360 connected");

    if (config_.enable_point_cloud) {
      const auto status = EnableLivoxLidarPointSend(handle, &Impl::control_callback, this);
      if (status != kLivoxLidarStatusSuccess) {
        publish_status(handle, atlas::sensors::ConnectionState::Error,
                       "failed to request point cloud streaming");
      }
    }
    if (config_.enable_imu) {
      const auto status = EnableLivoxLidarImuData(handle, &Impl::control_callback, this);
      if (status != kLivoxLidarStatusSuccess) {
        publish_status(handle, atlas::sensors::ConnectionState::Error,
                       "failed to request IMU streaming");
      }
    }
  }

  void on_point_cloud(const uint32_t handle, const LivoxLidarEthernetPacket* packet) noexcept {
    if (!running_.load() || !config_.enable_point_cloud || packet == nullptr) {
      return;
    }

    atlas::sensors::LidarFrame frame;
    frame.sensor_id = handle;
    frame.packet_sequence = packet->udp_cnt;
    frame.frame_index = packet->frame_cnt;
    frame.timestamp = timestamp_of(*packet);
    frame.duration_ns = static_cast<uint32_t>(packet->time_interval) * kTimeIntervalUnitNs;

    switch (packet->data_type) {
      case kLivoxLidarCartesianCoordinateHighData:
        decode_cartesian_high(*packet, frame);
        break;
      case kLivoxLidarCartesianCoordinateLowData:
        decode_cartesian_low(*packet, frame);
        break;
      case kLivoxLidarSphericalCoordinateData:
        decode_spherical(*packet, frame);
        break;
      default:
        rejected_packets_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    if (frame.points.empty() && packet->dot_num != 0) {
      rejected_packets_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    point_packets_.fetch_add(1, std::memory_order_relaxed);
    points_.fetch_add(frame.points.size(), std::memory_order_relaxed);
    output_.publish(frame);
  }

  void on_imu(const uint32_t handle, const LivoxLidarEthernetPacket* packet) noexcept {
    if (!running_.load() || !config_.enable_imu || packet == nullptr ||
        packet->data_type != kLivoxLidarImuData) {
      return;
    }

    const auto count = safe_point_count<LivoxLidarImuRawPoint>(*packet);
    if (count == 0) {
      rejected_packets_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    const auto first_timestamp = timestamp_of(*packet);
    const auto duration_ns = static_cast<uint32_t>(packet->time_interval) * kTimeIntervalUnitNs;
    for (uint32_t index = 0; index < count; ++index) {
      LivoxLidarImuRawPoint raw{};
      std::memcpy(&raw, packet->data + index * sizeof(raw), sizeof(raw));
      atlas::imu::ImuSample sample{
          .sensor_id = handle,
          .timestamp = first_timestamp,
          .gyro_x_rad_s = raw.gyro_x,
          .gyro_y_rad_s = raw.gyro_y,
          .gyro_z_rad_s = raw.gyro_z,
          .accel_x_m_s2 = raw.acc_x * kStandardGravityMps2,
          .accel_y_m_s2 = raw.acc_y * kStandardGravityMps2,
          .accel_z_m_s2 = raw.acc_z * kStandardGravityMps2,
      };
      sample.timestamp.nanoseconds += point_offset_ns(index, count, duration_ns);
      output_.publish(sample);
    }

    imu_packets_.fetch_add(1, std::memory_order_relaxed);
    imu_samples_.fetch_add(count, std::memory_order_relaxed);
  }

  static void decode_cartesian_high(const LivoxLidarEthernetPacket& packet,
                                    atlas::sensors::LidarFrame& frame) {
    const auto count = safe_point_count<LivoxLidarCartesianHighRawPoint>(packet);
    frame.points.reserve(count);
    for (uint32_t index = 0; index < count; ++index) {
      LivoxLidarCartesianHighRawPoint raw{};
      std::memcpy(&raw, packet.data + index * sizeof(raw), sizeof(raw));
      frame.points.push_back({
          .x_m = static_cast<float>(raw.x) * kMillimetersToMeters,
          .y_m = static_cast<float>(raw.y) * kMillimetersToMeters,
          .z_m = static_cast<float>(raw.z) * kMillimetersToMeters,
          .offset_ns = point_offset_ns(index, count, frame.duration_ns),
          .reflectivity = raw.reflectivity,
          .tag = raw.tag,
      });
    }
  }

  static void decode_cartesian_low(const LivoxLidarEthernetPacket& packet,
                                   atlas::sensors::LidarFrame& frame) {
    const auto count = safe_point_count<LivoxLidarCartesianLowRawPoint>(packet);
    frame.points.reserve(count);
    for (uint32_t index = 0; index < count; ++index) {
      LivoxLidarCartesianLowRawPoint raw{};
      std::memcpy(&raw, packet.data + index * sizeof(raw), sizeof(raw));
      frame.points.push_back({
          .x_m = static_cast<float>(raw.x) * kCentimetersToMeters,
          .y_m = static_cast<float>(raw.y) * kCentimetersToMeters,
          .z_m = static_cast<float>(raw.z) * kCentimetersToMeters,
          .offset_ns = point_offset_ns(index, count, frame.duration_ns),
          .reflectivity = raw.reflectivity,
          .tag = raw.tag,
      });
    }
  }

  static void decode_spherical(const LivoxLidarEthernetPacket& packet,
                               atlas::sensors::LidarFrame& frame) {
    const auto count = safe_point_count<LivoxLidarSpherPoint>(packet);
    frame.points.reserve(count);
    constexpr float angle_scale = std::numbers::pi_v<float> / 18000.0F;
    for (uint32_t index = 0; index < count; ++index) {
      LivoxLidarSpherPoint raw{};
      std::memcpy(&raw, packet.data + index * sizeof(raw), sizeof(raw));
      const float radius = static_cast<float>(raw.depth) * kMillimetersToMeters;
      const float theta = static_cast<float>(raw.theta) * angle_scale;
      const float phi = static_cast<float>(raw.phi) * angle_scale;
      const float horizontal_radius = radius * std::sin(theta);
      frame.points.push_back({
          .x_m = horizontal_radius * std::cos(phi),
          .y_m = horizontal_radius * std::sin(phi),
          .z_m = radius * std::cos(theta),
          .offset_ns = point_offset_ns(index, count, frame.duration_ns),
          .reflectivity = raw.reflectivity,
          .tag = raw.tag,
      });
    }
  }

  void publish_status(const uint32_t handle, const atlas::sensors::ConnectionState state,
                      std::string message) const noexcept {
    atlas::sensors::SensorStatus status;
    status.sensor_id = handle;
    status.state = state;
    status.driver = "livox-mid360";
    {
      std::lock_guard lock(device_mutex_);
      status.serial_number = serial_number_;
      status.ip_address = ip_address_;
    }
    status.message = std::move(message);
    output_.publish(status);
  }

  void set_error(std::string message) {
    std::lock_guard lock(error_mutex_);
    last_error_ = std::move(message);
  }

  atlas::sensors::SensorDataBus& output_;
  LivoxSourceConfig config_;
  std::atomic_bool running_{false};
  std::atomic_bool sdk_initialized_{false};
  std::atomic_uint32_t active_handle_{0};
  std::atomic_uint64_t point_packets_{0};
  std::atomic_uint64_t points_{0};
  std::atomic_uint64_t imu_packets_{0};
  std::atomic_uint64_t imu_samples_{0};
  std::atomic_uint64_t rejected_packets_{0};
  mutable std::mutex error_mutex_;
  std::string last_error_;
  mutable std::mutex device_mutex_;
  std::string serial_number_;
  std::string ip_address_;
};

LivoxSource::LivoxSource(atlas::sensors::SensorDataBus& output, LivoxSourceConfig config)
    : impl_(std::make_unique<Impl>(output, std::move(config))) {}

LivoxSource::~LivoxSource() = default;

bool LivoxSource::start() { return impl_->start(); }
void LivoxSource::stop() noexcept { impl_->stop(); }
bool LivoxSource::running() const noexcept { return impl_->running(); }
LivoxSourceStats LivoxSource::stats() const noexcept { return impl_->stats(); }
std::string LivoxSource::last_error() const { return impl_->last_error(); }

}  // namespace atlas::livox
