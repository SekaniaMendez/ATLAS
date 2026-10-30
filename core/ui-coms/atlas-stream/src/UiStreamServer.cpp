#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atlas/ui/UiStreamServer.hpp>
#include <atlas/ui/WireProtocol.hpp>
#include <atlas/ui/GeometryWireProtocol.hpp>
#include "CameraReceiver.hpp"
#include "LiveGeometryWorker.hpp"

#ifdef __APPLE__
#include <dns_sd.h>
#endif

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace atlas::ui {
namespace {

constexpr int kNoSocket = -1;

void close_socket(const int socket_fd) noexcept {
  if (socket_fd != kNoSocket) {
    ::shutdown(socket_fd, SHUT_RDWR);
    ::close(socket_fd);
  }
}

[[nodiscard]] bool send_all(const int socket_fd, const std::span<const std::byte> bytes) noexcept {
  size_t sent = 0;
  while (sent < bytes.size()) {
#ifdef MSG_NOSIGNAL
    const auto result = ::send(socket_fd, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
#else
    const auto result = ::send(socket_fd, bytes.data() + sent, bytes.size() - sent, 0);
#endif
    if (result > 0) {
      sent += static_cast<size_t>(result);
      continue;
    }
    if (result < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

[[nodiscard]] bool is_visual_point(const atlas::sensors::LidarPoint& point,
                                   const float maximum_distance_squared) noexcept {
  const float distance_squared =
      point.x_m * point.x_m + point.y_m * point.y_m + point.z_m * point.z_m;
  return distance_squared > 0.0001F && distance_squared <= maximum_distance_squared &&
         std::isfinite(distance_squared);
}

}  // namespace

class UiStreamServer::Impl {
 public:
  explicit Impl(UiStreamServerConfig config, CameraHandler handler)
      : config_(std::move(config)), camera_receiver_(std::move(handler)),
        geometry_worker_(config_.geometry_interval, [this](auto snapshot, uint64_t revision, float ms) {
          if (!running_.load() || !client_connected_.load()) return;
          const auto generation = client_generation_.load();
          auto message = wire::encode_geometry_snapshot(snapshot,
              next_sequence_.fetch_add(1, std::memory_order_relaxed), revision, ms);
          {
            std::lock_guard lock(pending_mutex_);
            if (!client_connected_.load() || generation != client_generation_.load()) return;
            pending_geometry_.swap(message);
          }
          pending_cv_.notify_one();
        }) {
    if (config_.max_points_per_frame == 0 || config_.max_pending_points == 0 ||
        config_.frame_interval.count() <= 0 || config_.send_timeout.count() <= 0 ||
        !std::isfinite(config_.maximum_distance_m) ||
        config_.maximum_distance_m <= 0) {
      throw std::invalid_argument("Invalid UI stream limits");
    }
  }
  ~Impl() { stop(); }

  [[nodiscard]] bool start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
      return true;
    }

    const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listener == kNoSocket) {
      return fail_start("could not create TCP socket");
    }

    int reuse_address = 1;
    ::setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(config_.port);
    if (::bind(listener, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
      close_socket(listener);
      return fail_start("could not bind TCP port " + std::to_string(config_.port));
    }
    if (::listen(listener, 1) != 0) {
      close_socket(listener);
      return fail_start("could not listen on TCP port " + std::to_string(config_.port));
    }

    listener_fd_.store(listener);
    if (config_.world_frame && config_.live_geometry) geometry_worker_.start();
    register_bonjour();
    accept_thread_ = std::thread([this] { accept_loop(); });
    sender_thread_ = std::thread([this] { sender_loop(); });
    return true;
  }

  void stop() noexcept {
    if (!running_.exchange(false)) {
      return;
    }

    pending_cv_.notify_all();
    geometry_worker_.stop();
    close_socket(listener_fd_.exchange(kNoSocket));
    {
      std::lock_guard lock(client_mutex_);
      close_socket(std::exchange(client_fd_, kNoSocket));
    }
    if (accept_thread_.joinable()) {
      accept_thread_.join();
    }
    if (sender_thread_.joinable()) {
      sender_thread_.join();
    }
    camera_receiver_.stop();
    unregister_bonjour();
  }

  [[nodiscard]] bool running() const noexcept { return running_.load(); }

  [[nodiscard]] std::string last_error() const {
    std::lock_guard lock(error_mutex_);
    return last_error_;
  }

  [[nodiscard]] UiStreamServerStats stats() const noexcept {
    return {
        .frames_sent = frames_sent_.load(),
        .points_sent = points_sent_.load(),
        .dropped_points = dropped_points_.load(),
        .warnings_sent = warnings_sent_.load(),
        .slam_statuses_sent = slam_statuses_sent_.load(),
        .geometry_snapshots_sent = geometry_snapshots_sent_.load(),
        .geometry_errors = geometry_worker_.errors(),
        .camera_frames_received = camera_receiver_.received(),
        .camera_frames_rejected = camera_receiver_.rejected(),
        .client_connected = client_connected_.load(),
    };
  }

  void on_lidar_frame(const atlas::sensors::LidarFrame& frame) {
    if (!running_.load() || !client_connected_.load()) {
      return;
    }

    if (config_.world_frame && config_.live_geometry) geometry_worker_.submit(frame);

    std::lock_guard lock(pending_mutex_);
    if (config_.world_frame) {
      // A map snapshot is complete in itself: concatenating snapshots would
      // duplicate geometry and display stale maps under network backpressure.
      pending_points_.clear();
      pending_timestamp_ns_ = frame.timestamp.nanoseconds;
    }
    if (pending_points_.empty()) {
      pending_timestamp_ns_ = frame.timestamp.nanoseconds;
    }
    const auto available =
        config_.max_pending_points - std::min(config_.max_pending_points, pending_points_.size());
    const auto accepted = std::min(available, frame.points.size());
    pending_points_.insert(pending_points_.end(), frame.points.begin(),
                           frame.points.begin() + static_cast<std::ptrdiff_t>(accepted));
    pending_frame_ = true;
    dropped_points_.fetch_add(frame.points.size() - accepted, std::memory_order_relaxed);
    pending_cv_.notify_one();
  }

  void on_sensor_warning(const atlas::sensors::SensorWarning& warning) {
    if (!running_.load() || !client_connected_.load()) {
      return;
    }

    const auto sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    auto message = wire::encode_sensor_warning(warning, sequence);
    {
      std::lock_guard lock(pending_mutex_);
      if (pending_warning_messages_.size() == kMaximumPendingWarnings) {
        pending_warning_messages_.erase(pending_warning_messages_.begin());
      }
      pending_warning_messages_.push_back(std::move(message));
    }
    pending_cv_.notify_one();
  }

  void publish_slam_status(const wire::SlamStatus& status) {
    if (!config_.world_frame || !running_.load() || !client_connected_.load()) return;
    if (config_.live_geometry)
      geometry_worker_.set_tracking(status.state == wire::SlamTrackingState::Tracking && !status.fatal_error);
    auto message = wire::encode_slam_status(status, next_sequence_.fetch_add(1, std::memory_order_relaxed));
    {
      std::lock_guard lock(pending_mutex_);
      pending_slam_status_.swap(message);
    }
    pending_cv_.notify_one();
  }

 private:
  [[nodiscard]] bool fail_start(std::string message) {
    set_error(std::move(message));
    running_.store(false);
    return false;
  }

  void set_error(std::string message) {
    std::lock_guard lock(error_mutex_);
    last_error_ = std::move(message);
  }

  void accept_loop() noexcept {
    while (running_.load()) {
      sockaddr_in peer{};
      socklen_t peer_length = sizeof(peer);
      const int accepted =
          ::accept(listener_fd_.load(), reinterpret_cast<sockaddr*>(&peer), &peer_length);
      if (accepted == kNoSocket) {
        if (running_.load() && errno != EINTR) {
          set_error("TCP accept failed");
        }
        continue;
      }

#ifdef SO_NOSIGPIPE
      int no_sigpipe = 1;
      ::setsockopt(accepted, SOL_SOCKET, SO_NOSIGPIPE, &no_sigpipe, sizeof(no_sigpipe));
#endif
      timeval send_timeout{};
      send_timeout.tv_sec = static_cast<time_t>(config_.send_timeout.count() / 1000);
      send_timeout.tv_usec = static_cast<suseconds_t>((config_.send_timeout.count() % 1000) * 1000);
      if (::setsockopt(accepted, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout)) != 0) {
        close_socket(accepted);
        set_error("Could not set TCP send timeout");
        continue;
      }
      int camera_fd = -1;
      {
        std::lock_guard lock(client_mutex_);
        if (!running_.load()) { close_socket(accepted); break; }
        client_connected_.store(false);
        close_socket(std::exchange(client_fd_, accepted));
        geometry_worker_.reset();
        clear_pending();
        // Keep the sender excluded until the provenance is completely sent.
        // Old viewers know Hello and ignore its payload; new viewers default to
        // Unknown until this exact, bounded declaration arrives.
        const auto hello = wire::encode_source_hello(config_.source_kind,
            next_sequence_.fetch_add(1, std::memory_order_relaxed));
        if (send_all(accepted, hello)) {
          client_connected_.store(true);
          camera_fd = ::dup(accepted);
        } else {
          close_socket(std::exchange(client_fd_, kNoSocket));
          set_error("Could not send source declaration");
        }
      }
      camera_receiver_.replace(camera_fd);
    }
  }

  void sender_loop() noexcept {
    auto next_frame = std::chrono::steady_clock::now() + config_.frame_interval;
    while (running_.load()) {
      std::vector<atlas::sensors::LidarPoint> points;
      std::vector<std::vector<std::byte>> warning_messages;
      std::vector<std::byte> slam_status;
      std::vector<std::byte> geometry;
      uint64_t generation = 0;
      uint64_t timestamp_ns = 0;
      bool has_frame = false;
      {
        std::unique_lock lock(pending_mutex_);
        pending_cv_.wait_until(lock, next_frame, [this] {
          return !running_.load() || !pending_warning_messages_.empty() || !pending_slam_status_.empty() ||
                 !pending_geometry_.empty();
        });
        if (!running_.load()) {
          break;
        }
        warning_messages.swap(pending_warning_messages_);
        slam_status.swap(pending_slam_status_);
        geometry.swap(pending_geometry_);
        generation = client_generation_.load();
        if (std::chrono::steady_clock::now() >= next_frame) {
          points.swap(pending_points_);
          timestamp_ns = std::exchange(pending_timestamp_ns_, 0);
          has_frame = std::exchange(pending_frame_, false);
          next_frame = std::chrono::steady_clock::now() + config_.frame_interval;
        }
      }

      if (!slam_status.empty() && send_message(slam_status, generation))
        slam_statuses_sent_.fetch_add(1, std::memory_order_relaxed);

      if (!geometry.empty() && send_message(geometry, generation))
        geometry_snapshots_sent_.fetch_add(1, std::memory_order_relaxed);

      for (const auto& message : warning_messages) {
        if (send_message(message, generation)) {
          warnings_sent_.fetch_add(1, std::memory_order_relaxed);
        }
      }

      if (!has_frame || !client_connected_.load()) {
        continue;
      }
      auto visual_points = select_visual_points(points);
      if (visual_points.empty() && !config_.world_frame) {
        continue;
      }

      const auto sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
      const auto message = wire::encode_point_cloud(
          visual_points, sequence, timestamp_ns,
          config_.world_frame ? (wire::kPointCloudWorldFrame | wire::kPointCloudRgb) : 0);
      const bool sent = send_message(message, generation);
      if (sent) {
        frames_sent_.fetch_add(1, std::memory_order_relaxed);
        points_sent_.fetch_add(visual_points.size(), std::memory_order_relaxed);
      }
    }
  }

  [[nodiscard]] bool send_message(const std::span<const std::byte> message, uint64_t generation) noexcept {
    std::lock_guard lock(client_mutex_);
    if (client_fd_ == kNoSocket || generation != client_generation_.load()) {
      return false;
    }
    if (send_all(client_fd_, message)) {
      return true;
    }
    close_socket(std::exchange(client_fd_, kNoSocket));
    client_connected_.store(false);
    return false;
  }

  [[nodiscard]] std::vector<atlas::sensors::LidarPoint> select_visual_points(
      const std::vector<atlas::sensors::LidarPoint>& input) {
    std::vector<atlas::sensors::LidarPoint> valid;
    valid.reserve(std::min(input.size(), config_.max_points_per_frame));
    const float maximum_distance_squared = config_.maximum_distance_m * config_.maximum_distance_m;
    const auto visible = [&](const atlas::sensors::LidarPoint& point) {
      // World coordinates may be far from the original sensor origin; a raw
      // sensor-distance gate would silently erase valid parts of the map.
      return config_.world_frame
          ? std::isfinite(point.x_m) && std::isfinite(point.y_m) && std::isfinite(point.z_m)
          : is_visual_point(point, maximum_distance_squared);
    };

    size_t valid_count = 0;
    for (const auto& point : input) {
      if (visible(point)) {
        ++valid_count;
      }
    }
    const size_t stride = std::max<size_t>(
        1, (valid_count + config_.max_points_per_frame - 1) / config_.max_points_per_frame);
    size_t valid_index = 0;
    for (const auto& point : input) {
      if (!visible(point)) {
        continue;
      }
      if (valid_index % stride == 0 && valid.size() < config_.max_points_per_frame) {
        valid.push_back(point);
      }
      ++valid_index;
    }
    dropped_points_.fetch_add(input.size() - valid.size(), std::memory_order_relaxed);
    return valid;
  }

  void clear_pending() {
    std::lock_guard lock(pending_mutex_);
    // Sender captures this generation under the same lock as pending messages.
    client_generation_.fetch_add(1);
    dropped_points_.fetch_add(pending_points_.size(), std::memory_order_relaxed);
    pending_points_.clear();
    pending_warning_messages_.clear();
    pending_slam_status_.clear();
    pending_geometry_.clear();
    pending_timestamp_ns_ = 0;
    pending_frame_ = false;
  }

  void register_bonjour() noexcept {
#ifdef __APPLE__
    DNSServiceRef service = nullptr;
    const auto error =
        DNSServiceRegister(&service, 0, 0, config_.service_name.c_str(), "_atlas._tcp", nullptr,
                           nullptr, htons(config_.port), 0, nullptr, nullptr, nullptr);
    if (error == kDNSServiceErr_NoError) {
      bonjour_service_ = service;
    } else {
      set_error("Bonjour registration failed; manual IP connection remains available");
    }
#endif
  }

  void unregister_bonjour() noexcept {
#ifdef __APPLE__
    if (bonjour_service_ != nullptr) {
      DNSServiceRefDeallocate(bonjour_service_);
      bonjour_service_ = nullptr;
    }
#endif
  }

  UiStreamServerConfig config_;
  CameraReceiver camera_receiver_;
  LiveGeometryWorker geometry_worker_;
  std::atomic_bool running_{false};
  std::atomic_int listener_fd_{kNoSocket};
  std::thread accept_thread_;
  std::thread sender_thread_;
  mutable std::mutex client_mutex_;
  int client_fd_ = kNoSocket;
  std::atomic_bool client_connected_{false};
  std::atomic_uint64_t client_generation_{0};
  std::mutex pending_mutex_;
  std::condition_variable pending_cv_;
  std::vector<atlas::sensors::LidarPoint> pending_points_;
  std::vector<std::vector<std::byte>> pending_warning_messages_;
  std::vector<std::byte> pending_slam_status_;
  std::vector<std::byte> pending_geometry_;
  uint64_t pending_timestamp_ns_ = 0;
  bool pending_frame_ = false;
  std::atomic_uint32_t next_sequence_{1};
  std::atomic_uint64_t frames_sent_{0};
  std::atomic_uint64_t points_sent_{0};
  std::atomic_uint64_t dropped_points_{0};
  std::atomic_uint64_t warnings_sent_{0};
  std::atomic_uint64_t slam_statuses_sent_{0};
  std::atomic_uint64_t geometry_snapshots_sent_{0};
  mutable std::mutex error_mutex_;
  std::string last_error_;
#ifdef __APPLE__
  DNSServiceRef bonjour_service_ = nullptr;
#endif
};

UiStreamServer::UiStreamServer(UiStreamServerConfig config, CameraHandler handler)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(handler))) {}

UiStreamServer::~UiStreamServer() = default;
bool UiStreamServer::start() { return impl_->start(); }
void UiStreamServer::stop() noexcept { impl_->stop(); }
bool UiStreamServer::running() const noexcept { return impl_->running(); }
std::string UiStreamServer::last_error() const { return impl_->last_error(); }
UiStreamServerStats UiStreamServer::stats() const noexcept { return impl_->stats(); }
void UiStreamServer::on_lidar_frame(const atlas::sensors::LidarFrame& frame) {
  impl_->on_lidar_frame(frame);
}
void UiStreamServer::on_sensor_warning(const atlas::sensors::SensorWarning& warning) {
  impl_->on_sensor_warning(warning);
}
void UiStreamServer::publish_slam_status(const wire::SlamStatus& status) {
  impl_->publish_slam_status(status);
}

}  // namespace atlas::ui
