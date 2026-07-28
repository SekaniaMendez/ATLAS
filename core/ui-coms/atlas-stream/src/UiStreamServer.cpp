#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atlas/ui/UiStreamServer.hpp>
#include <atlas/ui/WireProtocol.hpp>

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
  explicit Impl(UiStreamServerConfig config) : config_(std::move(config)) {}
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
        .client_connected = client_connected_.load(),
    };
  }

  void on_lidar_frame(const atlas::sensors::LidarFrame& frame) {
    if (!running_.load() || !client_connected_.load()) {
      return;
    }

    std::lock_guard lock(pending_mutex_);
    if (pending_points_.empty()) {
      pending_timestamp_ns_ = frame.timestamp.nanoseconds;
    }
    const auto available =
        config_.max_pending_points - std::min(config_.max_pending_points, pending_points_.size());
    const auto accepted = std::min(available, frame.points.size());
    pending_points_.insert(pending_points_.end(), frame.points.begin(),
                           frame.points.begin() + static_cast<std::ptrdiff_t>(accepted));
    dropped_points_.fetch_add(frame.points.size() - accepted, std::memory_order_relaxed);
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
      {
        std::lock_guard lock(client_mutex_);
        close_socket(std::exchange(client_fd_, accepted));
        client_connected_.store(true);
      }
      clear_pending();
    }
  }

  void sender_loop() noexcept {
    auto next_frame = std::chrono::steady_clock::now() + config_.frame_interval;
    while (running_.load()) {
      std::vector<atlas::sensors::LidarPoint> points;
      uint64_t timestamp_ns = 0;
      {
        std::unique_lock lock(pending_mutex_);
        pending_cv_.wait_until(lock, next_frame, [this] { return !running_.load(); });
        if (!running_.load()) {
          break;
        }
        points.swap(pending_points_);
        timestamp_ns = std::exchange(pending_timestamp_ns_, 0);
      }
      next_frame = std::chrono::steady_clock::now() + config_.frame_interval;

      if (points.empty() || !client_connected_.load()) {
        continue;
      }
      auto visual_points = select_visual_points(points);
      if (visual_points.empty()) {
        continue;
      }

      const auto sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
      const auto message = wire::encode_point_cloud(visual_points, sequence, timestamp_ns);
      bool sent = false;
      {
        std::lock_guard lock(client_mutex_);
        if (client_fd_ != kNoSocket) {
          sent = send_all(client_fd_, message);
          if (!sent) {
            close_socket(std::exchange(client_fd_, kNoSocket));
            client_connected_.store(false);
          }
        }
      }
      if (sent) {
        frames_sent_.fetch_add(1, std::memory_order_relaxed);
        points_sent_.fetch_add(visual_points.size(), std::memory_order_relaxed);
      }
    }
  }

  [[nodiscard]] std::vector<atlas::sensors::LidarPoint> select_visual_points(
      const std::vector<atlas::sensors::LidarPoint>& input) {
    std::vector<atlas::sensors::LidarPoint> valid;
    valid.reserve(std::min(input.size(), config_.max_points_per_frame));
    const float maximum_distance_squared = config_.maximum_distance_m * config_.maximum_distance_m;

    size_t valid_count = 0;
    for (const auto& point : input) {
      if (is_visual_point(point, maximum_distance_squared)) {
        ++valid_count;
      }
    }
    const size_t stride = std::max<size_t>(
        1, (valid_count + config_.max_points_per_frame - 1) / config_.max_points_per_frame);
    size_t valid_index = 0;
    for (const auto& point : input) {
      if (!is_visual_point(point, maximum_distance_squared)) {
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
    dropped_points_.fetch_add(pending_points_.size(), std::memory_order_relaxed);
    pending_points_.clear();
    pending_timestamp_ns_ = 0;
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
  std::atomic_bool running_{false};
  std::atomic_int listener_fd_{kNoSocket};
  std::thread accept_thread_;
  std::thread sender_thread_;
  mutable std::mutex client_mutex_;
  int client_fd_ = kNoSocket;
  std::atomic_bool client_connected_{false};
  std::mutex pending_mutex_;
  std::condition_variable pending_cv_;
  std::vector<atlas::sensors::LidarPoint> pending_points_;
  uint64_t pending_timestamp_ns_ = 0;
  std::atomic_uint32_t next_sequence_{1};
  std::atomic_uint64_t frames_sent_{0};
  std::atomic_uint64_t points_sent_{0};
  std::atomic_uint64_t dropped_points_{0};
  mutable std::mutex error_mutex_;
  std::string last_error_;
#ifdef __APPLE__
  DNSServiceRef bonjour_service_ = nullptr;
#endif
};

UiStreamServer::UiStreamServer(UiStreamServerConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

UiStreamServer::~UiStreamServer() = default;
bool UiStreamServer::start() { return impl_->start(); }
void UiStreamServer::stop() noexcept { impl_->stop(); }
bool UiStreamServer::running() const noexcept { return impl_->running(); }
std::string UiStreamServer::last_error() const { return impl_->last_error(); }
UiStreamServerStats UiStreamServer::stats() const noexcept { return impl_->stats(); }
void UiStreamServer::on_lidar_frame(const atlas::sensors::LidarFrame& frame) {
  impl_->on_lidar_frame(frame);
}

}  // namespace atlas::ui
