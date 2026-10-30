#pragma once
#include <atlas/sensors/CameraRgbFrame.hpp>
#include <atomic>
#include <functional>
#include <thread>

namespace atlas::ui {
// Lifecycle serialized by UiStreamServer: replace on accept thread, stop after
// joining it. Owns a duplicated descriptor, never a sender's recycled fd.
class CameraReceiver {
 public:
  using Handler = std::function<bool(sensors::CameraRgbFrame)>;
  explicit CameraReceiver(Handler handler) : handler_(std::move(handler)) {}
  ~CameraReceiver() { stop(); }
  void replace(int owned_fd);
  void stop() noexcept;
  uint64_t received() const { return received_.load(); }
  uint64_t rejected() const { return rejected_.load(); }
 private:
  void receive();
  Handler handler_;
  int fd_ = -1;
  std::thread thread_;
  std::atomic<uint64_t> received_{0}, rejected_{0};
};
}
