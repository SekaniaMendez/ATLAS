#include "CameraReceiver.hpp"
#include <atlas/ui/CameraWireProtocol.hpp>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <cerrno>

namespace atlas::ui {
namespace {
bool receive_all(int fd, std::span<std::byte> bytes) {
  size_t count = 0;
  while (count < bytes.size()) {
    const auto n = ::recv(fd, bytes.data() + count, bytes.size() - count, 0);
    if (n > 0) count += size_t(n);
    else if (n < 0 && errno == EINTR) continue;
    else return false;
  }
  return true;
}
}
void CameraReceiver::stop() noexcept {
  if (fd_ >= 0) ::shutdown(fd_, SHUT_RDWR);
  if (thread_.joinable()) thread_.join();
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}
void CameraReceiver::replace(int owned_fd) {
  stop();
  fd_ = owned_fd;
  if (fd_ >= 0) thread_ = std::thread([this] { receive(); });
}
void CameraReceiver::receive() {
  try {
    std::array<std::byte, camera_wire::header_size> header;
    while (receive_all(fd_, header)) {
      const auto size = camera_wire::payload_size(header);
      if (!size) { ++rejected_; break; }
      std::vector<std::byte> payload(*size);
      if (!receive_all(fd_, payload)) break;
      auto frame = camera_wire::decode(payload);
      if (!frame) { ++rejected_; break; }
      if (handler_ && handler_(std::move(*frame))) ++received_;
      else ++rejected_;
    }
  } catch (...) { ++rejected_; }
  ::shutdown(fd_, SHUT_RDWR);
}
}
