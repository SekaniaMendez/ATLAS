#include "CaptureCodec.hpp"
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace atlas::storage {
class CaptureRecorder::Impl {
 public:
  Impl(const std::filesystem::path& directory, const std::string& metadata, size_t budget)
      : budget_(budget), start_(std::chrono::steady_clock::now()) {
    if (budget == 0) throw std::invalid_argument("Capture queue budget must be positive");
    binary::Encoder manifest; manifest.string(metadata);
    if (manifest.data.size() > binary::kMaxRecordBytes) throw std::invalid_argument("Capture metadata too large");
    binary::create_new_directory(directory);
    output_.open(directory / "events.atlas", std::ios::binary | std::ios::trunc);
    binary::magic(output_, "ATLCAP01"); binary::write_record(output_, manifest);
    output_.flush();
    if (!output_) throw std::runtime_error("Cannot write capture manifest");
    worker_ = std::thread([this] { run(); });
  }
  ~Impl() { try { close(); } catch (...) {} }

  template <typename T> void enqueue(const T& data) {
    std::lock_guard lock(mutex_);
    if (stats_.failed || closing_) return;
    try {
      const auto now = std::chrono::steady_clock::now();
      CaptureEvent event{stats_.accepted_events,
          static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_).count()),
          data};
      auto payload = encode_event(event);
      const size_t bytes = payload.data.capacity() + sizeof(binary::Encoder);
      if (bytes > budget_ - stats_.pending_bytes) throw std::runtime_error("Capture queue overflow: recording incomplete");
      queue_.push_back(std::move(payload));
      stats_.pending_bytes += bytes;
      stats_.peak_pending_bytes = std::max(stats_.peak_pending_bytes, stats_.pending_bytes);
      ++stats_.accepted_events;
    } catch (const std::exception& e) {
      stats_.failed = true; stats_.error = e.what();
    }
    ready_.notify_one();
  }
  CaptureStats stats() const { std::lock_guard lock(mutex_); return stats_; }
  void close() {
    std::lock_guard close_lock(close_mutex_);
    { std::lock_guard lock(mutex_); closing_ = true; }
    ready_.notify_one();
    if (worker_.joinable()) worker_.join();
    if (output_.is_open()) output_.close(); // retain readable prefix on failure
    const auto status = stats();
    if (status.failed) throw std::runtime_error(status.error);
  }
 private:
  void run() noexcept {
    try {
      for (;;) {
        binary::Encoder payload;
        {
          std::unique_lock lock(mutex_);
          ready_.wait(lock, [&] { return closing_ || stats_.failed || !queue_.empty(); });
          if (stats_.failed) return; // no completion marker on failed captures
          if (queue_.empty()) break;
          payload = std::move(queue_.front()); queue_.pop_front();
        }
        binary::write_record(output_, payload);
        { std::lock_guard lock(mutex_);
          stats_.pending_bytes -= payload.data.capacity() + sizeof(binary::Encoder);
          ++stats_.written_events;
        }
      }
      binary::Encoder footer; footer.u8(0); footer.u64(stats().written_events);
      binary::write_record(output_, footer);
      output_.flush(); output_.close();
      if (!output_) throw std::runtime_error("Capture close failed");
    } catch (const std::exception& e) {
      std::lock_guard lock(mutex_); stats_.failed = true; stats_.error = e.what();
    }
  }
  size_t budget_;
  std::chrono::steady_clock::time_point start_;
  std::ofstream output_;
  mutable std::mutex mutex_;
  std::mutex close_mutex_;
  std::condition_variable ready_;
  std::deque<binary::Encoder> queue_;
  CaptureStats stats_;
  bool closing_ = false;
  std::thread worker_;
};

CaptureRecorder::CaptureRecorder(const std::filesystem::path& directory, std::string metadata, size_t budget)
    : impl_(std::make_unique<Impl>(directory, metadata, budget)) {}
CaptureRecorder::~CaptureRecorder() = default;
void CaptureRecorder::on_lidar_frame(const sensors::LidarFrame& v) { impl_->enqueue(v); }
void CaptureRecorder::on_imu_sample(const imu::ImuSample& v) { impl_->enqueue(v); }
void CaptureRecorder::on_sensor_status(const sensors::SensorStatus& v) { impl_->enqueue(v); }
void CaptureRecorder::on_sensor_warning(const sensors::SensorWarning& v) { impl_->enqueue(v); }
void CaptureRecorder::on_camera_frame(const sensors::CameraFrame& v) { impl_->enqueue(v); }
CaptureStats CaptureRecorder::stats() const { return impl_->stats(); }
void CaptureRecorder::close() { impl_->close(); }

class CaptureReader::Impl {
 public:
  explicit Impl(const std::filesystem::path& directory) : input(directory / "events.atlas", std::ios::binary) {
    binary::expect_magic(input, "ATLCAP01");
    const auto bytes = binary::read_record(input); binary::Decoder d{bytes};
    manifest = d.string(); d.finish();
  }
  std::ifstream input;
  std::string manifest;
  uint64_t count = 0, last_offset = 0;
  bool done = false;
};
CaptureReader::CaptureReader(const std::filesystem::path& directory) : impl_(std::make_unique<Impl>(directory)) {}
CaptureReader::~CaptureReader() = default;
const std::string& CaptureReader::metadata() const { return impl_->manifest; }
std::optional<CaptureEvent> CaptureReader::next() {
  if (impl_->done) return std::nullopt;
  const auto bytes = binary::read_record(impl_->input);
  if (bytes.empty()) throw std::runtime_error("Empty capture record");
  if (bytes[0] == 0) {
    binary::Decoder d{bytes}; (void)d.u8();
    if (d.u64() != impl_->count) throw std::runtime_error("Capture event count mismatch");
    d.finish(); binary::expect_eof(impl_->input); impl_->done = true; return std::nullopt;
  }
  auto event = decode_event(bytes);
  if (event.sequence != impl_->count || event.arrival_offset_ns < impl_->last_offset)
    throw std::runtime_error("Capture sequence/timing mismatch");
  ++impl_->count; impl_->last_offset = event.arrival_offset_ns;
  return event;
}
}  // namespace atlas::storage
