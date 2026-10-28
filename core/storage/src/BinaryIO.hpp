#pragma once

#include <bit>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace atlas::storage::binary {
inline constexpr size_t kMaxRecordBytes = 64 * 1024 * 1024;

struct Encoder {
  std::vector<uint8_t> data;
  void u8(uint8_t v) { data.push_back(v); }
  void u32(uint32_t v) { for (int i = 0; i < 4; ++i) u8(static_cast<uint8_t>(v >> (8 * i))); }
  void u64(uint64_t v) { for (int i = 0; i < 8; ++i) u8(static_cast<uint8_t>(v >> (8 * i))); }
  void f32(float v) { u32(std::bit_cast<uint32_t>(v)); }
  void f64(double v) { u64(std::bit_cast<uint64_t>(v)); }
  void string(const std::string& v) {
    if (v.size() > kMaxRecordBytes) throw std::runtime_error("String exceeds archive limit");
    u32(static_cast<uint32_t>(v.size()));
    data.insert(data.end(), v.begin(), v.end());
  }
};

struct Decoder {
  std::span<const uint8_t> data;
  size_t position = 0;
  size_t remaining() const { return data.size() - position; }
  uint8_t u8() {
    if (remaining() == 0) throw std::runtime_error("Truncated archive payload");
    return data[position++];
  }
  uint32_t u32() { uint32_t v = 0; for (int i = 0; i < 4; ++i) v |= uint32_t(u8()) << (8 * i); return v; }
  uint64_t u64() { uint64_t v = 0; for (int i = 0; i < 8; ++i) v |= uint64_t(u8()) << (8 * i); return v; }
  float f32() { return std::bit_cast<float>(u32()); }
  double f64() { return std::bit_cast<double>(u64()); }
  std::string string() {
    const auto size = u32();
    if (size > remaining()) throw std::runtime_error("Truncated archive string");
    std::string v(reinterpret_cast<const char*>(data.data() + position), size);
    position += size;
    return v;
  }
  void finish() const { if (remaining()) throw std::runtime_error("Unexpected archive trailing fields"); }
};

inline uint32_t crc32(std::span<const uint8_t> data) {
  uint32_t crc = 0xffffffffU;
  for (const auto value : data) {
    crc ^= value;
    for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}

inline void write_bytes(std::ostream& output, std::span<const uint8_t> data) {
  output.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
  if (!output) throw std::runtime_error("Archive write failed (disk full or I/O error)");
}

inline std::vector<uint8_t> read_bytes(std::istream& input, size_t size) {
  if (size > kMaxRecordBytes) throw std::runtime_error("Archive record exceeds size limit");
  std::vector<uint8_t> data(size);
  input.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(size));
  if (!input) throw std::runtime_error("Truncated archive");
  return data;
}

inline void write_record(std::ostream& output, const Encoder& payload) {
  if (payload.data.size() > kMaxRecordBytes) throw std::runtime_error("Archive record exceeds size limit");
  Encoder header;
  header.u32(static_cast<uint32_t>(payload.data.size()));
  header.u32(crc32(payload.data));
  write_bytes(output, header.data);
  write_bytes(output, payload.data);
}

inline std::vector<uint8_t> read_record(std::istream& input) {
  const auto bytes = read_bytes(input, 8);
  Decoder header{bytes};
  const auto size = header.u32();
  const auto crc = header.u32();
  auto data = read_bytes(input, size);
  if (crc32(data) != crc) throw std::runtime_error("Archive checksum mismatch");
  return data;
}

inline void magic(std::ostream& output, const std::string& value) {
  output.write(value.data(), static_cast<std::streamsize>(value.size()));
  if (!output) throw std::runtime_error("Archive header write failed");
}
inline void expect_magic(std::istream& input, const std::string& value) {
  const auto data = read_bytes(input, value.size());
  if (std::string(data.begin(), data.end()) != value) throw std::runtime_error("Unsupported archive format/version");
}
inline void expect_eof(std::istream& input) {
  if (input.peek() != std::char_traits<char>::eof() || input.bad())
    throw std::runtime_error("Unexpected archive trailing data or read error");
}

/// Create-new directory is also the no-overwrite/exclusive-writer primitive.
inline void create_new_directory(const std::filesystem::path& path) {
  if (!std::filesystem::create_directory(path)) throw std::runtime_error("Directory already exists: " + path.string());
}
}  // namespace atlas::storage::binary
