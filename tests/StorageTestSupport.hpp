#pragma once
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#define REQUIRE(...) do { if (!(__VA_ARGS__)) throw std::runtime_error(std::string("line ") + std::to_string(__LINE__) + ": " + #__VA_ARGS__); } while (false)
template <typename F> void rejects(F fn) { bool failed = false; try { fn(); } catch (const std::exception&) { failed = true; } REQUIRE(failed); }
struct TempDirectory {
  std::filesystem::path path;
  TempDirectory() {
    const auto id = std::chrono::steady_clock::now().time_since_epoch().count();
    for (unsigned i = 0; ; ++i) {
      path = std::filesystem::temp_directory_path() / ("atlas-storage-test-" + std::to_string(id) + "-" + std::to_string(i));
      if (std::filesystem::create_directory(path)) break;
    }
  }
  ~TempDirectory() { std::error_code ignored; std::filesystem::remove_all(path, ignored); }
};
