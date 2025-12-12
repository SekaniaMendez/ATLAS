/// @file atlas_livox_probe.cpp
/// @brief Simple Livox LiDAR probe application.
///
/// This application initializes the Livox LiDAR SDK, listens for device discovery,
/// enables point and IMU data streaming on the first discovered device, and prints
/// basic information and data callbacks to the console.
///
/// It does NOT perform advanced data processing, visualization, or storage.

#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

namespace {
/// @brief Atomic flag indicating whether the application is running.
/// Used for clean shutdown on Ctrl+C.
std::atomic_bool g_running{true};

/// @brief Atomic variable storing the handle of the first discovered device.
/// Used to enable streaming only once.
std::atomic_uint32_t g_first_handle{0};

/// @brief Signal handler for SIGINT (Ctrl+C).
/// @param signum The signal number (ignored).
/// Sets the running flag to false to terminate the main loop.
void OnSignal(int) { g_running = false; }

/// @brief Async control callback used by many SDK control functions.
/// Prints the status and handle of the async control response.
/// @param status Status code returned by the SDK operation.
/// @param handle Device handle associated with the async control.
/// @param response Pointer to the async control response (unused).
/// @param client_data User data passed to the callback (unused).
void OnAsyncCtrl(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* /*response*/,
                 void* /*client_data*/) {
  std::cout << "[Livox] async: status=" << int(status) << " handle=" << handle << "\n";
}

/// @brief Device info callback invoked when device info is received.
/// Prints the device handle, device type, and optional info string.
/// @param handle Device handle.
/// @param dev_type Device type identifier.
/// @param info Optional device info string (often JSON-like).
/// @param client_data User data passed to the callback (unused).
void OnInfo(const uint32_t handle, const uint8_t dev_type, const char* info,
            void* /*client_data*/) {
  std::cout << "[Livox] info: handle=" << handle << " dev_type=" << unsigned(dev_type);
  if (info) {
    std::cout << " info=" << info;
  }
  std::cout << "\n";
}

/// @brief Callback fired when a device is discovered or updated.
/// Enables streaming on the first discovered device.
/// @param handle Device handle.
/// @param info Pointer to LivoxLidarInfo struct containing device info.
/// @param client_data User data passed to the callback (unused).
void OnInfoChange(const uint32_t handle, const LivoxLidarInfo* info, void* /*client_data*/) {
  std::cout << "[Livox] info-change: handle=" << handle;
  if (info) {
    std::cout << " dev_type=" << unsigned(info->dev_type);
  }
  std::cout << "\n";

  // Enable streaming once for the first discovered device.
  uint32_t expected = 0;
  if (g_first_handle.compare_exchange_strong(expected, handle)) {
    std::cout << "[Livox] first device discovered (handle=" << handle << ")\n";

    auto st_points = EnableLivoxLidarPointSend(handle, OnAsyncCtrl, nullptr);
    std::cout << "[Livox] EnableLivoxLidarPointSend -> status=" << int(st_points) << "\n";

    auto st_imu = EnableLivoxLidarImuData(handle, OnAsyncCtrl, nullptr);
    std::cout << "[Livox] EnableLivoxLidarImuData -> status=" << int(st_imu) << "\n";

    // Optional: choose point cloud data type. Leave commented until you confirm desired enum.
    // auto st_type = SetLivoxLidarPclDataType(handle, kLivoxLidarPointDataTypeCartesian,
    // OnAsyncCtrl, nullptr); std::cout << "[Livox] SetLivoxLidarPclDataType -> status=" <<
    // int(st_type) << "\n";
  }
}

/// @brief IMU data callback invoked when IMU packets arrive.
/// Prints device handle, device type, data type, and timestamp.
/// @param handle Device handle.
/// @param dev_type Device type identifier.
/// @param data Pointer to LivoxLidarEthernetPacket containing IMU data.
/// @param client_data User data passed to the callback (unused).
void OnImu(const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data,
           void* /*client_data*/) {
  std::cout << "[Livox] IMU: handle=" << handle << " dev_type=" << unsigned(dev_type);
  if (data) {
    std::cout << " data_type=" << unsigned(data->data_type) << " ts=" << data->timestamp;
  }
  std::cout << "\n";
}

}  // namespace

/// @brief Main entry point of the Livox LiDAR probe application.
///
/// Usage:
///   ./atlas_livox_probe [config_json_path]
///
/// If a config JSON path is provided as the first argument, it is passed to the SDK initialization.
/// The application installs signal handlers to gracefully exit on Ctrl+C (SIGINT).
/// It then waits in a loop, printing device info and data callbacks, until terminated.
///
/// @param argc Argument count.
/// @param argv Argument vector.
/// @return Exit code (0 on success, 1 on failure).
int main(int argc, char** argv) {
  std::signal(SIGINT, OnSignal);

  // If your SDK requires a config JSON, pass it as argv[1].
  // Example:
  //   ./atlas_livox_probe ../../config/mid360_config.json
  const char* config_path = (argc >= 2) ? argv[1] : nullptr;

  if (!LivoxLidarSdkInit(config_path)) {
    std::cerr << "[Livox] LivoxLidarSdkInit failed";
    if (config_path) {
      std::cerr << " (config_path='" << config_path << "')";
    }
    std::cerr << "\n";
    return 1;
  }

  SetLivoxLidarInfoCallback(OnInfo, nullptr);
  SetLivoxLidarInfoChangeCallback(OnInfoChange, nullptr);
  SetLivoxLidarImuDataCallback(OnImu, nullptr);

  std::cout << "[Livox] SDK initialized. Waiting for device... (Ctrl+C to exit)\n";

  while (g_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::cout << "[Livox] shutting down...\n";
  LivoxLidarSdkUninit();
  return 0;
}