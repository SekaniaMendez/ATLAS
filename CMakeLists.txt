cmake_minimum_required(VERSION 3.20)

project(ATLAS
  VERSION 0.1.0
  DESCRIPTION "High-performance, cross-platform LiDAR mapping and SLAM engine"
  LANGUAGES C CXX
)

# ----------------------------
# Global config
# ----------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default build type (single-config generators like Ninja/Make)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ----------------------------
# Options
# ----------------------------
# Build toggles
option(ATLAS_BUILD_APPS  "Build ATLAS applications" ON)
option(ATLAS_BUILD_TESTS "Build ATLAS tests" OFF)

# Optional: enable hardware integration via the Livox SDK (e.g., Mid-360).
# - OFF by default to keep ATLAS ROS-independent and buildable without vendor SDKs.
# - When enabling this, make sure the Git submodule is initialized:
#     git submodule update --init --recursive
# - Then configure with:
#     cmake --preset mac-debug -DATLAS_ENABLE_LIVOX=ON
option(ATLAS_ENABLE_LIVOX "Enable Livox Mid-360 support" OFF)

# Treat warnings seriously (can tune per-compiler later)
option(ATLAS_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

# ----------------------------
# Subprojects
# ----------------------------
add_subdirectory(core)

# Vendor/third-party integrations
# NOTE: Livox-SDK2 is brought in as a Git submodule under `third_party/`.
# We intentionally build only `sdk_core/` to avoid compiling upstream samples.
# This is intentionally optional so contributors can build ATLAS without the SDK.
if(ATLAS_ENABLE_LIVOX)
  # Build only the vendor SDK core library (avoid building the upstream samples).
  # This keeps ATLAS builds faster and cleaner.
  add_subdirectory(third_party/Livox-SDK2/sdk_core)

  # Livox-SDK2 is a vendor dependency and currently enables -Werror internally.
  # On AppleClang/Clang this can break the build due to third-party warnings
  # (e.g., RapidJSON warning pragmas, libc++ deprecations, etc.).
  # We keep ATLAS strict, but relax warnings for the vendor targets only.
  if(APPLE)
    foreach(_livox_tgt livox_lidar_sdk_static livox_lidar_sdk_shared)
      if(TARGET ${_livox_tgt})
        target_compile_options(${_livox_tgt} PRIVATE
          -Wno-error
          -Wno-error=unknown-warning-option
          -Wno-error=deprecated-declarations
          -Wno-error=unused-private-field
          -Wno-error=non-c-typedef-for-linkage
          -Wno-unknown-warning-option
          -Wno-deprecated-declarations
          -Wno-unused-private-field
          -Wno-non-c-typedef-for-linkage
        )
      endif()
    endforeach()

    # Livox-SDK2 vendors an older fmt/spdlog bundle which is not compatible with C++20
    # on AppleClang/libc++ (e.g., uses std::result_of which was removed in C++20).
    # ATLAS itself stays on C++20, but we compile ONLY the vendor SDK targets as C++17.
    foreach(_livox_tgt livox_lidar_sdk_static livox_lidar_sdk_shared)
      if(TARGET ${_livox_tgt})
        set_property(TARGET ${_livox_tgt} PROPERTY CXX_STANDARD 17)
        set_property(TARGET ${_livox_tgt} PROPERTY CXX_STANDARD_REQUIRED ON)
        set_property(TARGET ${_livox_tgt} PROPERTY CXX_EXTENSIONS OFF)
      endif()
    endforeach()
  endif()
endif()

# ----------------------------
# Modules
# ----------------------------
if(ATLAS_ENABLE_LIVOX)
  add_subdirectory(modules/livox)
endif()

if(ATLAS_BUILD_APPS)
  add_subdirectory(apps)
endif()

if(ATLAS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()