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
option(ATLAS_BUILD_APPS  "Build ATLAS applications" ON)
option(ATLAS_BUILD_TESTS "Build ATLAS tests" OFF)

# Treat warnings seriously (can tune per-compiler later)
option(ATLAS_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

# ----------------------------
# Subprojects
# ----------------------------
add_subdirectory(core)

if(ATLAS_BUILD_APPS)
  add_subdirectory(apps)
endif()

if(ATLAS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()