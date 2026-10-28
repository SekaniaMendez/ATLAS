add_library(atlas_core STATIC)

# Core sources
set(ATLAS_CORE_SOURCES
  common/src/atlas_dummy.cpp
  common/src/SensorDataBus.cpp
  preprocessing/src/ImuMotionMonitor.cpp
  preprocessing/src/LiDARFilter.cpp
  preprocessing/src/SensorPreprocessor.cpp
  slam/src/SparseVoxelMap.cpp
  slam/src/VoxelRegistration.cpp
  slam/src/VoxelSlamEngine.cpp
  storage/src/Capture.cpp
  storage/src/CaptureCodec.cpp
  storage/src/VoxelMapArchive.cpp
  storage/src/PagedVoxelMap.cpp
  timesync/src/SynchronizationOrchestrator.cpp
)

# GNSS (core) sources (add files here as you create them)
list(APPEND ATLAS_CORE_SOURCES
  gnss/src/NmeaSerialReader.cpp
  # gnss/src/quality.cpp
  # gnss/src/providers/nmea_serial_gnss_source.cpp
)

target_sources(atlas_core
  PRIVATE
    ${ATLAS_CORE_SOURCES}
)

target_include_directories(atlas_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/imu/include
    ${CMAKE_CURRENT_SOURCE_DIR}/preprocessing/include
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss/include
    ${CMAKE_CURRENT_SOURCE_DIR}/timesync/include
    ${CMAKE_CURRENT_SOURCE_DIR}/slam/include
    ${CMAKE_CURRENT_SOURCE_DIR}/storage/include
)

find_package(Threads REQUIRED)
include(${PROJECT_SOURCE_DIR}/cmake/AtlasEigen.cmake)
find_package(SQLite3 REQUIRED)
target_link_libraries(atlas_core PRIVATE Threads::Threads)
target_link_libraries(atlas_core PRIVATE Eigen3::Eigen)
if(TARGET SQLite3::SQLite3)
  target_link_libraries(atlas_core PRIVATE SQLite3::SQLite3)
else()
  target_link_libraries(atlas_core PRIVATE SQLite::SQLite3)
endif()

target_compile_features(atlas_core PUBLIC cxx_std_20)

# NOTE: Append new implementation files from the corresponding core domain above.

# Warnings (mínimo, portable)
if(MSVC)
  target_compile_options(atlas_core PRIVATE /W4)
  if(ATLAS_WARNINGS_AS_ERRORS)
    target_compile_options(atlas_core PRIVATE /WX)
  endif()
else()
  target_compile_options(atlas_core PRIVATE -Wall -Wextra -Wpedantic)
  if(ATLAS_WARNINGS_AS_ERRORS)
    target_compile_options(atlas_core PRIVATE -Werror)
  endif()
endif()
