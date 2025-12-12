# ATLAS Livox adapter module
# -------------------------
# This module wraps the Livox SDK and exposes it to ATLAS without
# leaking vendor headers or dependencies into atlas_core.

add_library(atlas_livox STATIC)

# Sources for the Livox adapter
# NOTE: Keep implementation files inside src/
# to clearly separate public API from private code.
target_sources(atlas_livox
  PRIVATE
    src/LivoxSource.cpp
)

# Public include path for ATLAS modules/apps that want to use Livox
# (does NOT expose Livox SDK headers)
target_include_directories(atlas_livox
  PUBLIC
    ${PROJECT_SOURCE_DIR}/modules/livox/include
  PRIVATE
    ${PROJECT_SOURCE_DIR}/third_party/Livox-SDK2/sdk_core/include
)

# Link against ATLAS core and the Livox SDK
# The SDK itself is built separately (as a vendor dependency).
target_link_libraries(atlas_livox
  PRIVATE
    atlas_core
    livox_lidar_sdk_shared
)

# ATLAS modules use C++20
# (the Livox SDK target itself is forced to C++17 in the root CMakeLists)
target_compile_features(atlas_livox PUBLIC cxx_std_20)