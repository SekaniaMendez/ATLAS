# Prefer an installed Eigen. Otherwise fetch a verified, fixed header-only release.
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
  if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
  endif()
  include(FetchContent)
  FetchContent_Declare(atlas_eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    SOURCE_SUBDIR atlas-header-only
  )
  FetchContent_MakeAvailable(atlas_eigen)
  add_library(atlas_eigen_headers INTERFACE)
  target_include_directories(atlas_eigen_headers SYSTEM INTERFACE "${atlas_eigen_SOURCE_DIR}")
  add_library(Eigen3::Eigen ALIAS atlas_eigen_headers)
endif()
