add_library(atlas_core STATIC)

target_sources(atlas_core
  PRIVATE
    src/atlas_dummy.cpp
)

target_include_directories(atlas_core
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_compile_features(atlas_core PUBLIC cxx_std_20)

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