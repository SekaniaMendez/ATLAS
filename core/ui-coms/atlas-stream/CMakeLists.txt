add_library(atlas_ui_stream STATIC)

target_sources(atlas_ui_stream
  PRIVATE
    src/UiStreamServer.cpp
    src/WireProtocol.cpp
)

target_include_directories(atlas_ui_stream
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(atlas_ui_stream PUBLIC atlas_core)
target_compile_features(atlas_ui_stream PUBLIC cxx_std_20)

if(MSVC)
  target_compile_options(atlas_ui_stream PRIVATE /W4)
else()
  target_compile_options(atlas_ui_stream PRIVATE -Wall -Wextra -Wpedantic)
endif()
