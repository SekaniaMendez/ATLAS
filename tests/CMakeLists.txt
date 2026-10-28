add_executable(atlas_sensor_data_bus_test
  SensorDataBusTest.cpp
)

target_link_libraries(atlas_sensor_data_bus_test PRIVATE atlas_core)
target_compile_features(atlas_sensor_data_bus_test PRIVATE cxx_std_20)

add_test(NAME atlas_sensor_data_bus_test COMMAND atlas_sensor_data_bus_test)

foreach(slam_test VoxelRegistration VoxelSlam)
  add_executable(atlas_${slam_test}_test ${slam_test}Test.cpp)
  target_link_libraries(atlas_${slam_test}_test PRIVATE atlas_core Eigen3::Eigen)
  target_include_directories(atlas_${slam_test}_test PRIVATE ${PROJECT_SOURCE_DIR}/core/slam/src)
  add_test(NAME atlas_${slam_test}_test COMMAND atlas_${slam_test}_test)
  if(NOT MSVC)
    target_compile_options(atlas_${slam_test}_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endforeach()

add_executable(atlas_sensor_preprocessor_test
  SensorPreprocessorTest.cpp
)

target_link_libraries(atlas_sensor_preprocessor_test PRIVATE atlas_core)
target_compile_features(atlas_sensor_preprocessor_test PRIVATE cxx_std_20)

add_test(NAME atlas_sensor_preprocessor_test COMMAND atlas_sensor_preprocessor_test)

add_executable(atlas_sparse_voxel_map_test
  SparseVoxelMapTest.cpp
)

target_link_libraries(atlas_sparse_voxel_map_test PRIVATE atlas_core)
target_compile_features(atlas_sparse_voxel_map_test PRIVATE cxx_std_20)

add_test(NAME atlas_sparse_voxel_map_test COMMAND atlas_sparse_voxel_map_test)

add_executable(atlas_sparse_voxel_room_test SparseVoxelRoomTest.cpp)
target_link_libraries(atlas_sparse_voxel_room_test PRIVATE atlas_core)
target_compile_features(atlas_sparse_voxel_room_test PRIVATE cxx_std_20)
add_test(NAME atlas_sparse_voxel_room_test COMMAND atlas_sparse_voxel_room_test)

foreach(voxel_test atlas_sparse_voxel_map_test atlas_sparse_voxel_room_test)
  if(MSVC)
    target_compile_options(${voxel_test} PRIVATE /W4)
    if(ATLAS_WARNINGS_AS_ERRORS)
      target_compile_options(${voxel_test} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${voxel_test} PRIVATE -Wall -Wextra -Wpedantic)
    if(ATLAS_WARNINGS_AS_ERRORS)
      target_compile_options(${voxel_test} PRIVATE -Werror)
    endif()
  endif()
endforeach()

foreach(storage_test Capture VoxelStorage Synchronization)
  add_executable(atlas_${storage_test}_test ${storage_test}Test.cpp)
  target_link_libraries(atlas_${storage_test}_test PRIVATE atlas_core)
  add_test(NAME atlas_${storage_test}_test COMMAND atlas_${storage_test}_test)
  if(NOT MSVC)
    target_compile_options(atlas_${storage_test}_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endforeach()
find_package(SQLite3 REQUIRED)
if(TARGET SQLite3::SQLite3)
  target_link_libraries(atlas_VoxelStorage_test PRIVATE SQLite3::SQLite3)
else()
  target_link_libraries(atlas_VoxelStorage_test PRIVATE SQLite::SQLite3)
endif()

if(TARGET atlas_ui_stream)
  add_executable(atlas_ui_wire_protocol_test
    UiWireProtocolTest.cpp
  )
  target_link_libraries(atlas_ui_wire_protocol_test PRIVATE atlas_ui_stream)
  target_compile_features(atlas_ui_wire_protocol_test PRIVATE cxx_std_20)
  # This protocol test uses assert; keep its checks active in optimized builds.
  if(MSVC)
    target_compile_options(atlas_ui_wire_protocol_test PRIVATE /UNDEBUG)
  else()
    target_compile_options(atlas_ui_wire_protocol_test PRIVATE -UNDEBUG)
  endif()
  add_test(NAME atlas_ui_wire_protocol_test COMMAND atlas_ui_wire_protocol_test)
endif()
