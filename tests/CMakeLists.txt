add_executable(atlas_sensor_data_bus_test
  SensorDataBusTest.cpp
)

target_link_libraries(atlas_sensor_data_bus_test PRIVATE atlas_core)
target_compile_features(atlas_sensor_data_bus_test PRIVATE cxx_std_20)

add_test(NAME atlas_sensor_data_bus_test COMMAND atlas_sensor_data_bus_test)

if(TARGET atlas_ui_stream)
  add_executable(atlas_ui_wire_protocol_test
    UiWireProtocolTest.cpp
  )
  target_link_libraries(atlas_ui_wire_protocol_test PRIVATE atlas_ui_stream)
  target_compile_features(atlas_ui_wire_protocol_test PRIVATE cxx_std_20)
  add_test(NAME atlas_ui_wire_protocol_test COMMAND atlas_ui_wire_protocol_test)
endif()
