# ATLAS applications

# ----------------------------
# Livox probe
# ----------------------------
# Only build this app when Livox support is enabled.
if(ATLAS_ENABLE_LIVOX)
  add_executable(atlas_livox_probe
    livox_probe/atlas_livox_probe.cpp
  )

  target_link_libraries(atlas_livox_probe
    PRIVATE
      atlas_livox
      livox_lidar_sdk_shared
  )

  if(TARGET atlas_ui_stream)
    target_link_libraries(atlas_livox_probe PRIVATE atlas_ui_stream)
    target_compile_definitions(atlas_livox_probe PRIVATE ATLAS_HAS_UI_STREAM=1)
  endif()
endif()

# ----------------------------
# UI streaming demo
# ----------------------------
if(TARGET atlas_ui_stream)
  add_executable(atlas_ui_demo
    ui_demo/atlas_ui_demo.cpp
  )
  target_link_libraries(atlas_ui_demo PRIVATE atlas_ui_stream)
endif()
