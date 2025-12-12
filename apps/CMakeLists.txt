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
endif()