if(CONFIG_INPUT_HANC_TRACKPOINT_I2C OR CONFIG_ZMK_BEHAVIOR_TP_TUNE)
    zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

add_subdirectory_ifdef(CONFIG_INPUT_HANC_TRACKPOINT_I2C drivers/input)
add_subdirectory_ifdef(CONFIG_ZMK_BEHAVIOR_TP_TUNE      drivers/behaviors)
