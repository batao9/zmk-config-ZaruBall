if(CONFIG_ZMK_INPUT_PROCESSOR_MOTION_THRESHOLD)
  zephyr_library()
  zephyr_library_sources(src/input_processor_motion_threshold.c)
endif()
