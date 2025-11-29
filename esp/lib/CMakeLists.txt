idf_component_register(SRCS "mf_motor.c" "gy25.c" "vl53l0x.c"
                      INCLUDE_DIRS "."
                      REQUIRES driver esp_timer)