idf_component_register(SRCS "motion.c"
                       INCLUDE_DIRS "include"
                       REQUIRES imu
                       PRIV_REQUIRES calib config events esp_timer)
