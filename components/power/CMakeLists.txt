idf_component_register(SRCS "power.c"
                       INCLUDE_DIRS "include"
                       REQUIRES obd
                       PRIV_REQUIRES esp_adc esp_timer nvs_flash config events imu motion)
