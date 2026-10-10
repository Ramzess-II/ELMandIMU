idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash esp_timer esp_app_format config events imu led motion net obd power
                                     update)
