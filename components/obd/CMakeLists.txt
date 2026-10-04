idf_component_register(SRCS "obd.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_timer nvs_flash config events)
