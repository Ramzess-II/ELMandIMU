idf_component_register(SRCS "net.c" "ble.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_timer bt spi_flash
                                     config events imu motion obd power proto update)
