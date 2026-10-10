idf_component_register(SRCS "imu.c"
                            "imu_bus.c"
                            "imu_registry.c"
                            "drivers/imu_icm20602.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_timer events)
