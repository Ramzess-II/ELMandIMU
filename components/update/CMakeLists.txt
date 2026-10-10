idf_component_register(SRCS "update.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES app_update bootloader_support esp_app_format esp_partition esp_timer
                                     mbedtls nvs_flash
                                     config events)
