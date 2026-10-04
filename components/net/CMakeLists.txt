idf_component_register(SRCS "net.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_wifi esp_netif esp_event esp_timer lwip
                                     config events imu motion obd power proto)
