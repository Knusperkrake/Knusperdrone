idf_component_register(SRCS "main.c" "mpu6050.c" "motor.c" "nvs_utils.c" "wifi.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver freertos nvs_flash esp_wifi esp_netif esp_event esp_http_server
                       )