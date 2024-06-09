idf_component_register(SRCS "wifi_server.cpp"
                    REQUIRES esp_wifi
                    REQUIRES nvs_flash
                    REQUIRES esp_http_server
                    REQUIRES camera
                    REQUIRES esp32-camera
                    INCLUDE_DIRS ../controller
                    INCLUDE_DIRS ".")