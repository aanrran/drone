idf_component_register(SRCS "wifi_server.cpp"
                    REQUIRES esp_wifi
                    REQUIRES nvs_flash
                    REQUIRES esp_http_server
                    REQUIRES ai_camera 
                    REQUIRES controller_module
                    INCLUDE_DIRS ../controller_module
                    INCLUDE_DIRS ../ai_camera
                    INCLUDE_DIRS ".")


