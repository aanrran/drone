#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Function to initialize the Wi-Fi server
esp_err_t wifi_server_init(const char* ssid, const char* password);

// Task function to start Wi-Fi
void wifi_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // WIFI_SERVER_H
