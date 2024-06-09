#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include "esp_err.h"

void startWiFiServer();
esp_err_t wifi_server_init(const char* ssid, const char* password);

#endif // WIFI_SERVER_H
