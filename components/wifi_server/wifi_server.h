#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include "esp_err.h"
#include "esp_http_server.h"

// Function to initialize the Wi-Fi server
esp_err_t wifi_server_init(const char* ssid, const char* password);
// Function to start the Wi-Fi task
void wifi_task(void *pvParameters);
// Function to start the HTTP server for streaming
void startWiFiServer();

// Declare the HTTP server handle
extern httpd_handle_t server;

#endif // WIFI_SERVER_H
