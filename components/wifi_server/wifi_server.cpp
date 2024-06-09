#include "wifi_server.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "controller_module.h"
#include "WiFi.h"
#include "nvs_flash.h"
#include "Arduino.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "ai_camera.h"

// HTTP server handle
httpd_handle_t server = NULL;

// HTTP server configuration
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// TCP server port for receiving joystick data
#define TCP_PORT 3333

// Wi-Fi task function
void wifi_task(void *pvParameters) {
    Serial.println("Starting Wi-Fi task...");

    // Initialize Wi-Fi server
    if (wifi_server_init("acer1664", "sdys3.14") != ESP_OK) {
        Serial.println("Wi-Fi server init failed");
        vTaskDelete(NULL);  // Delete this task if initialization fails
    }

    while (true) {
        delay(10000);
    }
}

// HTTP handler for streaming JPEG frames
esp_err_t jpg_stream_httpd_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char part_buf[64];
    static int64_t last_frame = 0;
    if (!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
            break;
        }
        if (fb->format != PIXFORMAT_JPEG) {
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            if (!jpeg_converted) {
                Serial.println("JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
            }
        } else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK) {
            size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if (fb->format != PIXFORMAT_JPEG) {
            free(_jpg_buf);
        }
        esp_camera_fb_return(fb);
        if (res != ESP_OK) {
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        Serial.printf("MJPG: %luKB %lums (%.1ffps)\n", (uint32_t)(_jpg_buf_len / 1024), (uint32_t)frame_time, 1000.0 / (float)frame_time);
    }

    last_frame = 0;
    return res;
}

// TCP task function to receive joystick data
void tcp_task(void *pvParameters) {
    Serial.println("Starting TCP task");

    // Create a TCP socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        Serial.println("Failed to create socket");
        vTaskDelete(NULL);
    }

    // Bind the socket to any IP address and TCP_PORT
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TCP_PORT);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        Serial.println("Failed to bind socket");
        close(sock);
        vTaskDelete(NULL);
    }

    if (listen(sock, 1) < 0) {
        Serial.println("Failed to listen on socket");
        close(sock);
        vTaskDelete(NULL);
    }

    Serial.println("Waiting for a connection...");
    int client_sock = accept(sock, NULL, NULL);
    if (client_sock < 0) {
        Serial.println("Failed to accept connection");
        close(sock);
        vTaskDelete(NULL);
    }

    char buffer[128];
    while (true) {
        int len = recv(client_sock, buffer, sizeof(buffer) - 1, 0);
        if (len < 0) {
            Serial.println("Failed to receive data");
            break;
        }
        buffer[len] = '\0';
        float x1, y1, x2, y2;
        sscanf(buffer, "%f %f %f %f", &x1, &y1, &x2, &y2);

        processJoystickData(x1, y1, x2, y2);
    }

    close(client_sock);
    close(sock);
    vTaskDelete(NULL);
}

// Function to start the HTTP server for streaming
void startWiFiServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_open_sockets = 5;
    config.max_uri_handlers = 8;
    config.max_resp_headers = 8;
    config.backlog_conn = 5;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    httpd_uri_t uri_handler = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = jpg_stream_httpd_handler,
        .user_ctx = NULL
    };

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_handler);
    }
}

// Function to initialize the Wi-Fi server
esp_err_t wifi_server_init(const char* ssid, const char* password) {
    Serial.println("Starting Wi-Fi server...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize the camera
    if (ai_camera_init() != ESP_OK) {
        Serial.println("Camera init failed");
        return ESP_FAIL;
    }

    // Connect to Wi-Fi
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Start the HTTP server
    startWiFiServer();
    Serial.println("Wi-Fi server started.");

    // Start TCP task to receive joystick data
    xTaskCreatePinnedToCore(tcp_task, "tcpTask", 4096, NULL, 1, NULL, 0);

    return ESP_OK;
}
