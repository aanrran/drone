#include "wifi_server.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "WiFi.h"
#include "nvs_flash.h"
#include "Arduino.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "ai_camera.h"
#include "joysticks_console.h"
#include "controller_module.h"

// HTTP server handle
httpd_handle_t server = NULL;

// HTTP server configuration
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// TCP server port for receiving joystick data
#define TCP_PORT 3333
#define RESTART_COMMAND "RESTART"

/**
 * @brief Calculate CRC-8 using the polynomial 0x07
 * 
 * @param data Pointer to the data buffer
 * @param len Length of the data buffer
 * @return uint8_t Calculated CRC-8 value
 */
uint8_t calculate_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Wi-Fi task function
 * Initializes the Wi-Fi server and keeps the task alive.
 * 
 * @param pvParameters Task parameters (not used).
 */
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

/**
 * @brief HTTP handler for streaming JPEG frames
 * Streams the camera feed as a series of JPEG images.
 * 
 * @param req Pointer to the HTTP request.
 * 
 * @return esp_err_t Returns ESP_OK on success, otherwise returns an error code.
 */
esp_err_t IRAM_ATTR  jpg_stream_httpd_handler(httpd_req_t *req) {
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

/**
 * @brief TCP task function to receive joystick data
 * Sets up a TCP server to receive joystick data and processes it.
 * 
 * @param pvParameters Task parameters (not used).
 */
void IRAM_ATTR tcp_task(void *pvParameters) {
    Serial.println("Starting TCP task");

    // Create a TCP socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        printf("Failed to create socket");
        vTaskDelete(NULL);
    }

    // Bind the socket to any IP address and TCP_PORT
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TCP_PORT);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("Failed to bind socket");
        close(sock);
        vTaskDelete(NULL);
    }

    if (listen(sock, 1) < 0) {
        printf("Failed to listen on socket");
        close(sock);
        vTaskDelete(NULL);
    }

    Serial.println("Waiting for a connection...");
    int client_sock = accept(sock, NULL, NULL);
    if (client_sock < 0) {
        printf("Failed to accept connection");
        close(sock);
        vTaskDelete(NULL);
    }

    char buffer[256];  // Buffer to hold received data
    while (true) {
        int len = recv(client_sock, buffer, sizeof(buffer), 0);
        if (len < 0) {
            printf("Failed to receive data");
            break;
        }

        if (len > 0) {
            // Check if the received data is the RESTART command
            if (strncmp(buffer, RESTART_COMMAND, len) == 0) {
                printf("Received RESTART command. Restarting the drone...");
                state_machine_restart();  // Restart the drone
            } 
            else if (len == 5) { // Expecting 4 bytes of joystick data and 1 byte of CRC
                // Parse the joystick data
                uint8_t x1 = buffer[0];
                uint8_t y1 = buffer[1];
                uint8_t x2 = buffer[2];
                uint8_t y2 = buffer[3];
                uint8_t received_crc = buffer[4];

                // Calculate CRC-8
                uint8_t calculated_crc = calculate_crc8((uint8_t*)buffer, 4);

                if (calculated_crc == received_crc) {
                    // Process joystick data
                    processJoystickData(x1, y1, x2, y2);
                } else {
                    printf("CRC mismatch, data corrupted");
                }
            }
            else if (len == 41) { // Expecting 40 bytes of PID parameters and 1 byte of CRC
                float Kp_roll, Ki_roll, Kd_roll, Kp_pitch, Ki_pitch, Kd_pitch, Kp_yaw, Ki_yaw, Kd_yaw, integral_max;
                memcpy(&Kp_roll, &buffer[0], sizeof(float));
                memcpy(&Ki_roll, &buffer[4], sizeof(float));
                memcpy(&Kd_roll, &buffer[8], sizeof(float));
                memcpy(&Kp_pitch, &buffer[12], sizeof(float));
                memcpy(&Ki_pitch, &buffer[16], sizeof(float));
                memcpy(&Kd_pitch, &buffer[20], sizeof(float));
                memcpy(&Kp_yaw, &buffer[24], sizeof(float));
                memcpy(&Ki_yaw, &buffer[28], sizeof(float));
                memcpy(&Kd_yaw, &buffer[32], sizeof(float));
                memcpy(&integral_max, &buffer[36], sizeof(float));
                uint8_t received_crc = buffer[40];

                // Calculate CRC-8
                uint8_t calculated_crc = calculate_crc8((uint8_t*)buffer, 40);

                if (calculated_crc == received_crc) {
                    // Set PID parameters
                    setPIDParameters(Kp_roll, Ki_roll, Kd_roll, Kp_pitch, Ki_pitch, Kd_pitch, Kp_yaw, Ki_yaw, Kd_yaw, integral_max);
                    printf("PID parameters set successfully");
                } else {
                    printf("CRC mismatch, PID data corrupted");
                }
            }
        } else {
            printf("Failed to parse data");
        }
    }

    close(client_sock);
    close(sock);
    vTaskDelete(NULL);
}

/**
 * @brief Function to start the HTTP server for streaming
 * Configures and starts the HTTP server to stream JPEG frames from the camera.
 */
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

/**
 * @brief Function to initialize the Wi-Fi server
 * Initializes the Wi-Fi connection, camera, and starts the HTTP and TCP servers.
 * 
 * @param ssid Wi-Fi SSID.
 * @param password Wi-Fi password.
 * 
 * @return esp_err_t Returns ESP_OK on success, otherwise returns an error code.
 */
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
    xTaskCreatePinnedToCore(tcp_task, "tcpTask", 4096, NULL, 2, NULL, 1);

    return ESP_OK;
}
