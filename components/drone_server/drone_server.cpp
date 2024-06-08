#include "drone_server.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_http_server.h>
#include <camera.h>
#include "controller.h"
#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "server";

extern QueueHandle_t command_queue;

static esp_err_t move_up_handler(httpd_req_t *req) {
    Command cmd = MOVE_UP;
    xQueueSend(command_queue, &cmd, portMAX_DELAY);
    httpd_resp_send(req, "Move Up", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static uint8_t *fb = NULL;
static size_t fb_len = 0;

static void camera_capture_task(void *pvParameters) {
    while (true) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (!frame) {
            ESP_LOGE(TAG, "Failed to capture frame");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        fb = frame->buf;
        fb_len = frame->len;
        esp_camera_fb_return(frame);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static esp_err_t stream_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_send(req, (const char *)fb, fb_len);
    return ESP_OK;
}

static httpd_uri_t move_up = {
    .uri       = "/move_up",
    .method    = HTTP_GET,
    .handler   = move_up_handler,
    .user_ctx  = NULL
};

static httpd_uri_t stream = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
};

void setup_wifi_server(void *pvParameters) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_Drone",
            .password = "your_password",
            .ssid_len = strlen("ESP32_Drone"),
            .channel = 1,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .ssid_hidden = 0,
            .max_connection = 4,
            .beacon_interval = 100,
            .pairwise_cipher = WIFI_CIPHER_TYPE_TKIP_CCMP,
            .ftm_responder = 0,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
            .sae_pwe_h2e = WPA3_SAE_PWE_UNSPECIFIED
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config((wifi_interface_t)ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(httpd_start(&server, &config));

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &move_up));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &stream));

    xTaskCreatePinnedToCore(camera_capture_task, "camera_capture_task", 8192, NULL, 10, NULL, 1);

    vTaskDelete(NULL);
}
