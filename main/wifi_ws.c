#include <string.h>

#include "wifi_ws.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#include "nvs_utils.h"
#include "esp_log.h"

#include <string.h>
#include <stdlib.h>

static const char *TAG = "wifi_ws";

/* global callback pointer, guarded by the websocket task */
static control_packet_callback_t g_control_cb = NULL;

void register_control_callback(control_packet_callback_t cb)
{
    g_control_cb = cb;
}

esp_err_t wifi_init_ap(void) {
    /* initialise NVS early; ignore if already done */
    esp_err_t err = nvs_initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialisation failed: %s", esp_err_to_name(err));
        /* continue - we may still be able to start wifi */
    } else {
        /* example: remember last SSID so we can compare on reboot */
        char stored_ssid[32];
        if (nvs_load_string("wifi", "ssid", stored_ssid, sizeof(stored_ssid)) == ESP_OK) {
            ESP_LOGI(TAG, "previous SSID was '%s'", stored_ssid);
        }
        /* save current value (could also be loaded from config) */
        nvs_save_string("wifi", "ssid", "DroneAP");
    }

    esp_err_t err2;

    err2 = esp_netif_init();
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err2));
        return err2;
    }
    err2 = esp_event_loop_create_default();
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "event loop create failed: %s", esp_err_to_name(err2));
        return err2;
    }
    err2 = esp_netif_create_default_wifi_ap();
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "netif create failed: %s", esp_err_to_name(err2));
        return err2;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err2 = esp_wifi_init(&cfg);
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err2));
        return err2;
    }

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "DroneAP",
            .ssid_len = strlen("DroneAP"),
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .password = "knusper123"
        },
    };
    err2 = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(err2));
        return err2;
    }
    err2 = esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(err2));
        return err2;
    }
    err2 = esp_wifi_start();
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err2));
        return err2;
    }

    ESP_LOGI(TAG, "WiFi AP setup complete (SSID %s)", wifi_config.ap.ssid);
    return ESP_OK;
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        printf("WebSocket connected\n");
        return ESP_OK;
    }

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    /* First call just fetches the header (length and type). */
    httpd_ws_recv_frame(req, &frame, 0);

    /* Guard against bogus lengths */
    if (frame.len == 0) {
        return ESP_OK;
    }

    uint8_t *buf = malloc(frame.len + 1);
    if (!buf) {
        ESP_LOGE(TAG, "malloc failed for %d bytes", frame.len + 1);
        return ESP_ERR_NO_MEM;
    }

    frame.payload = buf;
    esp_err_t r = httpd_ws_recv_frame(req, &frame, frame.len);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed: %s", esp_err_to_name(r));
        free(buf);
        return r;
    }

    /* always null‑terminate so we can print as a string if needed */
    buf[frame.len] = 0;

    if (frame.type == HTTPD_WS_TYPE_BINARY && frame.len == sizeof(CompactDronePacket)) {
        CompactDronePacket pkt;
        memcpy(&pkt, buf, sizeof(pkt));

        ESP_LOGI(TAG, "Control packet - thr=%u roll=%u pitch=%u yaw=%u", pkt.throttle, pkt.roll, pkt.pitch, pkt.yaw);

        if (g_control_cb) {
            g_control_cb(pkt);
        }
    } else if (frame.type == HTTPD_WS_TYPE_TEXT) {
        ESP_LOGI(TAG, "Received text: %s", buf);
    } else {
        ESP_LOGW(TAG, "ignored frame type %d len %d", frame.type, frame.len);
    }

    free(buf);
    return ESP_OK;
}

void websocket_server_start() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.ws_ping_pong_timeout_sec = 5;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t ws_uri = {
            .uri       = "/ws",
            .method    = HTTP_GET,
            .handler   = ws_handler,
            .user_ctx  = NULL,
            .is_websocket = true
        };
        httpd_register_uri_handler(server, &ws_uri);
    }
}
