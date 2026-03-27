#include <string.h>

#include "wifi_ws.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#include "nvs_utils.h"
#include "esp_log.h"
#include "esp_err.h"

#include <string.h>
#include <stdlib.h>

static const char *TAG = "wifi_ws";

/* global callback pointers, guarded by the websocket task */
static control_packet_callback_t g_control_cb = NULL;
static config_packet_callback_t g_config_cb = NULL;

/* store the current websocket request handle for sending responses */
static httpd_req_t *g_current_ws_req = NULL;

/* flag to track if a client is connected */
static bool client_connected = false;

void register_control_callback(control_packet_callback_t cb)
{
    g_control_cb = cb;
}

void register_config_callback(config_packet_callback_t cb)
{
    g_config_cb = cb;
}

bool is_client_connected(void)
{
    return client_connected;
}

/*
 * Get expected packet length for a given command
 * Returns 0 if command is invalid
 */
static size_t get_expected_packet_length(uint8_t command)
{
    switch (command) {
        case CMD_CONTROL:
            return 6;  // Fixed 6 bytes for control packets
        case CMD_READ_PID:
            return 1;  // Just command byte
        case CMD_WRITE_PID:
            return 1 + 9 * sizeof(float);  // Command + 9 PID floats (P,I,D for roll/pitch/yaw)
        case CMD_READ_BATTERY:
            return 1;  // Just command byte
        case CMD_READ_TRIM:
            return 1;  // Just command byte
        case CMD_WRITE_TRIM:
            return 1 + 4 * sizeof(int);  // Command + 4 trim int (FL, FR, BL, BR)
        case CMD_TARE_GYRO:
            return 1;  // Just command byte
        default:
            return 0;  // Invalid command
    }
}

esp_err_t websocket_send_response(const uint8_t *data, size_t len)
{
    if (!g_current_ws_req) {
        ESP_LOGE(TAG, "No active websocket connection for response");
        return ESP_FAIL;
    }

    httpd_ws_frame_t frame = {
        .payload = (uint8_t*)data,
        .len = len,
        .type = HTTPD_WS_TYPE_BINARY
    };

    esp_err_t ret = httpd_ws_send_frame(g_current_ws_req, &frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send websocket response: %s", esp_err_to_name(ret));
    }
    return ret;
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
    /*
    err2 = esp_netif_create_default_wifi_ap();
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "netif create failed: %s", esp_err_to_name(err2));
        return err2;
    }
    */

    if (esp_netif_create_default_wifi_ap() == NULL) {
    ESP_LOGE(TAG, "Failed to create default WiFi AP netif");
    return ESP_FAIL;
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


/*
 * Decode a six‑byte control packet (command 0) that was bit-packed on the remote end
 * using the same shifts used in C# code. The first 4 bits are the command (0 for control).
 * The implementation performs explicit unpacking using shifts to guarantee agreement
 * with the sender.
 */
static ControlPacket handleControlPacket(const uint8_t *data, size_t len)
{
    ControlPacket pkt = {0};
    if (len < 6) {
        return pkt;
    }

    /* Manual decoding to match the bit‑shifts used by the C# sender */
    uint64_t packed = 0;
    for (int i = 0; i < 6; i++) {
        packed |= ((uint64_t)data[i] << (i * 8));
    }

    // Command is already checked to be 0
    pkt.throttle = (packed >> 4)  & 0x7FF;
    pkt.roll     = (packed >> 15) & 0x7FF;
    pkt.pitch    = (packed >> 26) & 0x7FF;
    pkt.yaw      = (packed >> 37) & 0x7FF;
    return pkt;
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket handshake request (method=GET) from client");
        g_current_ws_req = req;  // store for sending responses
        client_connected = true;
        return ESP_OK;
    }

    // If we ever see another method, still allow handling for frames.
    ESP_LOGI(TAG, "ws_handler frame from method=%d", req->method);
    g_current_ws_req = req;

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    /* First call just fetches the header (length and type). */
    esp_err_t hr = httpd_ws_recv_frame(req, &frame, 0);
    if (hr != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame(header) failed: %s", esp_err_to_name(hr));
        return hr;
    }

    /* Guard against empty payloads (ping/pong) */
    if (frame.len == 0) {
        ESP_LOGI(TAG, "WebSocket frame header len=0 type=%d; ignoring", frame.type);
        if (frame.type == HTTPD_WS_TYPE_CLOSE) {
            client_connected = false;
            ESP_LOGI(TAG, "WebSocket closed by peer");
        }
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

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        client_connected = false;
        printf("\n\n WebSocket disconnected \n \n\n");
        free(buf);
        return ESP_OK;
    }

    if (frame.type == HTTPD_WS_TYPE_BINARY) {
        uint8_t command = buf[0] & 0xF;  // First 4 bits
        size_t expected_len = get_expected_packet_length(command);

        if (expected_len == 0) {
            ESP_LOGW(TAG, "Unknown command %d in binary packet", command);
        } else if (frame.len != expected_len) {
            ESP_LOGW(TAG, "Invalid packet length for command %d: got %d, expected %d",
                     command, frame.len, expected_len);
        } else if (command == CMD_CONTROL) {
            ControlPacket pkt = handleControlPacket(buf, frame.len);
            ESP_LOGI(TAG, "Control packet - thr=%u roll=%u pitch=%u yaw=%u",
                     pkt.throttle, pkt.roll, pkt.pitch, pkt.yaw);
            if (g_control_cb) {
                g_control_cb(pkt);
            }
        } else {
            // Valid config command with correct length
            ESP_LOGI(TAG, "Config packet cmd=%u len=%d", command, frame.len);
            if (g_config_cb) {
                g_config_cb(command, buf, frame.len, req);
            }
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
    //config.ws_ping_pong_timeout_sec = 5;

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
