#ifndef WIFI_WS_H
#define WIFI_WS_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

typedef enum {
    CMD_CONTROL = 0,  // Standard control packet sent 50 times/second
    CMD_READ_PID = 1, // Read all PID values
    CMD_WRITE_PID = 2, // Write all PID values
    CMD_READ_BATTERY = 3, // Read battery status
    CMD_READ_TRIM = 4, // Read trim values
    CMD_WRITE_TRIM = 5, // Write trim values
    CMD_TARE_GYRO = 6, // Tare gyro sensor
} packet_command_t;

/**
 * Control packet data: throttle, roll, pitch, yaw (11-bit values each)
 */
typedef struct {
    uint16_t throttle; // 0 to 2047
    uint16_t roll;     // 1024 is "Center"
    uint16_t pitch;    // 1024 is "Center"
    uint16_t yaw;      // 1024 is "Center"
} ControlPacket;

/**
 * Called when a valid control packet has been received over the websocket.
 */
typedef void (*control_packet_callback_t)(ControlPacket pkt);

/**
 * Called when a valid config packet has been received over the websocket.
 * command: the 4-bit command value
 * data: pointer to the packet data (including the command byte)
 * len: length of the packet in bytes
 * req: the HTTP request handle for sending responses
 */
typedef void (*config_packet_callback_t)(uint8_t command, const uint8_t *data, size_t len, httpd_req_t *req);

esp_err_t wifi_init_ap(void);
void websocket_server_start();

/**
 * Register a callback that will be invoked whenever a correctly‑sized
 * control packet arrives on the websocket.  Passing NULL disables the
 * callback.
 */
void register_control_callback(control_packet_callback_t cb);

/**
 * Register a callback that will be invoked whenever a config packet
 * arrives on the websocket.  Passing NULL disables the callback.
 */
void register_config_callback(config_packet_callback_t cb);

/**
 * Send a binary websocket response frame
 * data: the data to send
 * len: length of data in bytes
 * Returns ESP_OK on success
 */
esp_err_t websocket_send_response(const uint8_t *data, size_t len);

#endif