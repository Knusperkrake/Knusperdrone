#ifndef WIFI_WS_H
#define WIFI_WS_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/**
 * The on‑wire packet consists of six consecutive bytes produced by the
 * C# sender using manual bit shifts.  Although the struct below is defined
 * with packed bitfields, relying on memcpy to decode the frame can fail
 * because bitfield ordering and endianness vary between compilers and
 * architectures.  The implementation therefore performs an explicit
 * unpacking using shifts (see wifi_ws.c) to guarantee agreement with the
 * sender.
 *
 * All fields are 11‑bit unsigned values; the final 4 bits of the 48‑bit
 * word are unused.
 */
typedef struct __attribute__((packed)) {
    uint16_t throttle : 11; // 0 to 2047
    uint16_t roll     : 11; // 1024 is "Center"
    uint16_t pitch    : 11; // 1024 is "Center"
    uint16_t yaw      : 11; // 1024 is "Center"
    uint16_t unused   : 4;  // Padding to hit exactly 6 bytes
} CompactDronePacket;

/**
 * Called when a valid control packet has been received over the websocket.
 * The packet is already decoded into the bitfields above.
 */
typedef void (*control_packet_callback_t)(CompactDronePacket pkt);

esp_err_t wifi_init_ap(void);
void websocket_server_start();

/**
 * Register a callback that will be invoked whenever a correctly‑sized
 * binary frame arrives on the websocket.  Passing NULL disables the
 * callback.
 */
void register_control_callback(control_packet_callback_t cb);

#endif