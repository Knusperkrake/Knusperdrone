#ifndef WIFI_WS_H
#define WIFI_WS_H

#include <stdint.h>
#include <stddef.h>

/**
 * This bit-packed structure mirrors the 6‑byte control packet sent by the
 * remote device.  All fields are 11‑bit unsigned values and the whole
 * structure is marked packed to prevent the compiler from inserting any
 * padding.  On the wire the packet is transmitted as 6 raw bytes.
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

void wifi_init_ap();
void websocket_server_start();

/**
 * Register a callback that will be invoked whenever a correctly‑sized
 * binary frame arrives on the websocket.  Passing NULL disables the
 * callback.
 */
void register_control_callback(control_packet_callback_t cb);

#endif