/*
 * uart_packet.h — one packet per 8 ms hop, MCU -> laptop over USART2 (ST-LINK VCP)
 *
 * Packet layout (little-endian, 266 bytes):
 *   offset  size  field
 *   0       2     sync        0xA5 0x5A
 *   2       2     seq         uint16, +1 per packet (gaps = dropped packets)
 *   4       4     pitch_hz    float32, detected pitch (0 = unvoiced / not computed yet)
 *   8       1     target      uint8 MIDI note (0 = none)
 *   9       1     status      bit0 out_of_range, bit1 mode (0 nearest, 1 fixed), bit2 voiced
 *   10      256   samples     128 x int16 PCM, 16 kHz mono
 *
 * Wire load: 266 B x 10 bits / 8 ms = 332.5 kbaud of the 921600 link (36 %).
 */
#ifndef UART_PACKET_H
#define UART_PACKET_H

#include "main.h"
#include <stdint.h>

#define PKT_SAMPLES   128
#define PKT_HDR_SIZE  10
#define PKT_SIZE      (PKT_HDR_SIZE + 2 * PKT_SAMPLES)   /* 266 */
#define PKT_SYNC0     0xA5
#define PKT_SYNC1     0x5A

#define PKT_STATUS_OUT_OF_RANGE  (1u << 0)
#define PKT_STATUS_MODE_FIXED    (1u << 1)
#define PKT_STATUS_VOICED        (1u << 2)

extern volatile uint32_t pkt_sent;     /* packets handed to the UART DMA */
extern volatile uint32_t pkt_dropped;  /* packets skipped because the UART was still busy */

void uart_packet_init(UART_HandleTypeDef *huart);

/* Builds a packet from 128 int16 samples and starts a DMA transfer.
 * Never blocks. Returns 0 if sent, -1 if the previous packet was still
 * being transmitted (the packet is then dropped and counted). */
int uart_packet_send(const int16_t *samples, float pitch_hz,
                     uint8_t target_note, uint8_t status);

#endif /* UART_PACKET_H */
