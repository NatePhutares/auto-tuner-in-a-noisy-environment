/*
 * uart_packet.c — see uart_packet.h for the packet format.
 *
 * Two packet buffers (ping-pong): the next packet is built in one buffer
 * while the DMA may still be reading the other, so a buffer is never
 * overwritten mid-transfer.
 */
#include "uart_packet.h"
#include <string.h>

volatile uint32_t pkt_sent    = 0;
volatile uint32_t pkt_dropped = 0;

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t  s_buf[2][PKT_SIZE];
static uint8_t  s_idx = 0;
static uint16_t s_seq = 0;

void uart_packet_init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  s_idx   = 0;
  s_seq   = 0;
}

int uart_packet_send(const int16_t *samples, float pitch_hz,
                     uint8_t target_note, uint8_t status)
{
  /* Previous transfer still running: drop this packet rather than block the hop. */
  if (s_huart == NULL || s_huart->gState != HAL_UART_STATE_READY) {
    pkt_dropped++;
    s_seq++;                       /* keep counting so the laptop sees the gap */
    return -1;
  }

  uint8_t *b = s_buf[s_idx];
  b[0] = PKT_SYNC0;
  b[1] = PKT_SYNC1;
  b[2] = (uint8_t)(s_seq & 0xFF);
  b[3] = (uint8_t)(s_seq >> 8);
  memcpy(&b[4], &pitch_hz, 4);     /* Cortex-M4 is little-endian, same as the laptop */
  b[8] = target_note;
  b[9] = status;
  memcpy(&b[PKT_HDR_SIZE], samples, 2 * PKT_SAMPLES);

  if (HAL_UART_Transmit_DMA(s_huart, b, PKT_SIZE) != HAL_OK) {
    pkt_dropped++;
    s_seq++;
    return -1;
  }

  pkt_sent++;
  s_seq++;
  s_idx ^= 1u;
  return 0;
}
