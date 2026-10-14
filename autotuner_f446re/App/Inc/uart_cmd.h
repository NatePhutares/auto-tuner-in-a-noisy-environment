/*
 * uart_cmd.h — receives text command lines from the laptop on USART2 RX (ST-LINK VCP).
 *
 * The RX interrupt only stores bytes in a ring buffer (one byte per HAL_UART_Receive_IT).
 * The main loop calls uart_cmd_get_line() once per hop, so commands take effect at a hop boundary
 * and the DSP state is never touched from an interrupt.
 */
#ifndef UART_CMD_H
#define UART_CMD_H

#include "main.h"
#include <stdint.h>

#define UART_CMD_LINE_MAX 48

extern volatile uint32_t uart_cmd_bytes;      /* bytes received                       */
extern volatile uint32_t uart_cmd_overflows;  /* bytes lost because the ring was full */
extern volatile uint32_t uart_cmd_errors;     /* UART errors (overrun, framing)       */

void uart_cmd_start(UART_HandleTypeDef *huart);

/* Copies the next complete line (without \r\n) into line. Returns 1 if a line was available. */
int uart_cmd_get_line(char *line, int size);

#endif /* UART_CMD_H */
