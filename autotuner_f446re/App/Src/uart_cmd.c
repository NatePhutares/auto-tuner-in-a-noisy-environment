/*
 * uart_cmd.c — see uart_cmd.h. Uses the USART2 global interrupt (already enabled in CubeMX).
 *
 * It defines HAL_UART_RxCpltCallback and HAL_UART_ErrorCallback. If another file needs those
 * callbacks later, call uart_cmd_rx_byte()/uart_cmd_rx_error() logic from there instead.
 */
#include "uart_cmd.h"

#define RING_SIZE 128u                      /* power of two */

volatile uint32_t uart_cmd_bytes = 0;
volatile uint32_t uart_cmd_overflows = 0;
volatile uint32_t uart_cmd_errors = 0;

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_rx_byte;
static volatile uint8_t  s_ring[RING_SIZE];
static volatile uint32_t s_head = 0;        /* written by the ISR       */
static volatile uint32_t s_tail = 0;        /* written by the main loop */

static char s_line[UART_CMD_LINE_MAX];
static int  s_line_len = 0;

static void arm_receive(void)
{
    if (s_huart) (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

void uart_cmd_start(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_head = s_tail = 0;
    s_line_len = 0;
    arm_receive();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) return;
    uart_cmd_bytes++;
    if (s_head - s_tail < RING_SIZE) {
        s_ring[s_head & (RING_SIZE - 1u)] = s_rx_byte;
        s_head++;
    } else {
        uart_cmd_overflows++;
    }
    arm_receive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) return;
    uart_cmd_errors++;
    arm_receive();                           /* HAL aborts the reception on errors: restart it */
}

int uart_cmd_get_line(char *line, int size)
{
    while (s_tail != s_head) {
        char c = (char)s_ring[s_tail & (RING_SIZE - 1u)];
        s_tail++;
        if (c == '\n' || c == '\r') {
            if (s_line_len == 0) continue;   /* skip empty lines / the \n of \r\n */
            int n = s_line_len < size - 1 ? s_line_len : size - 1;
            for (int i = 0; i < n; i++) line[i] = s_line[i];
            line[n] = '\0';
            s_line_len = 0;
            return 1;
        }
        if (s_line_len < UART_CMD_LINE_MAX - 1) s_line[s_line_len++] = c;
    }
    return 0;
}
