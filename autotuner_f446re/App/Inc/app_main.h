/*
 * app_main.h — glue between the hardware (main.c) and the auto-tuner.
 *
 * main.c keeps the CubeMX code and the INMP441 conversion it already had (24-bit slot -> int16 with
 * MIC_SHIFT / MIC_GAIN_Q8). Per 8 ms hop it hands the int16 hop to app_process_hop(), which
 *   1. applies any command line received from the laptop,
 *   2. runs autotuner_process_hop() (int16 / 32768 -> float, the same scaling as the recorded WAVs),
 *   3. sends the tuned hop with pitch / target / status over UART,
 *   4. counts the out-of-range share (for Live Expressions) and redraws the OLED: mode, target and
 *      the note being sung (5 times a second, and right after a command so the screen confirms it).
 *
 * Everything worth watching is volatile so it can be added to CubeIDE "Live Expressions".
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "main.h"
#include <stdint.h>
#include "autotuner.h"

#define APP_CPU_HZ      180000000u
#define APP_HOP_CYCLES  (APP_CPU_HZ / AT_FS * AT_H)     /* 1 440 000 cycles = 8 ms deadline */

extern volatile uint32_t app_cycles_last;   /* DWT cycles of the last autotuner_process_hop() */
extern volatile uint32_t app_cycles_max;    /* worst case since reset (write 0 to restart)    */
extern volatile uint32_t app_deadline_miss; /* hops that took longer than APP_HOP_CYCLES      */
extern volatile uint32_t app_cmd_ok, app_cmd_rejected;   /* command lines accepted / rejected */
extern volatile uint32_t app_oor_voiced;    /* fixed mode: voiced hops since the last mode/target change */
extern volatile uint32_t app_oor_count;     /* ... of which out of range (oor % = count / voiced)        */
extern at_info_t         app_info;          /* last hop's pitch / target / status             */

void app_init(UART_HandleTypeDef *huart, I2C_HandleTypeDef *hi2c);

/* pcm: AT_H int16 mic samples of the hop that just arrived. */
void app_process_hop(const int16_t *pcm);

#endif /* APP_MAIN_H */
