/*
 * oled.h — 128x64 I2C OLED driver (SSD1306 or SH1106) on I2C1, non-blocking.
 *
 * Choose the controller below. 1.3" modules are almost always SH1106 (even when sold as "SSD1306"),
 * 0.96" modules are SSD1306:
 *   - wrong choice SH1106 on an SSD1306 panel : screen stays black (the charge pump is never enabled)
 *   - wrong choice SSD1306 on an SH1106 panel : picture shifted 2 px left, noise in the right 2 columns
 *
 * How it stays off the audio path: oled_init() runs once before the audio starts (blocking, ~20 ms).
 * After that, oled_flush() only copies the framebuffer (1 kB) into 8 page packets, and oled_poll()
 * (called once per 8 ms hop) starts the next page by I2C DMA if the bus is free. A full screen
 * therefore takes 8 hops = 64 ms and never blocks the DSP.
 * If no display answers at 0x3C / 0x3D, every call returns immediately and nothing else is affected.
 */
#ifndef OLED_H
#define OLED_H

#include "main.h"
#include <stdint.h>
#include "oled_gfx.h"

#define OLED_SSD1306  0
#define OLED_SH1106   1

#ifndef OLED_CONTROLLER
#define OLED_CONTROLLER  OLED_SH1106     /* 1.3" module; use OLED_SSD1306 for a 0.96" one */
#endif

extern volatile uint8_t  oled_present;   /* 1 = display answered at init                 */
extern volatile uint8_t  oled_address;   /* 7-bit address found (0x3C or 0x3D)            */
extern volatile uint32_t oled_frames;    /* complete screens sent                         */
extern volatile uint32_t oled_errors;    /* I2C errors (NACK, bus error) since reset      */

/* Finds the display, sends the init sequence and clears the screen. Returns 0 if a display answered. */
int  oled_init(I2C_HandleTypeDef *hi2c);

/* 1 while a screen is still being sent (oled_flush would be ignored). */
int  oled_busy(void);

/* Queues fb for sending. Returns 0 if queued, -1 if busy or no display (the frame is skipped). */
int  oled_flush(const oled_fb_t *fb);

/* Call often (once per hop is enough): starts the next page transfer when the bus is free. */
void oled_poll(void);

#endif /* OLED_H */
