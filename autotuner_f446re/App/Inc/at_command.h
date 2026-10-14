/*
 * at_command.h — text commands from the laptop (spec section 5). Pure parsing, no hardware.
 *
 * One command per line, case-insensitive, words separated by spaces, \r\n or \n line ending:
 *     set mode nearest
 *     set mode fixed
 *     target <note>          e.g. target C5, target F#3, target Bb2, target 60   (C1..C8)
 *
 * Anything else is rejected (counted in app_cmd_rejected). The MCU never answers in text: the TX
 * line carries the binary audio packets, so the effect shows up in the status / target bytes of
 * the next packets (rx_audio.py shows them in its status line).
 */
#ifndef AT_COMMAND_H
#define AT_COMMAND_H

#include "autotuner.h"

typedef enum {
    AT_CMD_NONE = 0,     /* empty line                */
    AT_CMD_INVALID,      /* unknown command / bad note */
    AT_CMD_MODE,         /* value = AT_MODE_*         */
    AT_CMD_TARGET        /* value = MIDI note         */
} at_cmd_type_t;

typedef struct {
    at_cmd_type_t type;
    int value;
} at_cmd_t;

/* Parses one line (with or without trailing \r\n). */
at_cmd_t at_command_parse(const char *line);

/* Applies a parsed command to the tuner. Returns 0 if applied, -1 if rejected. */
int at_command_apply(const at_cmd_t *cmd, autotuner_t *tuner);

#endif /* AT_COMMAND_H */
