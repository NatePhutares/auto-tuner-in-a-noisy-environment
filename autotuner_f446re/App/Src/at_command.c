/*
 * at_command.c — see at_command.h.
 */
#include "at_command.h"
#include <ctype.h>
#include <string.h>

#define MAX_WORDS 4          /* one more than the longest command, so "set mode fixed x" is rejected */
#define WORD_LEN  16

/* Splits a line into words (as typed). Returns the word count, or MAX_WORDS if there are too many. */
static int split(const char *line, char w[MAX_WORDS][WORD_LEN])
{
    int n = 0;
    while (*line) {
        while (*line == ' ' || *line == '\t' || *line == '\r' || *line == '\n') line++;
        if (!*line) break;
        if (n == MAX_WORDS) return MAX_WORDS;
        int i = 0;
        while (*line && *line != ' ' && *line != '\t' && *line != '\r' && *line != '\n') {
            if (i < WORD_LEN - 1) w[n][i++] = *line;
            line++;
        }
        w[n][i] = '\0';
        n++;
    }
    return n;
}

static int equals_ci(const char *a, const char *b)       /* case-insensitive compare */
{
    while (*a && *b) {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) return 0;
        a++;
        b++;
    }
    return *a == *b;
}

at_cmd_t at_command_parse(const char *line)
{
    char w[MAX_WORDS][WORD_LEN];
    at_cmd_t cmd = {AT_CMD_INVALID, 0};
    int n = split(line, w);
    if (n == 0) { cmd.type = AT_CMD_NONE; return cmd; }

    /* set mode nearest | set mode fixed */
    if (n == 3 && equals_ci(w[0], "set") && equals_ci(w[1], "mode")) {
        if (equals_ci(w[2], "nearest"))    { cmd.type = AT_CMD_MODE; cmd.value = AT_MODE_NEAREST; }
        else if (equals_ci(w[2], "fixed")) { cmd.type = AT_CMD_MODE; cmd.value = AT_MODE_FIXED; }
        return cmd;
    }

    /* target <note> */
    if (n == 2 && equals_ci(w[0], "target")) {
        w[1][0] = (char)toupper((unsigned char)w[1][0]);  /* "c5" -> "C5"; keeps the 'b' of "Bb2" */
        int midi = at_parse_note(w[1]);
        if (midi >= AT_TARGET_MIN && midi <= AT_TARGET_MAX) { cmd.type = AT_CMD_TARGET; cmd.value = midi; }
        return cmd;
    }
    return cmd;
}

int at_command_apply(const at_cmd_t *cmd, autotuner_t *tuner)
{
    switch (cmd->type) {
    case AT_CMD_MODE:   return autotuner_set_mode(tuner, cmd->value);
    case AT_CMD_TARGET: return autotuner_set_target(tuner, cmd->value);
    default:            return -1;
    }
}
