/*
 * app_main.c — see app_main.h.
 */
#include "app_main.h"
#include "at_command.h"
#include "uart_cmd.h"
#include "uart_packet.h"
#include "oled.h"
#include "app_screen.h"
#include <math.h>

#define SCREEN_EVERY_HOPS  25       /* 25 x 8 ms = 200 ms: 5 screen updates per second */
#define SUNG_HOLD_HOPS     25       /* keep the last sung note 200 ms after voicing stops */
#define SUNG_CONF_MIN      0.6f     /* pitch confidence needed for the OLED (voicing itself uses 0.5) */
#define SUNG_STABLE_HOPS   5        /* same note for 5 hops (40 ms) before it is shown                */

volatile uint32_t app_cycles_last = 0;
volatile uint32_t app_cycles_max = 0;
volatile uint32_t app_deadline_miss = 0;
volatile uint32_t app_cmd_ok = 0, app_cmd_rejected = 0;
volatile uint32_t app_oor_voiced = 0, app_oor_count = 0;
at_info_t         app_info;

static autotuner_t s_tuner;          /* ~80 kB: static, never on the 4 kB stack */
static float   s_in[AT_H], s_out[AT_H];
static int16_t s_out_pcm[AT_H];
static oled_fb_t s_screen;
static uint32_t s_hops = 0;
static int      s_screen_dirty = 1;  /* redraw at the next hop (set after every command) */
static int      s_last_mode = -1, s_last_target = -1;
static int      s_sung_midi = -1;    /* note shown on the OLED, -1 = none */
static int      s_sung_hold = 0;     /* hops left before the held note is cleared */
static int      s_cand_midi = -1;    /* note waiting to be shown ...                  */
static int      s_cand_run = 0;      /* ... and for how many hops in a row             */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void app_init(UART_HandleTypeDef *huart, I2C_HandleTypeDef *hi2c)
{
    at_config_t cfg;
    at_config_default(&cfg);         /* change fields here to override the Python defaults */
    autotuner_init(&s_tuner, &cfg);
    uart_packet_init(huart);
    uart_cmd_start(huart);
    (void)oled_init(hi2c);           /* no display connected: everything else still runs */
    dwt_init();
}

static void handle_commands(void)
{
    char line[UART_CMD_LINE_MAX];
    while (uart_cmd_get_line(line, sizeof(line))) {
        at_cmd_t cmd = at_command_parse(line);
        if (cmd.type == AT_CMD_NONE) continue;
        if (at_command_apply(&cmd, &s_tuner) == 0) app_cmd_ok++;
        else                                       app_cmd_rejected++;
        s_screen_dirty = 1;
    }
}

/* Out-of-range share in fixed mode, restarted whenever the mode or the fixed target changes.
 * Not shown on the OLED any more; kept for CubeIDE Live Expressions (app_oor_count / app_oor_voiced). */
static void update_oor_stats(const at_info_t *info)
{
    int target = info->mode == AT_MODE_FIXED ? info->target : -1;
    if (info->mode != s_last_mode || target != s_last_target) {
        app_oor_voiced = 0;
        app_oor_count = 0;
        s_last_mode = info->mode;
        s_last_target = target;
    }
    if (info->mode == AT_MODE_FIXED && info->voiced) {
        app_oor_voiced++;
        if (info->out_of_range) app_oor_count++;
    }
}

/* Sung note for the OLED. The tuner's voicing test (confidence > 0.5, level > -40 dBFS) also passes on
 * room noise, whose "pitch" jumps to a new note every few hops, so the screen is stricter:
 *   - confidence >= SUNG_CONF_MIN, and
 *   - the same note (median-smoothed pitch, rounded) for SUNG_STABLE_HOPS hops in a row.
 * Noise's "pitch" rarely stays on one note for 40 ms; a real voice does.
 * The shown note is held for SUNG_HOLD_HOPS after that stops, so consonants and breaths don't blank
 * it; then the screen shows '-'. Tuned on the recordings in python_prototype/wav_files: a note is
 * shown for 92-98 % of the voiced hops of humming / singing, while noise alone shows one 0-6 % of the
 * time (6 % = loud white noise at -20 dBFS). Stricter (0.8 / 10 hops) blanked too much real singing;
 * looser (4 hops) lets white noise through 15 % of the time. Other people talking can show a note.
 * Called every hop. */
static void update_sung_note(const at_info_t *info)
{
    int ok = info->voiced && info->has_midi && info->confidence >= SUNG_CONF_MIN;
    if (ok) {
        int m = (int)lrintf(info->smoothed_midi);
        if (m < 0 || m > 127) {
            ok = 0;
        } else {
            if (m == s_cand_midi) { if (s_cand_run < SUNG_STABLE_HOPS) s_cand_run++; }
            else                  { s_cand_midi = m; s_cand_run = 1; }
            if (s_cand_run >= SUNG_STABLE_HOPS) {
                s_sung_midi = m;
                s_sung_hold = SUNG_HOLD_HOPS;
                return;
            }
        }
    }
    if (!ok) { s_cand_midi = -1; s_cand_run = 0; }
    if (s_sung_hold > 0) s_sung_hold--;
    else                 s_sung_midi = -1;
}

static void update_screen(const at_info_t *info)
{
    int fixed = info->mode == AT_MODE_FIXED;
    app_screen_draw(&s_screen, info->mode, fixed ? info->target : -1, s_sung_midi);
    if (oled_flush(&s_screen) == 0) s_screen_dirty = 0;   /* busy: try again next hop */
}

static void float_to_pcm(const float *x, int16_t *out)
{
    for (int i = 0; i < AT_H; i++) {
        float v = x[i] * 32768.0f;
        if (v > 32767.0f) v = 32767.0f;
        if (v < -32768.0f) v = -32768.0f;
        out[i] = (int16_t)lrintf(v);
    }
}

void app_process_hop(const int16_t *pcm)
{
    oled_poll();                                        /* next OLED page, if the bus is free */
    handle_commands();                                  /* controls change at hop boundaries   */

    for (int i = 0; i < AT_H; i++) s_in[i] = (float)pcm[i] * (1.0f / 32768.0f);

    uint32_t t0 = DWT->CYCCNT;
    autotuner_process_hop(&s_tuner, s_in, s_out, &app_info);
    uint32_t dt = DWT->CYCCNT - t0;
    app_cycles_last = dt;
    if (dt > app_cycles_max) app_cycles_max = dt;
    if (dt > APP_HOP_CYCLES) app_deadline_miss++;

    float_to_pcm(s_out, s_out_pcm);
    uint8_t target = app_info.target >= 0 ? (uint8_t)app_info.target : 0u;
    uart_packet_send(s_out_pcm, app_info.f0_hz, target, app_info.status);

    update_oor_stats(&app_info);
    update_sung_note(&app_info);
    if (++s_hops % SCREEN_EVERY_HOPS == 0) s_screen_dirty = 1;
    if (s_screen_dirty) update_screen(&app_info);
}
