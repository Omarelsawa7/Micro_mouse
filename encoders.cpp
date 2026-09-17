#include "encoders.h"
#include "config.h"

#include <Arduino.h>
#include <string.h>
#include <limits.h>

volatile bool g_cell_watch_fired = false;

static pcnt_unit_handle_t s_unit_l = NULL;
static pcnt_unit_handle_t s_unit_r = NULL;
static pcnt_channel_handle_t s_ch_l = NULL;
static pcnt_channel_handle_t s_ch_r = NULL;

static volatile int32_t s_cell_start_avg = 0;
static volatile int32_t s_cell_target = 0;

/* Watchpoint bookkeeping: target is constant after the overflow fix, so we
 * must remove the previous watchpoint before re-adding to avoid exhausting
 * the PCNT watchpoint slots with duplicates every cell. */
static bool s_watch_armed = false;
static int s_watch_val = 0;

/* PCNT event callback runs in ISR context: keep minimal, set flag only. */
static bool pcnt_watch_cb(pcnt_unit_handle_t unit,
                          const pcnt_watch_event_data_t *edata,
                          void *user_ctx)
{
    (void)unit;
    (void)edata;
    (void)user_ctx;
    g_cell_watch_fired = true;
    return false; /* no high-priority task wake needed */
}

static void make_unit(pcnt_unit_handle_t *out_unit,
                      pcnt_channel_handle_t *out_ch,
                      int gpio_a, int gpio_b)
{
    pcnt_unit_config_t ucfg;
    pcnt_chan_config_t ccfg;
    pcnt_glitch_filter_config_t fcfg;
    pcnt_event_callbacks_t cbs;

    memset(&ucfg, 0, sizeof(ucfg));
    memset(&ccfg, 0, sizeof(ccfg));
    memset(&fcfg, 0, sizeof(fcfg));
    memset(&cbs, 0, sizeof(cbs));

    ucfg.low_limit  = INT16_MIN;
    ucfg.high_limit = INT16_MAX;
    ucfg.intr_priority = 0;
    ucfg.flags.accum_count = 0;
    /* glitch filter set separately below */

    pcnt_new_unit(&ucfg, out_unit);

    fcfg.max_glitch_ns = PCNT_GLITCH_NS;
    pcnt_unit_set_glitch_filter(*out_unit, &fcfg);

    ccfg.edge_gpio_num  = gpio_a;
    ccfg.level_gpio_num = gpio_b;
    pcnt_new_channel(*out_unit, &ccfg, out_ch);

    /* x2 quadrature: count on both edges of A, direction from B level.
     * (True x4 would need a second channel on edge=B/level=A.)
     * Must match ENCODER_QUAD_FACTOR=2 in config.h. */
    pcnt_channel_set_edge_action(*out_ch,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(*out_ch,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    cbs.on_reach = pcnt_watch_cb;
    pcnt_unit_register_event_callbacks(*out_unit, &cbs, NULL);
    pcnt_unit_enable(*out_unit);
    pcnt_unit_clear_count(*out_unit);
    pcnt_unit_start(*out_unit);
}

/* (Re)arm both hardware watchpoints at w ticks. Removes the previous
 * watchpoint first so repeated per-cell calls never accumulate. */
static void rearm_watchpoints(int32_t w)
{
    int wc = (int)w;

    if (wc > INT16_MAX - 1) {
        wc = INT16_MAX - 1;
    }
    if (wc < INT16_MIN + 1) {
        wc = INT16_MIN + 1;
    }

    if (s_watch_armed) {
        if (s_watch_val == wc) {
            return; /* already armed at this value, nothing to do */
        }
        (void)pcnt_unit_remove_watch_point(s_unit_l, s_watch_val);
        (void)pcnt_unit_remove_watch_point(s_unit_r, s_watch_val);
        s_watch_armed = false;
    }

    if (s_unit_l && s_unit_r) {
        (void)pcnt_unit_add_watch_point(s_unit_l, wc);
        (void)pcnt_unit_add_watch_point(s_unit_r, wc);
        s_watch_val = wc;
        s_watch_armed = true;
    }
}

void encoders_init(void)
{
    make_unit(&s_unit_l, &s_ch_l, PIN_ENC_L_A, PIN_ENC_L_B);
    make_unit(&s_unit_r, &s_ch_r, PIN_ENC_R_A, PIN_ENC_R_B);
    encoders_reset();
}

void encoders_reset(void)
{
    if (s_unit_l) {
        pcnt_unit_clear_count(s_unit_l);
    }
    if (s_unit_r) {
        pcnt_unit_clear_count(s_unit_r);
    }
    s_cell_start_avg = 0;
    s_cell_target = TICKS_PER_CELL;
    g_cell_watch_fired = false;
    rearm_watchpoints(s_cell_target);
}

void encoders_reset_cell_start(void)
{
    /* FIX: reset hardware counters to 0 at the start of EVERY cell.
     * At ~1180 ticks/cell (x2) the 16-bit PCNT window (32767) overflows
     * after ~27 cells if left accumulating, destroying distance tracking.
     * (At the old wrong x4 value ~2359 it was ~13 cells.)
     * After clearing, distances are always relative: start = 0,
     * target = TICKS_PER_CELL. */
    if (s_unit_l) {
        pcnt_unit_clear_count(s_unit_l);
    }
    if (s_unit_r) {
        pcnt_unit_clear_count(s_unit_r);
    }
    s_cell_start_avg = 0;
    s_cell_target = TICKS_PER_CELL;
    g_cell_watch_fired = false;

    /* Re-arm hardware watchpoints at the (constant) per-cell target.
     * Watchpoint is a fast-path hint only; software poll below is
     * authoritative. */
    rearm_watchpoints(s_cell_target);
}

int32_t encoders_get_left(void)
{
    int v = 0;
    if (s_unit_l) {
        pcnt_unit_get_count(s_unit_l, &v);
    }
    return (int32_t)v;
}

int32_t encoders_get_right(void)
{
    int v = 0;
    if (s_unit_r) {
        pcnt_unit_get_count(s_unit_r, &v);
    }
    return (int32_t)v;
}

int32_t encoders_avg(void)
{
    return (encoders_get_left() + encoders_get_right()) / 2;
}

void encoders_arm_cell_target(void)
{
    encoders_reset_cell_start();
}

bool encoders_cell_reached(void)
{
    int32_t avg;

    if (g_cell_watch_fired) {
        /* Verify with real counts to reject spurious trips. */
        avg = encoders_avg();
        if ((avg - s_cell_start_avg) >= (TICKS_PER_CELL - PCNT_WATCH_TOLERANCE)) {
            return true;
        }
        /* False alarm: re-arm flag off and keep driving. */
        g_cell_watch_fired = false;
    }
    avg = encoders_avg();
    return (avg - s_cell_start_avg) >= TICKS_PER_CELL;
}
