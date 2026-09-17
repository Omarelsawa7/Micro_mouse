#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/pulse_cnt.h"

/* Two PCNT units (left + right), full quadrature decode.
 * Watchpoint = cell target; ISR flag signals arrival (non-blocking poll). */

void encoders_init(void);
void encoders_reset(void);
void encoders_reset_cell_start(void);

int32_t encoders_get_left(void);
int32_t encoders_get_right(void);
int32_t encoders_avg(void);

/* Set watchpoint target = start + TICKS_PER_CELL. Call at cell entry. */
void encoders_arm_cell_target(void);

/* True when avg(left,right) delta from arm point >= TICKS_PER_CELL. */
bool encoders_cell_reached(void);

/* Expose flag for ISR + debug */
extern volatile bool g_cell_watch_fired;

#endif /* ENCODERS_H */
