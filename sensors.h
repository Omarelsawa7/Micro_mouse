#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    IR_FAR_LEFT = 0,
    IR_LEFT     = 1,
    IR_FRONT    = 2,
    IR_RIGHT    = 3,
    IR_FAR_RIGHT= 4
} IrIndex;

/* Live IR readings, updated by sensors_update() (1 channel per 1 ms tick,
 * round-robin, 5 ms full scan). Written from normal loop context only --
 * NO ISR involved, so plain reads/writes are safe. Main loop must call
 * sensors_update() every iteration and treat g_ir_raw as read-only. */
extern volatile uint16_t g_ir_raw[5];

void sensors_init(void);

/* Non-blocking round-robin ADC poll. Call once per robot_loop() iteration.
 * Internally gates on micros() using SENSOR_TIMER_INTERVAL_US; does at most
 * one analogRead() per call, so it never blocks and never trips the WDT. */
void sensors_update(void);

/* Non-blocking snapshots (plain copy; no ISR, no critical section needed) */
void sensors_snapshot(uint16_t out[5]);

/* Wall predicates using config.h thresholds */
bool sensors_has_wall_left(const uint16_t v[5]);
bool sensors_has_wall_right(const uint16_t v[5]);
bool sensors_has_wall_front(const uint16_t v[5]);
bool sensors_hand_detected(const uint16_t v[5]);

/* Centering error for PID. Returns 0 if no usable side wall.
 * Sets *valid = true only when at least one side wall is solid. */
float sensors_centering_error(const uint16_t v[5], bool *valid);

void sensors_print(const uint16_t v[5]);

#endif /* SENSORS_H */
