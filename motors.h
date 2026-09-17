#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

/* Phase-enable drive: PWM one IN pin, LOW the other.
 * speed range: -255..+255 (negative = reverse). */

void motors_init(void);
void motors_stop(void);
void motors_brake(void);
void motors_drive(int left_speed, int right_speed);
void motors_drive_base_with_correction(int base_speed, float correction);

#endif /* MOTORS_H */
