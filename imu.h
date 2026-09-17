#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

/* MPU6050 gyro-Z yaw integrator. Used EXCLUSIVELY for 90/180 deg turns.
 * Call imu_update() every main loop with dt in seconds. */

void  imu_init(void);
void  imu_calibrate(void); /* blocking-ish but uses millis(), no delay() */
void  imu_reset_yaw(void);
void  imu_update(float dt_sec);
float imu_get_yaw(void);
float imu_get_gyro_z_dps(void);

/* Non-blocking turn helpers (call each loop, returns true when done) */
bool  imu_turn_step(float target_deg, uint32_t now_ms, uint32_t *start_ms);

#endif /* IMU_H */
