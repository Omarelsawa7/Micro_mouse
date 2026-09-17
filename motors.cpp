#include "motors.h"
#include "config.h"

#include <Arduino.h>
#include <stdlib.h>

/* ESP-IDF version gate for LEDC API (core 2.x vs 3.x) */
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define USE_LEDC_ATTACH_NEW 1
#else
#define USE_LEDC_ATTACH_NEW 0
#endif

static int apply_deadzone(int v)
{
    int mag;

    if (v == 0) {
        return 0;
    }
    mag = abs(v);
    if (mag < MOTOR_MIN_DEADZONE) {
        mag = MOTOR_MIN_DEADZONE;
    }
    if (mag > MOTOR_PWM_MAX) {
        mag = MOTOR_PWM_MAX;
    }
    return (v > 0) ? mag : -mag;
}

static void write_phase_enable(uint8_t pin_pwm, uint8_t ch_pwm,
                               uint8_t pin_low, uint8_t ch_low,
                               int speed)
{
    int pwm;

    if (speed > MOTOR_PWM_MAX) {
        speed = MOTOR_PWM_MAX;
    } else if (speed < -MOTOR_PWM_MAX) {
        speed = -MOTOR_PWM_MAX;
    }
    speed = apply_deadzone(speed);

    if (speed >= 0) {
#if USE_LEDC_ATTACH_NEW
        (void)ch_pwm; (void)ch_low;
        ledcWrite(pin_pwm, (uint32_t)speed);
        ledcWrite(pin_low, 0);
#else
        ledcWrite(ch_pwm, (uint32_t)speed);
        ledcWrite(ch_low, 0);
#endif
        (void)pin_pwm; (void)pin_low;
        pwm = speed;
        (void)pwm;
    } else {
#if USE_LEDC_ATTACH_NEW
        (void)ch_pwm; (void)ch_low;
        ledcWrite(pin_pwm, 0);
        ledcWrite(pin_low, (uint32_t)(-speed));
#else
        ledcWrite(ch_pwm, 0);
        ledcWrite(ch_low, (uint32_t)(-speed));
#endif
    }
}

void motors_init(void)
{
    /* Direction pins as outputs, start stopped (strapping-pin kick is
     * hardware-unavoidable on GPIO 5/2, acknowledged by owner). */
    pinMode(PIN_MOT_R_IN1, OUTPUT);
    pinMode(PIN_MOT_R_IN2, OUTPUT);
    pinMode(PIN_MOT_L_IN3, OUTPUT);
    pinMode(PIN_MOT_L_IN4, OUTPUT);

#if USE_LEDC_ATTACH_NEW
    ledcAttach(PIN_MOT_R_IN1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcAttach(PIN_MOT_R_IN2, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcAttach(PIN_MOT_L_IN3, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcAttach(PIN_MOT_L_IN4, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
#else
    ledcSetup(LEDC_CH_R_IN1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcSetup(LEDC_CH_R_IN2, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcSetup(LEDC_CH_L_IN3, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcSetup(LEDC_CH_L_IN4, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcAttachPin(PIN_MOT_R_IN1, LEDC_CH_R_IN1);
    ledcAttachPin(PIN_MOT_R_IN2, LEDC_CH_R_IN2);
    ledcAttachPin(PIN_MOT_L_IN3, LEDC_CH_L_IN3);
    ledcAttachPin(PIN_MOT_L_IN4, LEDC_CH_L_IN4);
#endif

    motors_stop();
}

void motors_stop(void)
{
#if USE_LEDC_ATTACH_NEW
    ledcWrite(PIN_MOT_R_IN1, 0);
    ledcWrite(PIN_MOT_R_IN2, 0);
    ledcWrite(PIN_MOT_L_IN3, 0);
    ledcWrite(PIN_MOT_L_IN4, 0);
#else
    ledcWrite(LEDC_CH_R_IN1, 0);
    ledcWrite(LEDC_CH_R_IN2, 0);
    ledcWrite(LEDC_CH_L_IN3, 0);
    ledcWrite(LEDC_CH_L_IN4, 0);
#endif
}

/* Active brake: both IN HIGH (shorts motor) at low duty. */
void motors_brake(void)
{
#if USE_LEDC_ATTACH_NEW
    ledcWrite(PIN_MOT_R_IN1, 255);
    ledcWrite(PIN_MOT_R_IN2, 255);
    ledcWrite(PIN_MOT_L_IN3, 255);
    ledcWrite(PIN_MOT_L_IN4, 255);
#else
    ledcWrite(LEDC_CH_R_IN1, 255);
    ledcWrite(LEDC_CH_R_IN2, 255);
    ledcWrite(LEDC_CH_L_IN3, 255);
    ledcWrite(LEDC_CH_L_IN4, 255);
#endif
}

void motors_drive(int left_speed, int right_speed)
{
    /* FIX: motors are mirrored on the chassis. Logical + = forward for both
     * wheels; flip physical sign per MOTOR_*_INVERT so equal inputs drive
     * straight instead of spinning in place. */
#if MOTOR_LEFT_INVERT
    left_speed = -left_speed;
#endif
#if MOTOR_RIGHT_INVERT
    right_speed = -right_speed;
#endif
    write_phase_enable(PIN_MOT_R_IN1, LEDC_CH_R_IN1,
                       PIN_MOT_R_IN2, LEDC_CH_R_IN2, right_speed);
    write_phase_enable(PIN_MOT_L_IN3, LEDC_CH_L_IN3,
                       PIN_MOT_L_IN4, LEDC_CH_L_IN4, left_speed);
}

void motors_drive_base_with_correction(int base_speed, float correction)
{
    int c = (int)correction;
    int left;
    int right;

    if (c > MOTOR_CORRECTION_LIMIT) {
        c = MOTOR_CORRECTION_LIMIT;
    } else if (c < -MOTOR_CORRECTION_LIMIT) {
        c = -MOTOR_CORRECTION_LIMIT;
    }
    /* Positive correction = drifting right? Convention fixed in sensors:
     * error = (left - target_left) - (right - target_right).
     * error > 0 means too close to left wall -> steer right:
     * slow left, speed right. */
    left  = base_speed - c;
    right = base_speed + c;
    motors_drive(left, right);
}
