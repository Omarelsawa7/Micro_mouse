#include "sensors.h"
#include "config.h"

#include <Arduino.h>

volatile uint16_t g_ir_raw[5] = {0, 0, 0, 0, 0};

static const uint8_t k_rx_pins[SENSOR_COUNT] = {
    PIN_IR_FAR_LEFT, PIN_IR_LEFT, PIN_IR_FRONT, PIN_IR_RIGHT, PIN_IR_FAR_RIGHT
};

static const uint8_t k_em_pins[SENSOR_COUNT] = {
    PIN_EM_FAR_LEFT, PIN_EM_LEFT, PIN_EM_FRONT, PIN_EM_RIGHT, PIN_EM_FAR_RIGHT
};

/* Round-robin poll state (loop context only, no ISR, no mux needed). */
static uint8_t s_next_ch = 0;
static unsigned long s_last_sample_us = 0;
static bool s_first_run = true;

void sensors_init(void)
{
    uint8_t i;

    for (i = 0; i < SENSOR_COUNT; i++) {
        pinMode(k_em_pins[i], OUTPUT);
        digitalWrite(k_em_pins[i], HIGH); /* permanently ON, continuous mode */
    }

    analogReadResolution(12);
    /* analogSetAttenuation(ADC_11db) is default on most cores; keep default
     * for full 0-3.3V range with photodiodes. */

    s_next_ch = 0;
    s_first_run = true;
    s_last_sample_us = 0;

    /* NOTE: No hardware timer / ISR here by design. analogRead() uses
     * mutexes internally and is NOT ISR-safe on ESP32 Arduino (Interrupt
     * WDT panic). All sampling happens in sensors_update() below. */
}

/* Non-blocking 1 ms round-robin poll. Safe to call every robot_loop(). */
void sensors_update(void)
{
    unsigned long now;
    uint8_t ch;

    now = micros();
    if (s_first_run) {
        s_first_run = false;
        s_last_sample_us = now;
        /* Take an immediate first sample so IDLE prints are live at boot. */
    } else if ((now - s_last_sample_us) < (unsigned long)SENSOR_TIMER_INTERVAL_US) {
        return; /* not yet due: at most one analogRead per 1 ms */
    } else {
        /* Advance timestamp by exactly one interval to avoid drift.
         * If we fell behind by more than one interval (stall), resync. */
        s_last_sample_us += (unsigned long)SENSOR_TIMER_INTERVAL_US;
        if ((now - s_last_sample_us) >= (unsigned long)SENSOR_TIMER_INTERVAL_US) {
            s_last_sample_us = now;
        }
    }

    ch = s_next_ch;
    if (ch >= SENSOR_COUNT) {
        ch = 0;
    }
    /* Loop context: analogRead is legal here (WiFi/BT must stay OFF;
     * GPIO14 is ADC2, reads low if WiFi is enabled). */
    g_ir_raw[ch] = (uint16_t)analogRead(k_rx_pins[ch]);

    ch++;
    if (ch >= SENSOR_COUNT) {
        ch = 0;
    }
    s_next_ch = ch;
}

void sensors_snapshot(uint16_t out[5])
{
    uint8_t i;
    /* No ISR writer exists, so a plain copy is consistent. */
    for (i = 0; i < SENSOR_COUNT; i++) {
        out[i] = g_ir_raw[i];
    }
}

bool sensors_has_wall_left(const uint16_t v[5])
{
    return v[IR_LEFT] > IR_LEFT_WALL_THRESH;
}

bool sensors_has_wall_right(const uint16_t v[5])
{
    return v[IR_RIGHT] > IR_RIGHT_WALL_THRESH;
}

bool sensors_has_wall_front(const uint16_t v[5])
{
    return v[IR_FRONT] > IR_FRONT_WALL_THRESH;
}

bool sensors_hand_detected(const uint16_t v[5])
{
    return v[IR_FRONT] > HAND_WAVE_THRESHOLD;
}

float sensors_centering_error(const uint16_t v[5], bool *valid)
{
    bool has_l;
    bool has_r;
    float err_l;
    float err_r;

    has_l = v[IR_LEFT] > IR_NO_WALL_MAX;
    has_r = v[IR_RIGHT] > IR_NO_WALL_MAX;

    if (!has_l && !has_r) {
        if (valid) {
            *valid = false;
        }
        return 0.0f;
    }
    if (valid) {
        *valid = true;
    }

    /* Normalize around ideal center values so symmetric maze => ~0 error. */
    err_l = (float)v[IR_LEFT] - (float)IR_TARGET_LEFT_ADC;
    err_r = (float)v[IR_RIGHT] - (float)IR_TARGET_RIGHT_ADC;

    if (has_l && has_r) {
        return err_l - err_r;
    }
    if (has_l) {
        return err_l;
    }
    return -err_r;
}

void sensors_print(const uint16_t v[5])
{
    Serial.print("IR FL:");
    Serial.print(v[IR_FAR_LEFT]);
    Serial.print(" L:");
    Serial.print(v[IR_LEFT]);
    Serial.print(" F:");
    Serial.print(v[IR_FRONT]);
    Serial.print(" R:");
    Serial.print(v[IR_RIGHT]);
    Serial.print(" FR:");
    Serial.print(v[IR_FAR_RIGHT]);
}
