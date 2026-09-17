#include "imu.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>

#define REG_PWR_MGMT_1   0x6B
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_GYRO_Z_H     0x47

static float s_yaw_deg = 0.0f;
static float s_gyro_offset = 0.0f;
static float s_gyro_z_dps = 0.0f;

static void write_reg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

static int16_t read_gyro_z_raw(void)
{
    int16_t v;

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_GYRO_Z_H);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU6050_ADDR, 2, 1);
    if (Wire.available() < 2) {
        return 0;
    }
    v = (int16_t)((Wire.read() << 8) | Wire.read());
    return v;
}

void imu_init(void)
{
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    write_reg(REG_PWR_MGMT_1, 0x00);  /* wake */
    write_reg(REG_CONFIG, 0x03);      /* DLPF ~44 Hz, stable for turns */
    write_reg(REG_GYRO_CONFIG, 0x00); /* +-250 dps */
    s_yaw_deg = 0.0f;
    s_gyro_offset = 0.0f;
    s_gyro_z_dps = 0.0f;
}

void imu_calibrate(void)
{
    int32_t sum = 0;
    int count = 0;
    uint32_t t0;
    int16_t raw;

    Serial.println("IMU: calibrating gyro Z, keep robot still...");
    t0 = millis();
    /* Non-blocking style: tight sample loop without delay(), bounded by count.
     * I2C itself paces the loop (~1-2 ms per read). No delay() used. */
    while (count < IMU_CALIB_SAMPLES) {
        /* Timeout guard: 5 s max */
        if ((millis() - t0) > 5000) {
            break;
        }
        raw = read_gyro_z_raw();
        sum += raw;
        count++;
    }
    if (count > 0) {
        s_gyro_offset = (float)sum / (float)count;
    } else {
        s_gyro_offset = 0.0f;
    }
    s_yaw_deg = 0.0f;
    Serial.print("IMU: offset=");
    Serial.println(s_gyro_offset);
}

void imu_reset_yaw(void)
{
    s_yaw_deg = 0.0f;
}

void imu_update(float dt_sec)
{
    int16_t raw;
    float dps;

    if (dt_sec <= 0.0f || dt_sec > 0.1f) {
        return; /* reject bogus dt (loop stall) */
    }
    raw = read_gyro_z_raw();
    dps = ((float)raw - s_gyro_offset) / GYRO_SENS_250DPS;
    dps *= IMU_GYRO_SIGN; /* + = right turn gives +yaw; flip in config.h if reversed */
    s_gyro_z_dps = dps;
    s_yaw_deg += dps * dt_sec;
}

float imu_get_yaw(void)
{
    return s_yaw_deg;
}

float imu_get_gyro_z_dps(void)
{
    return s_gyro_z_dps;
}

bool imu_turn_step(float target_deg, uint32_t now_ms, uint32_t *start_ms)
{
    float err;

    if (start_ms == 0) {
        return false;
    }
    if (*start_ms == 0) {
        *start_ms = now_ms;
    }
    /* Timeout -> accept to avoid deadlock from drift */
    if ((now_ms - *start_ms) > TURN_TIMEOUT_MS) {
        return true;
    }
    err = target_deg - s_yaw_deg;
    if (err < 0.0f) {
        err = -err;
    }
    return err <= TURN_TOLERANCE_DEG;
}
