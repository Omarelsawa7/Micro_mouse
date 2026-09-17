#ifndef CONFIG_H
#define CONFIG_H

/*
 * Micromouse V8 - Centralized Configuration
 * Target: ESP32 (Arduino-ESP32 core, pure C modules)
 * Framework: Arduino APIs + ESP-IDF driver/pulse_cnt.h, Wire, LEDC
 * Style: STRICT C, no classes, no delay() in logic, Serial @ 115200
 */

#include <stdint.h>
#include <math.h>

/* ================= Serial ================= */
#define SERIAL_BAUD              115200

/* ================= Pin Mapping (DO NOT CHANGE) ================= */
/* IR receivers (analog, photodiode: more light = higher ADC) */
#define PIN_IR_FAR_LEFT          33
#define PIN_IR_LEFT              36
#define PIN_IR_FRONT             39
#define PIN_IR_RIGHT             34
#define PIN_IR_FAR_RIGHT         35

/* IR emitters (digital, permanently ON) */
#define PIN_EM_FAR_LEFT          27
#define PIN_EM_LEFT              26
#define PIN_EM_FRONT             25
#define PIN_EM_RIGHT             14
#define PIN_EM_FAR_RIGHT         32

/* Motors (TB6612/L298 style, phase-enable PWM on IN pins) */
#define PIN_MOT_R_IN1            23
#define PIN_MOT_R_IN2            19
#define PIN_MOT_L_IN3            12
#define PIN_MOT_L_IN4            2

/* Encoders (quadrature) */
#define PIN_ENC_R_A              4
#define PIN_ENC_R_B              16
#define PIN_ENC_L_A              17
#define PIN_ENC_L_B              18

/* IMU I2C (MPU6050 only) */
#define PIN_I2C_SDA              22
#define PIN_I2C_SCL              21
#define MPU6050_ADDR             0x68

/* DIP switches (wired to GND, use INPUT_PULLUP: CLOSED=LOW=0, OPEN=HIGH=1) */
#define PIN_DIP_SW1              15
#define PIN_DIP_SW2              5

/* ================= Mechanical Placeholders (EDIT THESE) ================= */
#define WHEEL_DIAMETER_MM        34.0f      /* <-- measure your wheel */
#define GEAR_RATIO               50.0f      /* <-- motor gearbox, e.g. 30:1 */
#define ENCODER_CPR              7         /* <-- ticks per MOTOR shaft rev (before gear), single channel */
#define CELL_SIZE_MM             180.0f     /* 18x18 cm cells (180 mm). Fixed. */

#ifndef PI_CONST
#define PI_CONST                 3.14159265f
#endif

/* Ticks per wheel revolution.
 * encoders.cpp uses ONE PCNT channel per wheel (edge=A, level=B) with
 * INCREASE on one edge / DECREASE on the other = 2 counts per full
 * quadrature cycle (x2 decoding), NOT x4. Full x4 needs a second channel
 * (edge=B, level=A). Keep this at 2 unless you add that second channel.
 * ENCODER_QUAD_FACTOR = 2 for current firmware, 4 only with dual-channel.
 */
#define ENCODER_QUAD_FACTOR      2

/* Exact math required by spec:
 * TICKS_PER_CELL = (CELL_SIZE_MM / (WHEEL_DIAMETER_MM * PI)) * ENCODER_CPR * GEAR_RATIO
 * Multiplied by QUAD_FACTOR for x4 decoding.
 */
#define TICKS_PER_CELL_FLOAT \
    (((CELL_SIZE_MM) / ((WHEEL_DIAMETER_MM) * (PI_CONST))) * ((float)(ENCODER_CPR)) * (GEAR_RATIO) * ((float)ENCODER_QUAD_FACTOR))

#define TICKS_PER_CELL           ((int32_t)((TICKS_PER_CELL_FLOAT) + 0.5f))

/* Derived helpers */
#define TICKS_PER_MM_FLOAT       ((TICKS_PER_CELL_FLOAT) / (CELL_SIZE_MM))
#define TICKS_PER_REV_FLOAT      (((float)(ENCODER_CPR)) * (GEAR_RATIO) * ((float)ENCODER_QUAD_FACTOR))

/* Example with defaults (34mm, 50:1, 7 CPR, x2):
 * revs per cell = 180 / (34*PI) = ~1.685
 * ticks = 1.685 * 7 * 50 * 2 = ~1180 ticks/cell
 * (x4 would be ~2359 but firmware is x2 -- see above.)
 * VERIFY on first run via IDLE serial print: push robot exactly 1 cell, check counts.
 */

/* ================= Motors / PWM ================= */
#define MOTOR_PWM_FREQ_HZ        20000
#define MOTOR_PWM_RES_BITS       8
#define MOTOR_PWM_MAX            255
#define MOTOR_BASE_SPEED         160        /* straight-line base PWM */
#define MOTOR_TURN_SPEED         140        /* rotation PWM for IMU turns */
#define MOTOR_MIN_DEADZONE       70         /* minimum PWM to overcome static friction */
#define MOTOR_CORRECTION_LIMIT   80         /* max PID correction added to base */

/* LEDC channels (one per IN pin, phase-enable mode) */
#define LEDC_CH_R_IN1            0
#define LEDC_CH_R_IN2            1
#define LEDC_CH_L_IN3            2
#define LEDC_CH_L_IN4            3

/* Motor polarity: left/right motors are mirrored on the chassis.
 * Giving both the same PWM sign spins in place, so invert one side in
 * software. Logical + = forward for BOTH wheels; motors_drive() flips
 * the physical sign for any side with INVERT=1. */
#define MOTOR_LEFT_INVERT        1
#define MOTOR_RIGHT_INVERT       0

/* ================= IR thresholds (raw ADC 0-4095, ESP32 12-bit) =================
 * Photodiode + emitter always ON: closer wall = higher reading.
 * Calibrate in IDLE mode (00) via Serial print, then update these.
 */
#define IR_LEFT_WALL_THRESH      500
#define IR_RIGHT_WALL_THRESH     500
#define IR_FRONT_WALL_THRESH     700
#define IR_FAR_LEFT_THRESH       400
#define IR_FAR_RIGHT_THRESH      400

/* Hand-wave start trigger (must be clearly above open-air value, below touch value) */
#define HAND_WAVE_THRESHOLD      300

/* Ideal centering targets: raw ADC you see when perfectly centered in a cell.
 * PID drives (left - right) toward (TARGET_LEFT - TARGET_RIGHT) ~= 0 if symmetric.
 */
#define IR_TARGET_LEFT_ADC       1800
#define IR_TARGET_RIGHT_ADC      1800
#define IR_NO_WALL_MAX           350        /* below this = treat as no wall, ignore PID */

/* ================= PID (IR wall centering) =================
 * NOTE: error unit = raw ADC counts (typ. +/-1000). KD acts on
 * (err-prev)/dt, so with dt=0.005 even small noise explodes. Keep KD small
 * (<=0.008) unless you add filtering in pid.cpp. */
#define PID_KP                   0.06f
#define PID_KI                   0.001f
#define PID_KD                   0.008f
#define PID_MAX_INTEGRAL         500.0f
#define PID_MAX_OUTPUT           80.0f      /* must be <= MOTOR_CORRECTION_LIMIT */
#define PID_SAMPLE_DT_SEC        0.005f     /* 5 ms main-loop PID update */

/* ================= Sensors polling ================= */

#define SENSOR_TIMER_INTERVAL_US 1000       /* 1 ms poll, round-robin 1ch/tick = 5 ms full scan */
#define SENSOR_COUNT             5

/* ================= Encoders / PCNT ================= */
#define PCNT_GLITCH_NS           1000
#define PCNT_WATCH_TOLERANCE     5          /* ticks tolerance for arrival check */

/* ================= IMU / Turns ================= */
#define GYRO_SENS_250DPS         131.0f     /* LSB per deg/s */
/* Sign of yaw integration. +1 = raw gyro Z positive when turning RIGHT.
 * VERIFY: IDLE-print YAW, rotate robot 90 deg RIGHT by hand. YAW must go
 * toward +90. If it goes toward -90, set this to -1. Wrong sign makes every
 * turn hit TURN_TIMEOUT_MS and corrupt heading. */
#define IMU_GYRO_SIGN            (+1.0f)
#define TURN_TOLERANCE_DEG       2.0f
#define TURN_TIMEOUT_MS          3000
#define TURN_90_DEG              90.0f
#define TURN_180_DEG             180.0f
#define IMU_CALIB_SAMPLES        500

/* ================= Timing (non-blocking, NO delay()) ================= */
#define IDLE_PRINT_INTERVAL_MS   200
#define HAND_WAVE_HOLD_MS        1000       /* unused since auto-start (hand-wave removed) */
#define START_COUNTDOWN_MS       3000       /* auto-start delay after DIP 01, non-blocking */
#define CELL_TIMEOUT_MS          4000
#define SETTLE_MS                80         /* short brake settle between cells */

/* ================= Maze =================
 * Target maze: 8x8 cells, 18x18 cm each. Goal = centre 2x2 (3..4, 3..4).
 * Overridable for host GUI testing (e.g. -DMAZE_SIZE=16). Firmware default 8. */
#ifndef MAZE_SIZE
#define MAZE_SIZE                8
#endif
#ifndef MAZE_GOAL_MIN
#define MAZE_GOAL_MIN            3
#endif
#ifndef MAZE_GOAL_MAX
#define MAZE_GOAL_MAX            4
#endif
#define START_X                  0
#define START_Y                  0

#endif 
