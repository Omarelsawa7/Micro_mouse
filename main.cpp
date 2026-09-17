#include "main.h"
#include "config.h"
#include "pid.h"
#include "motors.h"
#include "sensors.h"
#include "encoders.h"
#include "imu.h"
#include "maze.h"

#include <Arduino.h>

/* ================= Robot state machine ================= */

typedef enum {
    ST_IDLE = 0,
    ST_COUNTDOWN,     /* 3 s non-blocking auto-start after DIP 01 (no hand-wave) */
    ST_OBSERVE,       /* observe + plan (explore or return) */
    ST_TURN,
    ST_MOVE,
    ST_SETTLE,
    ST_GOAL_ARRIVED,  /* brief brake then 180 */
    ST_TURNAROUND,
    ST_DONE
} RobotState;

typedef enum {
    PHASE_EXPLORE = 0,
    PHASE_RETURN  = 1
} Phase;

static RobotState s_state = ST_IDLE;
static Phase      s_phase = PHASE_EXPLORE;
static Maze       s_maze;
static PidController s_pid;

static uint32_t s_last_idle_print = 0;
static uint32_t s_countdown_start_ms = 0;
static uint32_t s_last_count_print_ms = 0;
static uint32_t s_state_enter_ms  = 0;
static uint32_t s_turn_start_ms   = 0;
static uint32_t s_move_start_ms   = 0;
static uint32_t s_last_pid_ms     = 0;
static uint32_t s_last_loop_us    = 0;

static Direction s_desired_dir = DIR_NORTH;
static float     s_turn_target_deg = 0.0f;
static bool      s_turn_dir_right = true;

/* ---------- helpers (pure C) ---------- */

static uint8_t read_mode(void)
{
    /* INPUT_PULLUP: switch CLOSED to GND = LOW = 0, OPEN = HIGH = 1.
     * Mode bit1 = SW1, bit0 = SW2. 00 = IDLE, 01 = EXPLORE+RETURN. */
    int sw1 = digitalRead(PIN_DIP_SW1); /* 0 or 1 */
    int sw2 = digitalRead(PIN_DIP_SW2);
    if (sw1 != 0 && sw1 != 1) {
        sw1 = 1;
    }
    if (sw2 != 0 && sw2 != 1) {
        sw2 = 1;
    }
    return (uint8_t)((sw1 << 1) | sw2);
}

static const char *state_name(RobotState s)
{
    switch (s) {
    case ST_IDLE:         return "IDLE";
    case ST_COUNTDOWN:    return "COUNTDOWN";
    case ST_OBSERVE:      return "OBSERVE";
    case ST_TURN:         return "TURN";
    case ST_MOVE:         return "MOVE";
    case ST_SETTLE:       return "SETTLE";
    case ST_GOAL_ARRIVED: return "GOAL";
    case ST_TURNAROUND:   return "TURNAROUND";
    case ST_DONE:         return "DONE";
    }
    return "?";
}

static void enter_state(RobotState s)
{
    s_state = s;
    s_state_enter_ms = millis();
}

static int dir_diff_turn(Direction from, Direction to)
{
    /* 0 = fwd, 1 = right90, 2 = 180, 3 = left90 */
    return ((int)to - (int)from + 4) & 0x03;
}

static void advance_position(bool reached_by_encoders)
{
    switch (s_maze.heading) {
    case DIR_NORTH: s_maze.py++; break;
    case DIR_EAST:  s_maze.px++; break;
    case DIR_SOUTH: s_maze.py--; break;
    case DIR_WEST:  s_maze.px--; break;
    }
    if (s_maze.px < 0) {
        s_maze.px = 0;
    }
    if (s_maze.px >= MAZE_SIZE) {
        s_maze.px = MAZE_SIZE - 1;
    }
    if (s_maze.py < 0) {
        s_maze.py = 0;
    }
    if (s_maze.py >= MAZE_SIZE) {
        s_maze.py = MAZE_SIZE - 1;
    }
    /* FIX: only trust the new cell as mapped if encoders actually measured
     * a full cell. On timeout (stall/wall) we still update px/py
     * (best-effort odometry) but leave visited=false so the strict
     * pessimistic return will NOT route through an unconfirmed cell. */
    if (reached_by_encoders) {
        s_maze.cells[s_maze.px][s_maze.py].visited = true;
    }
}

/* Start a gyro turn: target signed degrees (+ = right, - = left). */
static void turn_begin(float target_deg)
{
    imu_reset_yaw();
    s_turn_target_deg = target_deg;
    s_turn_start_ms = millis();
    s_turn_dir_right = (target_deg >= 0.0f);
}

/* Must be called every loop during ST_TURN / ST_TURNAROUND.
 * Returns true when turn complete. Drives motors directly. */
static bool turn_update(void)
{
    uint32_t now;

    now = millis();

    /* Manual completion check (signed target, yaw integrates signed): */
    {
        float yaw = imu_get_yaw();
        float err = s_turn_target_deg - yaw;
        float aerr = err < 0 ? -err : err;
        bool timeout = (now - s_turn_start_ms) > TURN_TIMEOUT_MS;
        bool reached = aerr <= TURN_TOLERANCE_DEG;
        if (timeout && !reached) {
            /* FIX: old code silently accepted timeout as success, hiding a
             * gyro-sign or stall failure and snapping heading anyway.
             * Warn loudly; caller still snaps heading (best effort). */
            Serial.print("WARN: turn timeout tgt=");
            Serial.print(s_turn_target_deg);
            Serial.print(" yaw=");
            Serial.println(yaw);
        }
        if (reached || timeout) {
            motors_stop();
            return true;
        }
        /* Keep rotating: right turn = left fwd, right rev. */
        if (s_turn_dir_right) {
            motors_drive(MOTOR_TURN_SPEED, -MOTOR_TURN_SPEED);
        } else {
            motors_drive(-MOTOR_TURN_SPEED, MOTOR_TURN_SPEED);
        }
        return false;
    }
}

/* Decide next direction and set up turn/move. Returns false if trapped. */
static bool plan_next_step(bool pessimistic, int tx_desc)
{
    Direction next;
    int diff;
    (void)tx_desc;

    if (pessimistic) {
        maze_flood_to_start(&s_maze, true);
    } else {
        maze_flood_to_goal(&s_maze, false);
    }
    if (!maze_next_dir(&s_maze, s_maze.px, s_maze.py, s_maze.heading,
                       pessimistic, &next)) {
        return false;
    }
    s_desired_dir = next;
    diff = dir_diff_turn(s_maze.heading, next);
    if (diff == 0) {
        /* Go straight: skip TURN state */
        encoders_arm_cell_target();
        pid_reset(&s_pid);
        s_move_start_ms = millis();
        s_last_pid_ms = millis();
        enter_state(ST_MOVE);
    } else if (diff == 1) {
        turn_begin(TURN_90_DEG);
        enter_state(ST_TURN);
    } else if (diff == 3) {
        turn_begin(-TURN_90_DEG);
        enter_state(ST_TURN);
    } else {
        turn_begin(TURN_180_DEG);
        enter_state(ST_TURN);
    }
    return true;
}

/* ================= public ================= */

void robot_setup(void)
{
    Serial.begin(SERIAL_BAUD);

    pinMode(PIN_DIP_SW1, INPUT_PULLUP);
    pinMode(PIN_DIP_SW2, INPUT_PULLUP);

    motors_init();
    sensors_init();
    encoders_init();
    imu_init();

    pid_init(&s_pid, PID_KP, PID_KI, PID_KD, PID_MAX_INTEGRAL, PID_MAX_OUTPUT);
    maze_init(&s_maze);

    imu_calibrate();

    s_last_loop_us = micros();
    s_last_idle_print = millis();

    Serial.println("==== Micromouse V8 boot ====");
    Serial.print("TICKS_PER_CELL=");
    Serial.println(TICKS_PER_CELL);
    Serial.println("Mode 00=IDLE(print) 01=EXPLORE (stop at goal)");
    Serial.println("Keep WiFi/BT OFF. Emitters permanently ON.");
    enter_state(ST_IDLE);
}

void robot_loop(void)
{
    uint8_t mode;
    uint32_t now_ms;
    uint32_t now_us;
    float dt_sec;
    uint16_t v[5];

    now_ms = millis();
    now_us = micros();
    dt_sec = (float)(now_us - s_last_loop_us) / 1000000.0f;
    if (dt_sec < 0.0f) {
        dt_sec = 0.0f;
    }
    if (dt_sec > 0.1f) {
        dt_sec = 0.1f; /* clamp after stalls */
    }
    s_last_loop_us = now_us;

    /* Non-blocking 1 ms round-robin ADC poll (replaces the old timer ISR:
     * analogRead is not ISR-safe and panicked the Interrupt WDT).
     * Must run every loop, including IDLE, so it sits above the mode check. */
    sensors_update();

    /* Gyro integration runs every loop (needed for turns). */
    imu_update(dt_sec);

    /* Any mode other than 01 is IDLE (per owner decision for 10/11). */
    mode = read_mode();
    if (mode != 1) {
        if (s_state != ST_IDLE) {
            motors_stop();
            enter_state(ST_IDLE);
            Serial.println("-> IDLE (mode != 01)");
        }
        /* IDLE: continuously print live sensors + encoders, non-blocking. */
        if ((now_ms - s_last_idle_print) >= IDLE_PRINT_INTERVAL_MS) {
            s_last_idle_print = now_ms;
            sensors_snapshot(v);
            sensors_print(v);
            Serial.print(" | ENC L:");
            Serial.print(encoders_get_left());
            Serial.print(" R:");
            Serial.print(encoders_get_right());
            Serial.print(" AVG:");
            Serial.print(encoders_avg());
            Serial.print(" | YAW:");
            Serial.print(imu_get_yaw());
            Serial.print(" | POS:");
            Serial.print(s_maze.px);
            Serial.print(",");
            Serial.print(s_maze.py);
            Serial.print(" H:");
            Serial.print((int)s_maze.heading);
            Serial.println();
        }
        /* Reset mission so re-entering 01 starts clean. */
        s_phase = PHASE_EXPLORE;
        return;
    }

    /* ---- mode == 01 mission ---- */
    sensors_snapshot(v);

    switch (s_state) {
    case ST_IDLE:
        Serial.println("Mode 01: auto-start in 3 s (flip DIP to 00 to abort)...");
        maze_init(&s_maze);
        s_phase = PHASE_EXPLORE;
        pid_reset(&s_pid);
        motors_stop();
        s_countdown_start_ms = now_ms;
        s_last_count_print_ms = now_ms;
        enter_state(ST_COUNTDOWN);
        break;

    case ST_COUNTDOWN: {
        /* 3-second non-blocking auto-start after DIP S1 -> 01. NO delay():
         * we return every loop; sensors/IMU keep updating. Aborts on its
         * own if DIP leaves 01: the top-of-loop mode check forces IDLE. */
        uint32_t elapsed = now_ms - s_countdown_start_ms;
        if (elapsed >= (uint32_t)START_COUNTDOWN_MS) {
            Serial.println("START explore.");
            encoders_reset();
            enter_state(ST_OBSERVE);
        } else if ((now_ms - s_last_count_print_ms) >= 500) {
            /* Throttled remaining-seconds print, no delay(). */
            uint32_t remain_ms = (uint32_t)START_COUNTDOWN_MS - elapsed;
            s_last_count_print_ms = now_ms;
            Serial.print("Starting in ");
            Serial.print((unsigned long)((remain_ms + 999u) / 1000u));
            Serial.println(" s...");
        }
        break;
    }

    case ST_OBSERVE: {
        bool wl, wr, wf;
        wl = sensors_has_wall_left(v);
        wr = sensors_has_wall_right(v);
        wf = sensors_has_wall_front(v);
        maze_observe(&s_maze, wl, wr, wf);

        Serial.print(s_phase == PHASE_EXPLORE ? "EXPLORE @" : "RETURN @");
        Serial.print(s_maze.px);
        Serial.print(",");
        Serial.print(s_maze.py);
        Serial.print(" H");
        Serial.print((int)s_maze.heading);
        Serial.print(" W L/R/F:");
        Serial.print(wl);
        Serial.print("/");
        Serial.print(wr);
        Serial.print("/");
        Serial.println(wf);

        if (s_phase == PHASE_EXPLORE && maze_is_goal(s_maze.px, s_maze.py)) {
            Serial.println("GOAL reached. Mission complete. Stopping forever.");
            motors_stop();
            enter_state(ST_DONE);
            break;
        }
        /* Explore-only: stop at goal, never return. The PHASE_RETURN branch
         * and ST_GOAL_ARRIVED/ST_TURNAROUND below are kept but unreachable. */
        if (s_phase == PHASE_RETURN && maze_is_start(s_maze.px, s_maze.py)) {
            Serial.println("HOME (0,0). Mission complete. Stopping forever.");
            motors_stop();
            enter_state(ST_DONE);
            break;
        }
        if (s_phase == PHASE_EXPLORE) {
            if (!plan_next_step(false, 0)) {
                Serial.println("ERROR: trapped in EXPLORE. Stopping.");
                motors_stop();
                enter_state(ST_DONE);
            }
        } else {
            /* Strict pessimistic return: unknown = wall. */
            if (!plan_next_step(true, 0)) {
                Serial.println("ERROR: no safe mapped path home (pessimistic). Stopping.");
                motors_stop();
                enter_state(ST_DONE);
            } else {
                Serial.println("Return step planned (pessimistic flood).");
            }
        }
        break;
    }

    case ST_TURN:
        if (turn_update()) {
            /* Snap heading to desired (gyro may have small residual). */
            s_maze.heading = s_desired_dir;
            Serial.print("Turn done. Heading=");
            Serial.println((int)s_maze.heading);
            encoders_arm_cell_target();
            pid_reset(&s_pid);
            s_move_start_ms = now_ms;
            s_last_pid_ms = now_ms;
            enter_state(ST_MOVE);
        }
        break;

    case ST_MOVE: {
        bool valid;
        float err;
        float corr;
        bool arrived;
        bool timeout;

        /* PID at ~5 ms cadence, non-blocking. FIX: use measured dt instead
         * of the fixed PID_SAMPLE_DT_SEC so loop jitter doesn't scale D. */
        if ((now_ms - s_last_pid_ms) >= 5) {
            uint32_t dt_ms = now_ms - s_last_pid_ms;
            float dt_meas = (float)dt_ms / 1000.0f;
            if (dt_meas < 0.002f) {
                dt_meas = 0.002f;
            } else if (dt_meas > 0.05f) {
                dt_meas = 0.05f;
            }
            s_last_pid_ms = now_ms;
            err = sensors_centering_error(v, &valid);
            if (valid) {
                corr = pid_compute(&s_pid, err, dt_meas);
            } else {
                /* No side walls: drive straight, decay integral. */
                pid_reset(&s_pid);
                corr = 0.0f;
            }
            motors_drive_base_with_correction(MOTOR_BASE_SPEED, corr);
        }
        arrived = encoders_cell_reached();
        timeout = (now_ms - s_move_start_ms) > CELL_TIMEOUT_MS;
        if (arrived || timeout) {
            motors_brake();
            if (timeout && !arrived) {
                Serial.println("WARN: cell timeout, forcing advance (unconfirmed).");
            }
            advance_position(arrived);
            Serial.print("Cell done @");
            Serial.print(s_maze.px);
            Serial.print(",");
            Serial.println(s_maze.py);
            enter_state(ST_SETTLE);
        }
        break;
    }

    case ST_SETTLE:
        /* Short brake settle, non-blocking. */
        if ((now_ms - s_state_enter_ms) >= SETTLE_MS) {
            motors_stop();
            enter_state(ST_OBSERVE);
        }
        break;

    case ST_GOAL_ARRIVED:
        /* Unused in explore-only mode (kept for possible revert). */
        if ((now_ms - s_state_enter_ms) >= SETTLE_MS) {
            turn_begin(TURN_180_DEG);
            enter_state(ST_TURNAROUND);
        }
        break;

    case ST_TURNAROUND:
        if (turn_update()) {
            s_maze.heading = (Direction)(((int)s_maze.heading + 2) & 0x03);
            s_phase = PHASE_RETURN;
            Serial.print("Turnaround done. Heading=");
            Serial.println((int)s_maze.heading);
            Serial.println("Begin STRICT pessimistic return (mapped path only).");
            maze_flood_to_start(&s_maze, true);
            maze_print_dist(&s_maze);
            enter_state(ST_OBSERVE);
        }
        break;

    case ST_DONE:
        motors_stop();
        /* Stop forever per spec. Still service loop + allow exit via DIP 00. */
        break;
    }

    (void)state_name; /* keep helper for debugger */
}
