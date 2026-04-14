/**
 * @file motor_foc.h
 * @brief Voltage-mode FOC (Field Oriented Control) motor control interface
 */
#pragma once

/** @brief Clamp value x between min_val and max_val */
#define _limit(x, min_val, max_val)     ((x) < (min_val) ? (min_val) : ((x) > (max_val) ? (max_val) : (x)))

/**
 * @brief Motor control mode enumeration
 */
typedef enum
{
    control_type_null,                  /* No control */
    control_type_position,              /* Position control */
    control_type_speed,                 /* Speed control */
    control_type_speed_torque,          /* Speed-torque cascade control */
    control_type_torque,                /* Voltage-command torque control */
    control_type_position_speed_torque, /* Position-speed-torque cascade control */
} motor_control_type;

/**
 * @brief Motor control context (target values for each control mode)
 */
typedef struct {
    motor_control_type type;
    float position;        /* Target mechanical angle (rad) */
    float speed;           /* Target speed (rad/s) */
    float torque_norm_d;   /* Target d-axis voltage command magnitude (0~1) */
    float torque_norm_q;   /* Target q-axis voltage command magnitude (0~1) */
    float max_speed;       /* Max speed for cascade control (rad/s) */
    float max_torque_norm; /* Max q-axis torque for cascade control (0~1) */
} motor_control_context_t;

/**
 * @brief Zero-point calibration state machine
 */
typedef enum {
    CALI_IDLE = 0,         /* Idle, not calibrating */
    CALI_LOCKING,          /* Locking rotor to electrical zero */
    CALI_SAMPLING,         /* Sampling encoder to determine zero angle */
    CALI_DONE              /* Calibration complete */
} zero_cali_state_t;

extern volatile zero_cali_state_t zero_cali_state;
extern motor_control_context_t motor_control_context;
void motor_control_set_context(const motor_control_context_t *ctx);
void motor_control_get_context(motor_control_context_t *ctx_out);
void motor_control_estop(void);

/** @brief Inverse Park + SVPWM + set PWM duty */
void foc_forward(float d, float q, float rotor_rad);

void lib_position_control(float rad);                                                           /* Position (angle) control */
void lib_speed_control(float speed);                                                            /* Speed control */
void lib_torque_control(float torque_norm_d, float torque_norm_q);                              /* Voltage-command torque control */
void lib_speed_torque_control(float speed, float max_torque_norm);                              /* Speed-torque cascade */
void lib_position_speed_torque_control(float position, float max_speed, float max_torque_norm); /* Position-speed-torque cascade */

/** @brief Compute cyclic difference, wrapping within [-cycle/2, cycle/2] */
float cycle_diff(float diff, float cycle);

/**
 * @brief Set PID parameters for all four loops
 */
void set_motor_pid(
    float position_p, float position_i, float position_d,
    float speed_p, float speed_i, float speed_d,
    float torque_d_p, float torque_d_i, float torque_d_d,
    float torque_q_p, float torque_q_i, float torque_q_d
);

/** @brief Main motor control update, called from TIM3 ISR */
void motor_control_update(void);
