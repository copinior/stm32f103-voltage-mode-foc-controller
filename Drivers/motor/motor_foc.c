/**
 * @file motor_foc.c
 * @brief Voltage-mode FOC motor control implementation
 *        Includes SVPWM, multi-loop control paths, zero-point calibration, and test modes.
 */
#include "motor_foc.h"
#include "arm_math.h"
#include "global_def.h"
#include "motor_runtime_param.h"
#include <stdbool.h>
#include <math.h>
#include "algorithm/filter.h"

#define rad60 deg2rad(60)
#define SQRT3 1.73205080756887729353

/* ---- PID controller instances (CMSIS-DSP) ---- */
static arm_pid_instance_f32 pid_position;
static arm_pid_instance_f32 pid_speed;
static arm_pid_instance_f32 pid_torque_d;
static arm_pid_instance_f32 pid_torque_q;

/* Default motor control context (no control active) */
motor_control_context_t motor_control_context = {
    .type = control_type_null,
    .position = 0.0f,
    .speed = 0.0f,
    .torque_norm_d = 0.0f,
    .torque_norm_q = 0.0f,
    .max_speed = 0.0f,
    .max_torque_norm = 0.0f,
};

/* ===========================================================================
 * SVPWM (Space Vector PWM)
 * ---------------------------------------------------------------------------
 * Performs inverse Park transform (dq -> alpha-beta) then generates three-phase
 * duty cycles using mid-point clamped SVPWM.
 *   phi   : electrical angle (rad)
 *   d, q  : dq-axis voltage commands (-1 ~ +1)
 *   d_u/v/w : output duty cycles (0 ~ 1)
 * ===========================================================================*/
static void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w) {
    d = _limit(d, -1.0f, 1.0f);
    q = _limit(q, -1.0f, 1.0f);

    float sin_phi, cos_phi;
    sin_phi = arm_sin_f32(phi);
    cos_phi = arm_cos_f32(phi);

    /* Inverse Park transform: dq -> alpha-beta */
    float alpha = d * cos_phi - q * sin_phi;
    float beta  = d * sin_phi + q * cos_phi;

    /* Inverse Clarke transform: alpha-beta -> UVW */
    float Ua = alpha;
    float Ub = -0.5f * alpha + SQRT3 / 2 * beta;
    float Uc = -0.5f * alpha - SQRT3 / 2 * beta;

    /* Mid-point clamped modulation */
    float Umax = Ua;
    float Umin = Ua;
    if (Ub > Umax) Umax = Ub;
    if (Uc > Umax) Umax = Uc;
    if (Ub < Umin) Umin = Ub;
    if (Uc < Umin) Umin = Uc;

    float U_zero = -0.5f * (Umax + Umin);

    /* Shift to 0~1 duty range */
    *d_u = Ua + U_zero + 0.5f;
    *d_v = Ub + U_zero + 0.5f;
    *d_w = Uc + U_zero + 0.5f;
}

/* Weak default: overridden by the real set_pwm_duty() in main.c */
void set_pwm_duty(float d_u, float d_v, float d_w) __attribute__((weak));
void set_pwm_duty(float d_u, float d_v, float d_w) {
    while(1)
        ;
}

/**
 * @brief  FOC forward path: dq voltage -> SVPWM -> PWM output
 * @param  d         d-axis voltage command (-1~1)
 * @param  q         q-axis voltage command (-1~1)
 * @param  rotor_rad electrical angle (rad)
 */
void foc_forward(float d, float q, float rotor_rad) {
    float d_u = 0;
    float d_v = 0;
    float d_w = 0;
    svpwm(rotor_rad, d, q, &d_u, &d_v, &d_w);
    set_pwm_duty(d_u, d_v, d_w);
}

/**
 * @brief  Compute cyclic difference with wrap-around
 * @param  diff   Raw difference
 * @param  cycle  Full cycle range (e.g. 2*PI)
 * @retval Wrapped difference in [-cycle/2, cycle/2]
 */
float cycle_diff(float diff, float cycle) {
    if (diff > (cycle / 2))
        diff -= cycle;
    else if (diff < (-cycle / 2))
        diff += cycle;
    return diff;
}

/* ===========================================================================
 * Control Loops
 * ===========================================================================*/

/** @brief Position PID loop: error = target - current mechanical angle */
static float position_loop(float rad) {
    float diff = cycle_diff(rad - motor_mech_angle, 2 * PI);
    return arm_pid_f32(&pid_position, diff);
}

/** @brief Position control: compute q-axis command from position error */
void lib_position_control(float rad) {
    float d = 0;
    float q = position_loop(rad);
    foc_forward(d, q, motor_logic_angle);
}

/** @brief Speed PID loop: error = target_speed - current_speed */
static float speed_loop(float speed_rad) {
    float diff = speed_rad - motor_speed;
    return arm_pid_f32(&pid_speed, diff);
}

/** @brief Speed control with output slew-rate limiting */
void lib_speed_control(float speed) {
    static float q_cmd = 0.0f;
    float d = 0;
    float q = speed_loop(speed);

    /* Slew-rate limit: max delta per control step */
    float dq_max = 0.0012f;
    if (q - q_cmd > dq_max) q = q_cmd + dq_max;
    if (q - q_cmd < -dq_max) q = q_cmd - dq_max;

    q_cmd = q;
    foc_forward(d, q_cmd, motor_logic_angle);
}

/** @brief d-axis torque loop (pass-through, no current feedback yet) */
static float torque_d_loop(float d) {
    return d;
}

/** @brief q-axis torque loop (pass-through, no current feedback yet) */
static float torque_q_loop(float q) {
    return q;
}

/** @brief Direct voltage-command torque control */
void lib_torque_control(float torque_norm_d, float torque_norm_q) {
    float d = torque_d_loop(torque_norm_d);
    float q = torque_q_loop(torque_norm_q);
    foc_forward(d, q, motor_logic_angle);
}

/** @brief Speed-torque cascade: speed loop output clamped by max_torque_norm */
void lib_speed_torque_control(float speed_rad, float max_torque_norm) {
    static float torque_cmd = 0.0f;
    static float speed_ref_f = 0.0f;
    float torque_norm;
    float speed_err;
    float dq_max = 0.0014f;

    if (!isfinite(speed_rad) || !isfinite(max_torque_norm)) {
        torque_cmd = 0.0f;
        lib_torque_control(0, 0);
        return;
    }

    max_torque_norm = fabsf(max_torque_norm);
    if (max_torque_norm > 1.0f) {
        max_torque_norm = 1.0f;
    }

    speed_ref_f = low_pass_filter(speed_rad, speed_ref_f, 0.20f);
    speed_err = speed_ref_f - motor_speed;
    torque_norm = speed_loop(speed_ref_f);

    if (fabsf(speed_err) < 0.08f && fabsf(speed_ref_f) < 0.35f) {
        torque_norm = 0.0f;
    }
    if (speed_ref_f > 0.4f && torque_norm < 0.0f && speed_err > -0.30f) {
        torque_norm = 0.0f;
    }
    if (speed_ref_f < -0.4f && torque_norm > 0.0f && speed_err < 0.30f) {
        torque_norm = 0.0f;
    }

    torque_norm = min(fabsf(torque_norm), max_torque_norm) * (torque_norm > 0 ? 1 : -1);

    if (torque_norm - torque_cmd > dq_max) torque_norm = torque_cmd + dq_max;
    if (torque_norm - torque_cmd < -dq_max) torque_norm = torque_cmd - dq_max;

    torque_cmd = torque_norm;
    lib_torque_control(0, torque_cmd);
}

/** @brief Position-speed-torque cascade: position -> speed -> torque */
void lib_position_speed_torque_control(float position_rad, float max_speed_rad, float max_torque_norm) {
    float speed_rad;
    if (!isfinite(position_rad) || !isfinite(max_speed_rad) || !isfinite(max_torque_norm)) {
        lib_torque_control(0, 0);
        return;
    }

    max_speed_rad = fabsf(max_speed_rad);
    speed_rad = position_loop(position_rad);
    if (!isfinite(speed_rad)) {
        speed_rad = 0.0f;
    }
    speed_rad = min(fabsf(speed_rad), max_speed_rad) * (speed_rad > 0 ? 1 : -1);
    lib_speed_torque_control(speed_rad, max_torque_norm);
}

/**
 * @brief  Set PID parameters for all four control loops and re-initialize
 */
void set_motor_pid(float position_p, float position_i, float position_d,
                   float speed_p, float speed_i, float speed_d,
                   float torque_d_p, float torque_d_i, float torque_d_d,
                   float torque_q_p, float torque_q_i, float torque_q_d) {
    pid_position.Kp = position_p;
    pid_position.Ki = position_i;
    pid_position.Kd = position_d;

    pid_speed.Kp = speed_p;
    pid_speed.Ki = speed_i;
    pid_speed.Kd = speed_d;

    pid_torque_d.Kp = torque_d_p;
    pid_torque_d.Ki = torque_d_i;
    pid_torque_d.Kd = torque_d_d;

    pid_torque_q.Kp = torque_q_p;
    pid_torque_q.Ki = torque_q_i;
    pid_torque_q.Kd = torque_q_d;

    arm_pid_init_f32(&pid_position, false);
    arm_pid_init_f32(&pid_speed, false);
    arm_pid_init_f32(&pid_torque_d, false);
    arm_pid_init_f32(&pid_torque_q, false);
}

#include "tim.h"

/** @brief Release motor: set all PWM channels to 0 (motor free-wheels) */
void motor_release(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
}

volatile zero_cali_state_t zero_cali_state = CALI_IDLE;

void motor_control_set_context(const motor_control_context_t *ctx)
{
    if (ctx == 0) return;
    __disable_irq();
    motor_control_context = *ctx;
    __enable_irq();
}

void motor_control_get_context(motor_control_context_t *ctx_out)
{
    if (ctx_out == 0) return;
    __disable_irq();
    *ctx_out = motor_control_context;
    __enable_irq();
}

void motor_control_estop(void)
{
    __disable_irq();
    motor_control_context.type = control_type_null;
    __enable_irq();
    motor_release();
}

/* ===========================================================================
 * motor_control_update()  --  Called from TIM3 ISR at motor_speed_calc_freq Hz
 * ===========================================================================*/
void motor_control_update(void) {
    /* --------------------------------------------------------
     * 1. Angle data synchronization
     * -------------------------------------------------------- */

    static float last_angle = 0.0f;
    static float speed_flit = 0.0f;
    static uint32_t last_sample_count = 0;

    const float control_dt = 1.0f / motor_speed_calc_freq;

    uint32_t sample_count = encoder_sample_count;
    if (sample_count != last_sample_count) {
        uint32_t sample_step = sample_count - last_sample_count;
        if (sample_step == 0u) sample_step = 1u;
        float delta = cycle_diff(encoder_angle - last_angle, 2.0f * PI);
        float raw_speed = delta / (control_dt * (float)sample_step);
        raw_speed = _limit(raw_speed, -80.0f, 80.0f);
        speed_flit = low_pass_filter(raw_speed, speed_flit, 0.18f);
        last_angle = encoder_angle;
        last_sample_count = sample_count;
    }
    motor_speed = speed_flit;

    /* --------------------------------------------------------
     * 2. Compute electrical angle (requires rotor_zero_angle)
     * -------------------------------------------------------- */
    /* Mechanical angle = encoder_angle - zero offset, wrapped to [0, 2PI] */
    float mech_angle = cycle_diff(encoder_angle - rotor_zero_angle, 2.0f * PI);
    while(mech_angle < 0) mech_angle += 2*PI;
    while(mech_angle > 2*PI) mech_angle -= 2*PI;

    motor_mech_angle = mech_angle;

    /* Electrical angle = mechanical angle * pole pairs, wrapped to [0, 2PI] */
    motor_logic_angle = mech_angle * POLAR_PAIRS;
    while(motor_logic_angle < 0) motor_logic_angle += 2*PI;
    while(motor_logic_angle > 2*PI) motor_logic_angle -= 2*PI;

    /* --------------------------------------------------------
     * Zero-point calibration state machine (test_mode == 99)
     * -------------------------------------------------------- */
    if (test_mode == 99) {
        /* IMPORTANT: must use static, otherwise counters reset on every entry */
        static uint32_t cali_tick = 0;
        static int zero_cnt = 0;
        static float zero_sum = 0.0f;

        /* If state was reset externally (e.g. re-entering from another mode), reset counters */
        if (zero_cali_state == CALI_LOCKING && cali_tick == 0 && zero_cnt != 0) {
             zero_cnt = 0;
             zero_sum = 0.0f;
        }

        switch ((int)zero_cali_state) {
            case (zero_cali_state_t)CALI_LOCKING:
                /* Apply d-axis voltage to lock rotor at electrical angle 0 */
                foc_forward(0.15f, 0.0f, 0.0f);
                if (++cali_tick > 2000) {
                    cali_tick = 0;
                    zero_sum = 0;
                    zero_cnt = 0;
                    zero_cali_state = CALI_SAMPLING;
                }
                break;

            case (zero_cali_state_t)CALI_SAMPLING:
                /* Keep locking, accumulate encoder readings */
                foc_forward(0.15f, 0.0f, 0.0f);
                zero_sum += encoder_angle;
                zero_cnt++;
                if (zero_cnt >= 200) {
                    /* Average the samples to get zero angle */
                    rotor_zero_angle = zero_sum / zero_cnt;

                    /* Wrap zero angle to [0, 2PI] */
                    while(rotor_zero_angle < 0.0f) rotor_zero_angle += 2.0f*PI;
                    while(rotor_zero_angle > 2.0f*PI) rotor_zero_angle -= 2.0f*PI;

                    zero_cali_state = CALI_DONE;
                }
                break;

            case (zero_cali_state_t)CALI_DONE:
                /* Calibration done, set 50% duty (motor idle) */
                set_pwm_duty(0.5f, 0.5f, 0.5f);
                break;

            default:
                zero_cali_state = CALI_IDLE;
                cali_tick = 0;  /* Reset on unexpected state */
                break;
        }
        return;
    }

    /* --------------------------------------------------------
     * 3. Control mode state machine
     * -------------------------------------------------------- */
    if (test_mode == 1) {
        /* Open-loop velocity test: ignore encoder, self-generate rotating field
         * Verifies SVPWM and PWM hardware are working correctly */
        static float open_loop_angle = 0.0f;
        open_loop_angle += 0.01f;   /* Adjust this value to change speed */
        if(open_loop_angle > 2*PI) open_loop_angle -= 2*PI;
        /* d=0, q=0.6 (voltage space vector ~60%) */
        foc_forward(0.0f, 0.6f, open_loop_angle);
        return;
    }

    switch (motor_control_context.type)
    {
    case control_type_position:
        lib_position_control(motor_control_context.position);
        break;
    case control_type_speed:
        lib_speed_control(motor_control_context.speed);
        break;
    case control_type_speed_torque:
        lib_speed_torque_control(motor_control_context.speed, motor_control_context.max_torque_norm);
        break;
    case control_type_torque:
        lib_torque_control(motor_control_context.torque_norm_d, motor_control_context.torque_norm_q);
        break;
    case control_type_position_speed_torque:
        lib_position_speed_torque_control(
            motor_control_context.position, 
            motor_control_context.max_speed,
            motor_control_context.max_torque_norm
        );
        break;
    default:
        motor_release();
        break;
    }
}
