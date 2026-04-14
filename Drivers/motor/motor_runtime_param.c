/**
 * @file motor_runtime_param.c
 * @brief Motor runtime parameter variable definitions
 */
#include "motor_runtime_param.h"

float motor_i_u;                                /* Phase U current */
float motor_i_v;                                /* Phase V current */
float motor_i_d;                                /* d-axis current */
float motor_i_q;                                /* q-axis current */
float motor_speed;                              /* Motor speed (rad/s) */
float motor_logic_angle;                        /* Electrical angle */
float motor_mech_angle;                         /* Mechanical angle (0~2PI) */
volatile float encoder_angle;                   /* Raw encoder angle */
float rotor_zero_angle;                         /* Zero-point encoder angle (d-axis aligned) */
volatile float encoder_angle_cache = 0.0f;      /* Encoder angle cache for ISR handoff */
volatile int test_mode;                         /* Test/control mode selector */
volatile int encoder_sample_req = 0;            /* Encoder sample request flag */
volatile int encoder_sample_ready = 0;          /* Encoder sample ready flag */
volatile uint32_t encoder_sample_count = 0;
