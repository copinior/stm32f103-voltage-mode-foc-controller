/**
 * @file motor_runtime_param.h
 * @brief Motor runtime parameters (angles, currents, speed, encoder state)
 */
#pragma once
#include <stdint.h>
#include "conf.h"

/* Rotor physical angle = encoder_angle - rotor_zero_angle */
#define rotor_phy_angle      (encoder_angle - rotor_zero_angle)

/* Rotor electrical angle = physical angle * pole pairs */
#define rotor_logic_angle    rotor_phy_angle * POLAR_PAIRS

extern float motor_i_u;                             /* Phase U current */
extern float motor_i_v;                             /* Phase V current */
extern float motor_i_d;                             /* d-axis current (Park transform) */
extern float motor_i_q;                             /* q-axis current (Park transform) */
extern float motor_speed;                           /* Motor speed (rad/s) */
extern float motor_logic_angle;                     /* Motor electrical angle (multi-pole) */
extern float motor_mech_angle;                      /* Mechanical angle (0~2PI) */
extern volatile float encoder_angle;                /* Raw angle read from encoder */
extern float rotor_zero_angle;                      /* Encoder angle when rotor d-axis aligns with coil d-axis */
extern volatile float encoder_angle_cache;          /* Encoder angle cache (written in main loop, read in ISR) */
extern volatile int test_mode;                      /* Current test/control mode selector */
extern volatile int encoder_sample_req;             /* Flag: main loop should sample encoder */
extern volatile int encoder_sample_ready;           /* Flag: encoder cache has new data ready */
extern volatile uint32_t encoder_sample_count;
