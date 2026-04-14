/**
 * @file conf.h
 * @brief Motor configuration parameters (pole pairs, PWM freq, control freq, etc.)
 */
#pragma once
#include "global_def.h"

#define POLAR_PAIRS            7            /* Number of motor pole pairs */

#define motor_pwm_freq         40000        /* Driver bridge PWM frequency (Hz) */
#define motor_speed_calc_freq  500          /* Motor speed calculation frequency (Hz) */

/* Software position cycle range for position mode control.
 * Equals +/- position_cycles. Can be modified freely, e.g. 1234*PI */
#define position_cycles        (6 * PI)
