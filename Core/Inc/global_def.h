/**
 * @file global_def.h
 * @brief Global macro definitions (math constants and utility macros)
 */
#pragma once

#ifndef PI
#define PI 3.14159265358979
#endif

#define deg2rad(a)      (PI * (a) / 180)    /* Degree to radian */
#define rad2deg(a)      (180 * (a) / PI)    /* Radian to degree */
#define max(a, b)       ((a) > (b) ? (a) : (b))
#define min(a, b)       ((a) < (b) ? (a) : (b))
