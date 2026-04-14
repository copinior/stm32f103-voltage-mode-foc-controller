/**
 * @file filter.c
 * @brief Digital filter algorithm implementations
 */
#include "filter.h"

/**
 * @brief  First-order low-pass filter: out = alpha * input + (1 - alpha) * last_output
 */
float low_pass_filter(float input, float last_output, float alpha) {
    return alpha * input + (1.0f - alpha) * last_output;
}
