/**
 * @file filter.h
 * @brief Digital filter algorithms
 */
#pragma once

/**
 * @brief  First-order low-pass filter (IIR)
 * @param  input        New input sample
 * @param  last_output  Previous filter output
 * @param  alpha        Filter coefficient (0~1, higher = faster response, more noise)
 * @retval Filtered output
 */
float low_pass_filter(float input, float last_output, float alpha);
