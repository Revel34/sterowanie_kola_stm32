/**
  * @file    encoder_math.h
  * @brief   Wrap-safe encoder deltas and count-to-wheel-unit conversion. HAL-free.
  */
#ifndef ENCODER_MATH_H
#define ENCODER_MATH_H

#include <stdint.h>

int32_t encoder_math_delta16(uint16_t now, uint16_t prev);
int32_t encoder_math_delta32(uint32_t now, uint32_t prev);
double  encoder_math_counts_to_rad(int64_t counts);
float   encoder_math_rad_per_s(int32_t delta, float dt_s);

#endif /* ENCODER_MATH_H */
