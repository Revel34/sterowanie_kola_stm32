/**
  * @file    encoder_math.c
  * @brief   Deltas use the counter's native width, so a wrap of the hardware
  *          counter yields the correct signed step without special cases.
  */
#include "encoder_math.h"
#include "app_config.h"

#define TWO_PI              6.283185307179586
#define COUNTS_PER_WHEEL_REV (ENC_COUNTS_PER_MOTOR_REV * GEAR_RATIO)

int32_t encoder_math_delta16(uint16_t now, uint16_t prev)
{
  return (int16_t)(uint16_t)(now - prev);
}

int32_t encoder_math_delta32(uint32_t now, uint32_t prev)
{
  return (int32_t)(now - prev);
}

double encoder_math_counts_to_rad(int64_t counts)
{
  return (double)counts * TWO_PI / (double)COUNTS_PER_WHEEL_REV;
}

float encoder_math_rad_per_s(int32_t delta, float dt_s)
{
  if (dt_s <= 0.0f)
  {
    return 0.0f;
  }
  return (float)delta * (float)TWO_PI / (COUNTS_PER_WHEEL_REV * dt_s);
}
