/**
  * @file    kinematics.c
  * @brief   v_right = x + z*W/2, v_left = x - z*W/2, then per-wheel flip.
  */
#include "kinematics.h"
#include "app_config.h"

#define WHEEL_COUNT 4U

static const uint8_t is_right[WHEEL_COUNT] = WHEEL_IS_RIGHT_SIDE;
static const float   dir_flip[WHEEL_COUNT] = WHEEL_DIR_FLIP;

float kinematics_wheel_velocity(uint8_t wheel_no, float lin_x, float ang_z)
{
  if ((wheel_no < 1U) || (wheel_no > WHEEL_COUNT))
  {
    return 0.0f;
  }
  float half_turn = ang_z * ROVER_WIDTH_M * 0.5f;
  float v = (is_right[wheel_no - 1U] != 0U) ? (lin_x + half_turn) : (lin_x - half_turn);

  return v * dir_flip[wheel_no - 1U];
}
