/**
  * @file    kinematics.h
  * @brief   Differential kinematics for 4 fixed wheels. HAL-free.
  */
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

/* Target linear velocity [m/s] of wheel 1..4 in the wheel's own rotation
 * sense (direction flip applied). Returns 0 for an invalid wheel number. */
float kinematics_wheel_velocity(uint8_t wheel_no, float lin_x, float ang_z);

#endif /* KINEMATICS_H */
