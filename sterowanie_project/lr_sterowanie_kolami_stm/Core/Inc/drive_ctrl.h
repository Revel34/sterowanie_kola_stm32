/**
  * @file    drive_ctrl.h
  * @brief   Per-wheel motor signal for the selected DRIVE mode. HAL-free.
  */
#ifndef DRIVE_CTRL_H
#define DRIVE_CTRL_H

#include "can_proto.h"
#include "pi_ctrl.h"

/* Signal in [-MOTOR_SIGNAL_FULL_SCALE, MOTOR_SIGNAL_FULL_SCALE].
 * target and measured in m/s. The PI state is reset outside PID mode. */
float drive_ctrl_signal(drive_mode_t mode, pi_ctrl_t *pi, float target_m_s,
                        float measured_m_s, float dt_s);

#endif /* DRIVE_CTRL_H */
