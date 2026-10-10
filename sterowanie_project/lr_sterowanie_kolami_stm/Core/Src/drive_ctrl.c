/**
  * @file    drive_ctrl.c
  * @brief   Mode 0 open loop (setpoint x gain, clamped), mode 1 PI, mode 2 stop.
  */
#include "drive_ctrl.h"
#include "app_config.h"

static float clamp_signal(float x)
{
  if (x > MOTOR_SIGNAL_FULL_SCALE)
  {
    return MOTOR_SIGNAL_FULL_SCALE;
  }
  if (x < -MOTOR_SIGNAL_FULL_SCALE)
  {
    return -MOTOR_SIGNAL_FULL_SCALE;
  }
  return x;
}

float drive_ctrl_signal(drive_mode_t mode, pi_ctrl_t *pi, float target_m_s,
                        float measured_m_s, float dt_s)
{
  if (mode == DRIVE_MODE_PID)
  {
    return clamp_signal(pi_ctrl_step(pi, target_m_s, measured_m_s, dt_s));
  }
  pi_ctrl_reset(pi);
  if (mode == DRIVE_MODE_OPEN_LOOP)
  {
    return clamp_signal(target_m_s * OPEN_LOOP_GAIN);
  }
  return 0.0f;
}
