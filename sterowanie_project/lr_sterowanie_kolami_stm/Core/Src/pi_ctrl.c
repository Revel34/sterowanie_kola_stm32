/**
  * @file    pi_ctrl.c
  * @brief   u = Kp*e + Ki*int(e) + Kd*de/dt, integral clamped, output clamped.
  *          The integral only accumulates while the previous output was not
  *          saturated. A zero setpoint resets the state and outputs 0.
  */
#include "pi_ctrl.h"

static float absf(float x)
{
  return (x < 0.0f) ? -x : x;
}

static float clampf(float x, float limit)
{
  if (x > limit)
  {
    return limit;
  }
  if (x < -limit)
  {
    return -limit;
  }
  return x;
}

void pi_ctrl_init(pi_ctrl_t *pi, const pi_ctrl_params_t *params)
{
  pi->p = *params;
  pi_ctrl_reset(pi);
}

void pi_ctrl_reset(pi_ctrl_t *pi)
{
  pi->integral = 0.0f;
  pi->prev_error = 0.0f;
  pi->output = 0.0f;
}

float pi_ctrl_step(pi_ctrl_t *pi, float setpoint, float measured, float dt_s)
{
  if (absf(setpoint) < pi->p.zero_setpoint_eps)
  {
    pi_ctrl_reset(pi);
    return 0.0f;
  }
  if (dt_s <= 0.0f)
  {
    return pi->output;
  }
  float error = setpoint - measured;

  if (absf(pi->output) < pi->p.output_limit)
  {
    pi->integral = clampf(pi->integral + error * dt_s, pi->p.integral_limit);
  }
  float derivative = (error - pi->prev_error) / dt_s;

  pi->prev_error = error;
  pi->output = clampf(pi->p.kp * error + pi->p.ki * pi->integral + pi->p.kd * derivative,
                      pi->p.output_limit);
  return pi->output;
}
