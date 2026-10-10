/**
  * @file    pi_ctrl.h
  * @brief   PI(D) velocity controller with conditional-integration
  *          anti-windup (legacy semantics). HAL-free.
  */
#ifndef PI_CTRL_H
#define PI_CTRL_H

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral_limit;
  float output_limit;
  float zero_setpoint_eps;
} pi_ctrl_params_t;

typedef struct
{
  pi_ctrl_params_t p;
  float integral;   /* integral of the error [error unit * s] */
  float prev_error;
  float output;
} pi_ctrl_t;

void  pi_ctrl_init(pi_ctrl_t *pi, const pi_ctrl_params_t *params);
void  pi_ctrl_reset(pi_ctrl_t *pi);
float pi_ctrl_step(pi_ctrl_t *pi, float setpoint, float measured, float dt_s);

#endif /* PI_CTRL_H */
