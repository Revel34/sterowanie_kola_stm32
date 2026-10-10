#include "pi_ctrl.h"
#include "test.h"

static pi_ctrl_t make(float kp, float ki, float i_lim, float out_lim)
{
  pi_ctrl_t pi;
  pi_ctrl_params_t p = { kp, ki, 0.0f, i_lim, out_lim, 0.01f };

  pi_ctrl_init(&pi, &p);
  return pi;
}

void test_pi_ctrl(void)
{
  pi_ctrl_t pi = make(2.0f, 3.0f, 1.0f, 1000.0f);

  /* P + I. */
  CHECK_NEAR(pi_ctrl_step(&pi, 0.5f, 0.0f, 0.1f), 2.0 * 0.5 + 3.0 * 0.05, 1e-6);
  CHECK_NEAR(pi.integral, 0.05, 1e-7);

  /* Zero setpoint resets state and outputs 0. */
  CHECK_NEAR(pi_ctrl_step(&pi, 0.005f, 0.3f, 0.1f), 0.0, 0.0);
  CHECK_NEAR(pi.integral, 0.0, 0.0);
  CHECK_NEAR(pi.prev_error, 0.0, 0.0);

  /* Output saturation. */
  pi = make(32.629f, 995.281f, 1.0f, 1000.0f);
  CHECK_NEAR(pi_ctrl_step(&pi, 50.0f, 0.0f, 0.1f), 1000.0, 0.0);
  pi_ctrl_reset(&pi);
  CHECK_NEAR(pi_ctrl_step(&pi, -50.0f, 0.0f, 0.1f), -1000.0, 0.0);

  /* Anti-windup: integration frozen while the previous output is saturated. */
  pi = make(100.0f, 1.0f, 10.0f, 1000.0f);
  pi_ctrl_step(&pi, 20.0f, 0.0f, 0.1f);           /* prev output 0: integrates once */
  CHECK_NEAR(pi.integral, 2.0, 1e-6);
  CHECK_NEAR(pi.output, 1000.0, 0.0);
  pi_ctrl_step(&pi, 20.0f, 0.0f, 0.1f);
  pi_ctrl_step(&pi, 20.0f, 0.0f, 0.1f);
  CHECK_NEAR(pi.integral, 2.0, 1e-6);
  pi_ctrl_step(&pi, 1.0f, 0.5f, 0.1f);            /* leaves saturation, output 50 + 2.0 */
  CHECK_NEAR(pi.output, 52.0, 1e-4);
  pi_ctrl_step(&pi, 1.0f, 0.5f, 0.1f);            /* integrates again */
  CHECK_NEAR(pi.integral, 2.05, 1e-5);

  /* Integral clamp. */
  pi = make(0.0f, 1.0f, 1.0f, 1000.0f);
  for (int i = 0; i < 5; i++)
  {
    pi_ctrl_step(&pi, 10.0f, 0.0f, 1.0f);
  }
  CHECK_NEAR(pi.integral, 1.0, 0.0);
  CHECK_NEAR(pi.output, 1.0, 0.0);
  for (int i = 0; i < 5; i++)
  {
    pi_ctrl_step(&pi, -10.0f, 0.0f, 1.0f);
  }
  CHECK_NEAR(pi.integral, -1.0, 0.0);

  /* Non-positive dt keeps the previous output. */
  CHECK_NEAR(pi_ctrl_step(&pi, -10.0f, 0.0f, 0.0f), -1.0, 0.0);
}
