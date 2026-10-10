#include "app_config.h"
#include "drive_ctrl.h"
#include "test.h"

void test_drive_ctrl(void)
{
  pi_ctrl_t pi;
  pi_ctrl_params_t p = { 100.0f, 10.0f, 0.0f, 1.0f, PI_OUTPUT_LIMIT, 0.01f };

  pi_ctrl_init(&pi, &p);

  /* Open loop: legacy x330, clamped to full scale. */
  CHECK_NEAR(drive_ctrl_signal(DRIVE_MODE_OPEN_LOOP, &pi, 1.0f, 0.0f, 0.1f), OPEN_LOOP_GAIN, 1e-3);
  CHECK_NEAR(drive_ctrl_signal(DRIVE_MODE_OPEN_LOOP, &pi, 10.0f, 0.0f, 0.1f), MOTOR_SIGNAL_FULL_SCALE, 0.0);
  CHECK_NEAR(drive_ctrl_signal(DRIVE_MODE_OPEN_LOOP, &pi, -10.0f, 0.0f, 0.1f), -MOTOR_SIGNAL_FULL_SCALE, 0.0);

  /* PID delegates to the controller. */
  CHECK_NEAR(drive_ctrl_signal(DRIVE_MODE_PID, &pi, 1.0f, 0.5f, 0.1f), 100.0 * 0.5 + 10.0 * 0.05, 1e-4);
  CHECK(pi.integral != 0.0f);

  /* X_STOP: zero output and PI reset. */
  CHECK_NEAR(drive_ctrl_signal(DRIVE_MODE_X_STOP, &pi, 1.0f, 0.5f, 0.1f), 0.0, 0.0);
  CHECK_NEAR(pi.integral, 0.0, 0.0);
}
