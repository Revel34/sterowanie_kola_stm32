#include "app_config.h"
#include "kinematics.h"
#include "test.h"

void test_kinematics(void)
{
  const double half_w = (double)ROVER_WIDTH_M / 2.0;

  /* Straight: right wheels flipped. */
  CHECK_NEAR(kinematics_wheel_velocity(1, 1.0f, 0.0f), -1.0, 1e-6);
  CHECK_NEAR(kinematics_wheel_velocity(2, 1.0f, 0.0f), -1.0, 1e-6);
  CHECK_NEAR(kinematics_wheel_velocity(3, 1.0f, 0.0f), 1.0, 1e-6);
  CHECK_NEAR(kinematics_wheel_velocity(4, 1.0f, 0.0f), 1.0, 1e-6);

  /* Spin in place: v_right = +z*W/2, v_left = -z*W/2, then flip. */
  CHECK_NEAR(kinematics_wheel_velocity(1, 0.0f, 1.0f), -half_w, 1e-6);
  CHECK_NEAR(kinematics_wheel_velocity(3, 0.0f, 1.0f), -half_w, 1e-6);

  /* Arc. */
  CHECK_NEAR(kinematics_wheel_velocity(2, 1.0f, 2.0f), -(1.0 + 2.0 * half_w), 1e-6);
  CHECK_NEAR(kinematics_wheel_velocity(4, 1.0f, 2.0f), 1.0 - 2.0 * half_w, 1e-6);

  /* Invalid wheel numbers. */
  CHECK_NEAR(kinematics_wheel_velocity(0, 1.0f, 1.0f), 0.0, 0.0);
  CHECK_NEAR(kinematics_wheel_velocity(5, 1.0f, 1.0f), 0.0, 0.0);
}
