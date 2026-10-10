#include "test.h"

int test_failures;
int test_checks;

int main(void)
{
  test_can_proto();
  test_kinematics();
  test_pi_ctrl();
  test_drive_ctrl();
  test_encoder_math();
  test_cmd_timeout();
  test_ring_buf();
  printf("%d checks, %d failures\n", test_checks, test_failures);
  return (test_failures == 0) ? 0 : 1;
}
