#include "cmd_timeout.h"
#include "test.h"

void test_cmd_timeout(void)
{
  cmd_timeout_t t;

  cmd_timeout_init(&t, 2500U);
  CHECK(!cmd_timeout_is_active(&t));
  cmd_timeout_update(&t, 10000U);                  /* no command yet: stays waiting */
  CHECK_EQ(t.state, CMD_STATE_WAITING);

  cmd_timeout_feed(&t, 100U);
  CHECK(cmd_timeout_is_active(&t));
  cmd_timeout_update(&t, 100U + 2499U);
  CHECK(cmd_timeout_is_active(&t));
  cmd_timeout_update(&t, 100U + 2500U);            /* expiry */
  CHECK(!cmd_timeout_is_active(&t));
  CHECK_EQ(t.state, CMD_STATE_TIMED_OUT);

  cmd_timeout_feed(&t, 5000U);                     /* auto-resume */
  CHECK(cmd_timeout_is_active(&t));

  cmd_timeout_expire(&t);                          /* forced (bus-off) */
  CHECK_EQ(t.state, CMD_STATE_TIMED_OUT);
  cmd_timeout_feed(&t, 5100U);
  CHECK(cmd_timeout_is_active(&t));

  cmd_timeout_init(&t, 2500U);                     /* expire before any command stays waiting */
  cmd_timeout_expire(&t);
  CHECK_EQ(t.state, CMD_STATE_WAITING);

  cmd_timeout_feed(&t, 0xFFFFFF00U);               /* millisecond counter wrap */
  cmd_timeout_update(&t, 0x00000100U);
  CHECK(cmd_timeout_is_active(&t));
  cmd_timeout_update(&t, 0x00000100U + 2500U);
  CHECK(!cmd_timeout_is_active(&t));
}
