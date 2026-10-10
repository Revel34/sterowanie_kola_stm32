/**
  * @file    cmd_timeout.c
  * @brief   WAITING --feed--> ACTIVE --timeout/expire--> TIMED_OUT --feed--> ACTIVE
  */
#include "cmd_timeout.h"

void cmd_timeout_init(cmd_timeout_t *t, uint32_t timeout_ms)
{
  t->state = CMD_STATE_WAITING;
  t->last_ms = 0U;
  t->timeout_ms = timeout_ms;
}

void cmd_timeout_feed(cmd_timeout_t *t, uint32_t now_ms)
{
  t->state = CMD_STATE_ACTIVE;
  t->last_ms = now_ms;
}

void cmd_timeout_update(cmd_timeout_t *t, uint32_t now_ms)
{
  if ((t->state == CMD_STATE_ACTIVE) && ((now_ms - t->last_ms) >= t->timeout_ms))
  {
    t->state = CMD_STATE_TIMED_OUT;
  }
}

void cmd_timeout_expire(cmd_timeout_t *t)
{
  if (t->state == CMD_STATE_ACTIVE)
  {
    t->state = CMD_STATE_TIMED_OUT;
  }
}

bool cmd_timeout_is_active(const cmd_timeout_t *t)
{
  return t->state == CMD_STATE_ACTIVE;
}
