/**
  * @file    cmd_timeout.h
  * @brief   Command-stream watchdog. Outputs are allowed only while ACTIVE.
  *          HAL-free.
  */
#ifndef CMD_TIMEOUT_H
#define CMD_TIMEOUT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  CMD_STATE_WAITING = 0,  /* no valid command since boot */
  CMD_STATE_ACTIVE,
  CMD_STATE_TIMED_OUT     /* timeout or forced stop; resumes on next valid frame */
} cmd_state_t;

typedef struct
{
  cmd_state_t state;
  uint32_t    last_ms;
  uint32_t    timeout_ms;
} cmd_timeout_t;

void cmd_timeout_init(cmd_timeout_t *t, uint32_t timeout_ms);
void cmd_timeout_feed(cmd_timeout_t *t, uint32_t now_ms);
void cmd_timeout_update(cmd_timeout_t *t, uint32_t now_ms);
void cmd_timeout_expire(cmd_timeout_t *t);
bool cmd_timeout_is_active(const cmd_timeout_t *t);

#endif /* CMD_TIMEOUT_H */
