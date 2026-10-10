/**
  * @file    can_proto.h
  * @brief   DRIVE command decode and wheel feedback encode (legacy layout,
  *          little-endian). HAL-free.
  */
#ifndef CAN_PROTO_H
#define CAN_PROTO_H

#include <stdbool.h>
#include <stdint.h>
#include "can_frame.h"

typedef enum
{
  DRIVE_MODE_OPEN_LOOP = 0,
  DRIVE_MODE_PID       = 1,
  DRIVE_MODE_X_STOP    = 2
} drive_mode_t;

typedef struct
{
  float        lin_x;   /* m/s   */
  float        lin_y;   /* m/s, ignored by fixed wheels */
  float        ang_z;   /* rad/s */
  drive_mode_t mode;
} drive_cmd_t;

uint8_t can_dlc_to_len(uint8_t dlc);
uint8_t can_len_to_dlc(uint8_t len);

/* Returns false for a wrong ID, wrong length or unknown mode. */
bool can_proto_decode_drive_cmd(const can_frame_t *frame, drive_cmd_t *cmd);

void can_proto_encode_feedback(uint8_t wheel_no, double position_rad,
                               float velocity_rad_s, can_frame_t *frame);

#endif /* CAN_PROTO_H */
