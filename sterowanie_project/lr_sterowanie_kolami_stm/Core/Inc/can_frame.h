/**
  * @file    can_frame.h
  * @brief   HAL-free CAN-FD frame type shared by the driver and the protocol.
  */
#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdint.h>
#include "app_config.h"

typedef struct
{
  uint32_t id;                        /* 11-bit standard identifier */
  uint8_t  len;                       /* payload length in bytes    */
  uint8_t  data[CAN_FRAME_MAX_LEN];
} can_frame_t;

#endif /* CAN_FRAME_H */
