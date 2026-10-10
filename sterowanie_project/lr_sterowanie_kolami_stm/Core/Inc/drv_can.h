/**
  * @file    drv_can.h
  * @brief   FDCAN1: filters, RX ISR to ring buffer, TX, bus-off recovery.
  */
#ifndef DRV_CAN_H
#define DRV_CAN_H

#include <stdbool.h>
#include <stdint.h>
#include "can_frame.h"

void drv_can_init(void);
bool drv_can_receive(can_frame_t *frame);
bool drv_can_send(const can_frame_t *frame);
/* Runs the bus-off recovery. Returns true once per detected bus-off. */
bool drv_can_service(uint32_t now_ms);
/* False once bus-off recovery has used all its retries. */
bool drv_can_is_healthy(void);

#endif /* DRV_CAN_H */
