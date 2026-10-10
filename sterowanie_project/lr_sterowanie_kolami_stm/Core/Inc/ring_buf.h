/**
  * @file    ring_buf.h
  * @brief   Lock-free single-producer/single-consumer ring of CAN frames.
  *          Producer: FDCAN RX ISR. Consumer: main loop.
  */
#ifndef RING_BUF_H
#define RING_BUF_H

#include <stdbool.h>
#include <stdint.h>
#include "can_frame.h"

typedef struct
{
  can_frame_t slots[CAN_RX_RING_CAPACITY];
  uint32_t    head;       /* written by the producer only */
  uint32_t    tail;       /* written by the consumer only */
  uint32_t    overflows;  /* frames dropped because the ring was full */
} ring_buf_t;

void ring_buf_init(ring_buf_t *rb);
bool ring_buf_push(ring_buf_t *rb, const can_frame_t *frame);
bool ring_buf_pop(ring_buf_t *rb, can_frame_t *frame);

#endif /* RING_BUF_H */
