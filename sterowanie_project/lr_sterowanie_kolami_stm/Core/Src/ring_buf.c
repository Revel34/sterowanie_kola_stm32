/**
  * @file    ring_buf.c
  * @brief   Lock-free SPSC ring. Indices run free and are masked on access;
  *          acquire/release ordering publishes the slot before the index.
  */
#include "ring_buf.h"

_Static_assert((CAN_RX_RING_CAPACITY & (CAN_RX_RING_CAPACITY - 1U)) == 0U,
               "CAN_RX_RING_CAPACITY must be a power of 2");

#define RING_MASK (CAN_RX_RING_CAPACITY - 1U)

void ring_buf_init(ring_buf_t *rb)
{
  rb->head = 0U;
  rb->tail = 0U;
  rb->overflows = 0U;
}

bool ring_buf_push(ring_buf_t *rb, const can_frame_t *frame)
{
  uint32_t head = rb->head;
  uint32_t tail = __atomic_load_n(&rb->tail, __ATOMIC_ACQUIRE);

  if ((head - tail) >= CAN_RX_RING_CAPACITY)
  {
    rb->overflows++;
    return false;
  }
  rb->slots[head & RING_MASK] = *frame;
  __atomic_store_n(&rb->head, head + 1U, __ATOMIC_RELEASE);
  return true;
}

bool ring_buf_pop(ring_buf_t *rb, can_frame_t *frame)
{
  uint32_t tail = rb->tail;
  uint32_t head = __atomic_load_n(&rb->head, __ATOMIC_ACQUIRE);

  if (head == tail)
  {
    return false;
  }
  *frame = rb->slots[tail & RING_MASK];
  __atomic_store_n(&rb->tail, tail + 1U, __ATOMIC_RELEASE);
  return true;
}
