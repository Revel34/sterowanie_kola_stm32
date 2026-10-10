#include <string.h>
#include "ring_buf.h"
#include "test.h"

static can_frame_t frame(uint32_t id)
{
  can_frame_t f;

  memset(&f, 0, sizeof(f));
  f.id = id;
  f.len = 8;
  f.data[0] = (uint8_t)id;
  f.data[CAN_FRAME_MAX_LEN - 1U] = (uint8_t)(id >> 8);
  return f;
}

static ring_buf_t rb;

void test_ring_buf(void)
{
  can_frame_t in, out;

  ring_buf_init(&rb);
  CHECK(!ring_buf_pop(&rb, &out));

  /* FIFO order. */
  in = frame(1); CHECK(ring_buf_push(&rb, &in));
  in = frame(2); CHECK(ring_buf_push(&rb, &in));
  CHECK(ring_buf_pop(&rb, &out)); CHECK_EQ(out.id, 1);
  CHECK(ring_buf_pop(&rb, &out)); CHECK_EQ(out.id, 2);
  CHECK(!ring_buf_pop(&rb, &out));

  /* Overflow: newest frame dropped and counted. */
  for (uint32_t i = 0U; i < CAN_RX_RING_CAPACITY; i++)
  {
    in = frame(100U + i);
    CHECK(ring_buf_push(&rb, &in));
  }
  in = frame(999U);
  CHECK(!ring_buf_push(&rb, &in));
  CHECK_EQ(rb.overflows, 1);
  for (uint32_t i = 0U; i < CAN_RX_RING_CAPACITY; i++)
  {
    CHECK(ring_buf_pop(&rb, &out));
    CHECK_EQ(out.id, 100U + i);
  }
  CHECK(!ring_buf_pop(&rb, &out));

  /* Slot wrap over many cycles, full payload copied. */
  for (uint32_t i = 0U; i < 5U * CAN_RX_RING_CAPACITY; i++)
  {
    in = frame(0x200U + i);
    CHECK(ring_buf_push(&rb, &in));
    CHECK(ring_buf_pop(&rb, &out));
    CHECK_EQ(memcmp(&in, &out, sizeof(in)), 0);
  }

  /* Index wrap across 2^32. */
  rb.head = 0xFFFFFFFEU;
  rb.tail = 0xFFFFFFFEU;
  for (uint32_t i = 0U; i < 4U; i++)
  {
    in = frame(0x300U + i);
    CHECK(ring_buf_push(&rb, &in));
  }
  for (uint32_t i = 0U; i < 4U; i++)
  {
    CHECK(ring_buf_pop(&rb, &out));
    CHECK_EQ(out.id, 0x300U + i);
  }
  CHECK(!ring_buf_pop(&rb, &out));
}
