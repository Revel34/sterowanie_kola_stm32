#include <string.h>
#include "can_proto.h"
#include "test.h"

static can_frame_t cmd_frame(int16_t x, int16_t y, int16_t mode, int16_t z)
{
  can_frame_t f;
  int16_t v[4] = { x, y, mode, z };

  memset(&f, 0, sizeof(f));
  f.id = CAN_ID_DRIVE_CMD;
  f.len = CAN_DRIVE_DLC;
  for (int i = 0; i < 4; i++)
  {
    f.data[2 * i] = (uint8_t)((uint16_t)v[i] & 0xFFU);
    f.data[2 * i + 1] = (uint8_t)((uint16_t)v[i] >> 8);
  }
  return f;
}

static void test_decode_endianness_and_sign(void)
{
  /* lin_x 1.234, lin_y -0.5, mode 1 (PID), ang_z -2.0; bytes spelled out. */
  can_frame_t f = { .id = CAN_ID_DRIVE_CMD, .len = 8,
                    .data = { 0xD2, 0x04, 0x0C, 0xFE, 0xE8, 0x03, 0x30, 0xF8 } };
  drive_cmd_t c;

  CHECK(can_proto_decode_drive_cmd(&f, &c));
  CHECK_NEAR(c.lin_x, 1.234, 1e-6);
  CHECK_NEAR(c.lin_y, -0.5, 1e-6);
  CHECK_NEAR(c.ang_z, -2.0, 1e-6);
  CHECK_EQ(c.mode, DRIVE_MODE_PID);

  f = cmd_frame(INT16_MIN, INT16_MAX, 0, -1);
  CHECK(can_proto_decode_drive_cmd(&f, &c));
  CHECK_NEAR(c.lin_x, -32.768, 1e-5);
  CHECK_NEAR(c.lin_y, 32.767, 1e-5);
  CHECK_NEAR(c.ang_z, -0.001, 1e-9);
}

static void test_decode_modes(void)
{
  drive_cmd_t c;
  can_frame_t f;

  f = cmd_frame(0, 0, 0, 0);    CHECK(can_proto_decode_drive_cmd(&f, &c)); CHECK_EQ(c.mode, DRIVE_MODE_OPEN_LOOP);
  f = cmd_frame(0, 0, -99, 0);  CHECK(can_proto_decode_drive_cmd(&f, &c)); CHECK_EQ(c.mode, DRIVE_MODE_OPEN_LOOP);
  f = cmd_frame(0, 0, 1099, 0); CHECK(can_proto_decode_drive_cmd(&f, &c)); CHECK_EQ(c.mode, DRIVE_MODE_PID);
  f = cmd_frame(0, 0, 2050, 0); CHECK(can_proto_decode_drive_cmd(&f, &c)); CHECK_EQ(c.mode, DRIVE_MODE_X_STOP);
  f = cmd_frame(0, 0, 1100, 0); CHECK(!can_proto_decode_drive_cmd(&f, &c));
  f = cmd_frame(0, 0, 500, 0);  CHECK(!can_proto_decode_drive_cmd(&f, &c));
  f = cmd_frame(0, 0, -1000, 0); CHECK(!can_proto_decode_drive_cmd(&f, &c));
}

static void test_decode_rejects(void)
{
  drive_cmd_t c;
  can_frame_t f = cmd_frame(1000, 0, 0, 0);

  f.id = CAN_ID_DRIVE_CMD + 1U;
  CHECK(!can_proto_decode_drive_cmd(&f, &c));
  f = cmd_frame(1000, 0, 0, 0);
  f.len = 7;
  CHECK(!can_proto_decode_drive_cmd(&f, &c));
  f.len = 12;
  CHECK(!can_proto_decode_drive_cmd(&f, &c));
}

static void test_encode_feedback(void)
{
  can_frame_t f;

  can_proto_encode_feedback(3, 1.5, -2.5f, &f);
  CHECK_EQ(f.id, CAN_ID_DRIVE_FB_BASE + 3U);
  CHECK_EQ(f.len, 8);
  CHECK_EQ(f.data[0], 0x00); CHECK_EQ(f.data[1], 0x00);          /* steering placeholder 0 */
  CHECK_EQ(f.data[2], 0x98); CHECK_EQ(f.data[3], 0x3A);          /* 15000 */
  CHECK_EQ(f.data[4], 0x00); CHECK_EQ(f.data[5], 0x00);
  CHECK_EQ(f.data[6], 0x3C); CHECK_EQ(f.data[7], 0xF6);          /* -2500 */

  can_proto_encode_feedback(1, -1.0, 0.0f, &f);                  /* -10000 */
  CHECK_EQ(f.data[2], 0xF0); CHECK_EQ(f.data[3], 0xD8);
  CHECK_EQ(f.data[4], 0xFF); CHECK_EQ(f.data[5], 0xFF);
}

static int16_t fb_velocity(const can_frame_t *f)
{
  return (int16_t)(uint16_t)(f->data[6] | (f->data[7] << 8));
}

static int32_t fb_position(const can_frame_t *f)
{
  return (int32_t)((uint32_t)f->data[2] | ((uint32_t)f->data[3] << 8) |
                   ((uint32_t)f->data[4] << 16) | ((uint32_t)f->data[5] << 24));
}

static void test_encode_limits(void)
{
  can_frame_t f;

  can_proto_encode_feedback(1, 0.0, 40.0f, &f);
  CHECK_EQ(fb_velocity(&f), INT16_MAX);
  can_proto_encode_feedback(1, 0.0, -40.0f, &f);
  CHECK_EQ(fb_velocity(&f), INT16_MIN);
  can_proto_encode_feedback(1, 0.0, 1.2346f, &f);                 /* rounds to nearest */
  CHECK_EQ(fb_velocity(&f), 1235);
  can_proto_encode_feedback(1, 300000.0, 0.0f, &f);               /* 3e9 wraps modulo 2^32 */
  CHECK_EQ(fb_position(&f), 3000000000LL - 4294967296LL);
}

static void test_dlc(void)
{
  CHECK_EQ(can_dlc_to_len(8), 8);
  CHECK_EQ(can_dlc_to_len(9), 12);
  CHECK_EQ(can_dlc_to_len(15), 64);
  CHECK_EQ(can_len_to_dlc(0), 0);
  CHECK_EQ(can_len_to_dlc(8), 8);
  CHECK_EQ(can_len_to_dlc(12), 9);
  CHECK_EQ(can_len_to_dlc(13), 10);
  CHECK_EQ(can_len_to_dlc(64), 15);
}

void test_can_proto(void)
{
  test_decode_endianness_and_sign();
  test_decode_modes();
  test_decode_rejects();
  test_encode_feedback();
  test_encode_limits();
  test_dlc();
}
