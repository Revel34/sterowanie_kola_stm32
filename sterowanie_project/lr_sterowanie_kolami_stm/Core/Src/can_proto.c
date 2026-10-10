/**
  * @file    can_proto.c
  * @brief   DRIVE protocol.
  *          Command 0x100, DLC 8: int16 lin_x, lin_y, lin_z (mode), ang_z, each / 1000.
  *          Feedback base + wheel, DLC 8: int16 steering (rad x 10000),
  *          int32 position (rad x 10000, wraps modulo 2^32), int16 velocity
  *          (rad/s x 1000, saturated).
  */
#include "can_proto.h"

#define CMD_OFS_LIN_X   0U
#define CMD_OFS_LIN_Y   2U
#define CMD_OFS_MODE    4U
#define CMD_OFS_ANG_Z   6U
#define FB_OFS_STEERING 0U
#define FB_OFS_POSITION 2U
#define FB_OFS_VELOCITY 6U

/* CAN-FD DLC code to payload length (ISO 11898-1). */
static const uint8_t dlc_len[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

uint8_t can_dlc_to_len(uint8_t dlc)
{
  return dlc_len[dlc & 0x0FU];
}

/* Smallest DLC code whose length holds len bytes. */
uint8_t can_len_to_dlc(uint8_t len)
{
  uint8_t dlc = 0U;

  while ((dlc < 15U) && (dlc_len[dlc] < len))
  {
    dlc++;
  }
  return dlc;
}

static int16_t get_i16(const uint8_t *p)
{
  return (int16_t)(uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
  put_u16(p, (uint16_t)v);
  put_u16(p + 2, (uint16_t)(v >> 16));
}

static int64_t round_to_i64(double x)
{
  return (int64_t)((x >= 0.0) ? (x + 0.5) : (x - 0.5));
}

static int16_t sat_i16(double x)
{
  if (x >= (double)INT16_MAX)
  {
    return INT16_MAX;
  }
  if (x <= (double)INT16_MIN)
  {
    return INT16_MIN;
  }
  return (int16_t)round_to_i64(x);
}

static bool mode_matches(int16_t raw, int16_t mode)
{
  int32_t diff = (int32_t)raw - (int32_t)mode * (int32_t)CAN_CMD_SCALE;

  return (diff > -CAN_MODE_TOLERANCE_RAW) && (diff < CAN_MODE_TOLERANCE_RAW);
}

static bool decode_mode(int16_t raw, drive_mode_t *mode)
{
  static const drive_mode_t modes[] = { DRIVE_MODE_OPEN_LOOP, DRIVE_MODE_PID, DRIVE_MODE_X_STOP };

  for (uint32_t i = 0U; i < sizeof(modes) / sizeof(modes[0]); i++)
  {
    if (mode_matches(raw, (int16_t)modes[i]))
    {
      *mode = modes[i];
      return true;
    }
  }
  return false;
}

bool can_proto_decode_drive_cmd(const can_frame_t *frame, drive_cmd_t *cmd)
{
  if ((frame->id != CAN_ID_DRIVE_CMD) || (frame->len != CAN_DRIVE_DLC))
  {
    return false;
  }
  if (!decode_mode(get_i16(&frame->data[CMD_OFS_MODE]), &cmd->mode))
  {
    return false;
  }
  cmd->lin_x = (float)get_i16(&frame->data[CMD_OFS_LIN_X]) / CAN_CMD_SCALE;
  cmd->lin_y = (float)get_i16(&frame->data[CMD_OFS_LIN_Y]) / CAN_CMD_SCALE;
  cmd->ang_z = (float)get_i16(&frame->data[CMD_OFS_ANG_Z]) / CAN_CMD_SCALE;
  return true;
}

void can_proto_encode_feedback(uint8_t wheel_no, double position_rad,
                               float velocity_rad_s, can_frame_t *frame)
{
  frame->id = CAN_ID_DRIVE_FB_BASE + wheel_no;
  frame->len = CAN_DRIVE_DLC;
  put_u16(&frame->data[FB_OFS_STEERING], (uint16_t)(int16_t)CAN_FB_STEERING_RAW);
  put_u32(&frame->data[FB_OFS_POSITION], (uint32_t)round_to_i64(position_rad * CAN_FB_ANGLE_SCALE));
  put_u16(&frame->data[FB_OFS_VELOCITY], (uint16_t)sat_i16((double)velocity_rad_s * CAN_FB_VEL_SCALE));
}
