#include "app_config.h"
#include "encoder_math.h"
#include "test.h"

#define TWO_PI_D 6.283185307179586

void test_encoder_math(void)
{
  const int32_t per_rev = (int32_t)(ENC_COUNTS_PER_MOTOR_REV * GEAR_RATIO);

  /* 16-bit wrap. */
  CHECK_EQ(encoder_math_delta16(100, 50), 50);
  CHECK_EQ(encoder_math_delta16(5, 65530), 11);
  CHECK_EQ(encoder_math_delta16(65530, 5), -11);
  CHECK_EQ(encoder_math_delta16(0, 32768), -32768);

  /* 32-bit wrap. */
  CHECK_EQ(encoder_math_delta32(3U, 0xFFFFFFFEU), 5);
  CHECK_EQ(encoder_math_delta32(0xFFFFFFFEU, 3U), -5);
  CHECK_EQ(encoder_math_delta32(0x80000010U, 0x7FFFFFF0U), 0x20);

  /* Conversions: one wheel revolution. */
  CHECK_NEAR(encoder_math_counts_to_rad(per_rev), TWO_PI_D, 1e-9);
  CHECK_NEAR(encoder_math_counts_to_rad(-3LL * per_rev), -3.0 * TWO_PI_D, 1e-9);
  CHECK_NEAR(encoder_math_counts_to_rad(1000000000000LL), 1e12 * TWO_PI_D / per_rev, 1e-3);
  CHECK_NEAR(encoder_math_rad_per_s(per_rev, 1.0f), TWO_PI_D, 1e-5);
  CHECK_NEAR(encoder_math_rad_per_s(-per_rev, 0.1f), -10.0 * TWO_PI_D, 1e-4);
  CHECK_NEAR(encoder_math_rad_per_s(10, 0.0f), 0.0, 0.0);
}
