/**
  * @file    drv_encoder.c
  * @brief   Motor N uses TIM(N+1). TIM2/TIM5 are 32-bit, TIM3/TIM4 16-bit.
  */
#include "drv_encoder.h"
#include "app_config.h"
#include "encoder_math.h"
#include "main.h"
#include "tim.h"

static TIM_HandleTypeDef *const enc_tim[MOTOR_COUNT] = MOTOR_ENC_TIMS;
static const uint8_t is_32bit[MOTOR_COUNT] = MOTOR_ENC_IS_32BIT;
static const int8_t  enc_sign[MOTOR_COUNT] = ENCODER_SIGN;
static uint32_t      prev_count[MOTOR_COUNT];

void drv_encoder_init(void)
{
  for (uint8_t m = 0U; m < MOTOR_COUNT; m++)
  {
    if (HAL_TIM_Encoder_Start(enc_tim[m], TIM_CHANNEL_ALL) != HAL_OK)
    {
      Error_Handler();
    }
    prev_count[m] = __HAL_TIM_GET_COUNTER(enc_tim[m]);
  }
}

int32_t drv_encoder_read_delta(uint8_t motor)
{
  uint32_t now = __HAL_TIM_GET_COUNTER(enc_tim[motor]);
  int32_t  delta = (is_32bit[motor] != 0U)
                   ? encoder_math_delta32(now, prev_count[motor])
                   : encoder_math_delta16((uint16_t)now, (uint16_t)prev_count[motor]);

  prev_count[motor] = now;
  return delta * enc_sign[motor];
}
