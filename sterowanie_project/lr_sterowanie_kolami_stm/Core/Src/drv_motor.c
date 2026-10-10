/**
  * @file    drv_motor.c
  * @brief   Motor N uses TIM1 channel N and M<N>_DIR / M<N>_EN.
  */
#include "drv_motor.h"
#include "app_config.h"
#include "main.h"
#include "tim.h"

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
} pin_t;

static const uint32_t pwm_channel[MOTOR_COUNT] = MOTOR_PWM_CHANNELS;
static const pin_t    dir_pin[MOTOR_COUNT] = MOTOR_DIR_PINS;
static const pin_t    en_pin[MOTOR_COUNT] = MOTOR_EN_PINS;

static GPIO_PinState level(uint32_t lvl)
{
  return (lvl != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void drv_motor_init(void)
{
  drv_motor_enable_all(false);
  for (uint8_t m = 0U; m < MOTOR_COUNT; m++)
  {
    drv_motor_set_duty(m, 0.0f);
    if (HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, pwm_channel[m]) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

void drv_motor_set_duty(uint8_t motor, float duty)
{
  bool     positive = (duty >= 0.0f);
  float    magnitude = positive ? duty : -duty;
  uint32_t full = __HAL_TIM_GET_AUTORELOAD(&MOTOR_PWM_TIM) + 1U;

  if (!(magnitude < 1.0f))
  {
    magnitude = (magnitude >= 1.0f) ? 1.0f : 0.0f;  /* NaN -> 0 */
  }
  HAL_GPIO_WritePin(dir_pin[motor].port, dir_pin[motor].pin,
                    level(positive ? MOTOR_DIR_POSITIVE_LEVEL : !MOTOR_DIR_POSITIVE_LEVEL));
  __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, pwm_channel[motor], (uint32_t)(magnitude * (float)full));
}

void drv_motor_enable_all(bool enable)
{
  for (uint8_t m = 0U; m < MOTOR_COUNT; m++)
  {
    HAL_GPIO_WritePin(en_pin[m].port, en_pin[m].pin,
                      level(enable ? MOTOR_EN_ACTIVE_LEVEL : !MOTOR_EN_ACTIVE_LEVEL));
  }
}
