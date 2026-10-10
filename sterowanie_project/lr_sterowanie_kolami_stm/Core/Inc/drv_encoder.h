/**
  * @file    drv_encoder.h
  * @brief   Quadrature encoder timers (TIM2..TIM5).
  */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

void    drv_encoder_init(void);
/* Signed counts since the previous call, ENCODER_SIGN applied. */
int32_t drv_encoder_read_delta(uint8_t motor);

#endif /* DRV_ENCODER_H */
