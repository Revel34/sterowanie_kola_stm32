/**
  * @file    drv_motor.h
  * @brief   H-bridge outputs: TIM1 PWM + DIR/EN GPIO per motor.
  */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* Duty 0, bridges disabled, then PWM started at 0. */
void drv_motor_init(void);
/* duty in [-1, 1], sign selects DIR. */
void drv_motor_set_duty(uint8_t motor, float duty);
void drv_motor_enable_all(bool enable);

#endif /* DRV_MOTOR_H */
