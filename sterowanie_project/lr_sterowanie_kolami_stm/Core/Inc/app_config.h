/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Every application constant of the wheel-control firmware.
  *
  *          Markers:
  *            PLACEHOLDER - REPLACE              generic/legacy value, replace
  *            UNKNOWN - REQUIRES CONFIRMATION    depends on hardware not seen
  *          Unmarked values were fixed by the project owner or by the CubeMX
  *          configuration.
  *
  *          This header is HAL-free. The "HAL mapping" section names HAL
  *          symbols but is only expanded inside drv_*.c files.
  ******************************************************************************
  */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Board ---------------------------------------------------------------------*/
#define MOTOR_COUNT                 4U

/* Wheel number (1 = right-front, 2 = right-rear, 3 = left-front,
 * 4 = left-rear) driven by motor 1..4. Selects kinematics side, direction
 * flip and the feedback CAN ID. */
#define MOTOR_WHEEL_NO              { 1U, 2U, 3U, 4U }  /* confirmed: motor N drives wheel N */

/* Indexed by wheel number - 1. Right wheels x -1, left wheels x +1. */
#define WHEEL_IS_RIGHT_SIDE         { 1, 1, 0, 0 }
#define WHEEL_DIR_FLIP              { -1.0f, -1.0f, 1.0f, 1.0f }

/* Mechanics (Rover 3.0 legacy values) ---------------------------------------*/
#define GEAR_RATIO                  19.0f   /* PLACEHOLDER - REPLACE */
#define ENC_COUNTS_PER_MOTOR_REV    28.0f   /* PLACEHOLDER - REPLACE */
#define WHEEL_DIAMETER_M            0.26f   /* PLACEHOLDER - REPLACE */
#define ROVER_WIDTH_M               0.713f  /* PLACEHOLDER - REPLACE */
/* Rover length 0.789 m (PLACEHOLDER) is not used by fixed-wheel kinematics. */

/* Encoder count sign per motor (+1: count up = positive wheel rotation). */
#define ENCODER_SIGN                { 1, 1, 1, 1 }      /* UNKNOWN - REQUIRES CONFIRMATION */

/* Timing --------------------------------------------------------------------*/
#define CONTROL_TICK_MS             10U     /* TIM6 update period (CubeMX) */
#define CONTROL_TICKS_PER_STEP      10U     /* control + feedback every 100 ms */
#define CMD_TIMEOUT_MS              2500U   /* PLACEHOLDER - REPLACE */

/* Motor output --------------------------------------------------------------*/
/* Controller signal that maps to 100 % TIM1 duty (legacy PWM scale). */
#define MOTOR_SIGNAL_FULL_SCALE     1000.0f
/* Open loop: signal = setpoint [m/s] x gain, clamped to full scale. */
#define OPEN_LOOP_GAIN              330.0f  /* PLACEHOLDER - REPLACE */

/* GPIO level (0/1) that enables an H-bridge. Active-high keeps the bridges
 * off with the reset-low pin state. */
#define MOTOR_EN_ACTIVE_LEVEL       1U      /* UNKNOWN - REQUIRES CONFIRMATION */
/* GPIO level (0/1) on DIR for a positive signal (legacy: low). */
#define MOTOR_DIR_POSITIVE_LEVEL    0U      /* UNKNOWN - REQUIRES CONFIRMATION */

/* PI velocity controller (error in m/s, output in signal units) -------------*/
#define PI_KP                       32.629f  /* PLACEHOLDER - REPLACE */
#define PI_KI                       995.281f /* PLACEHOLDER - REPLACE */
#define PI_KD                       0.0f     /* PLACEHOLDER - REPLACE */
#define PI_INTEGRAL_LIMIT           1.0f     /* PLACEHOLDER - REPLACE */
#define PI_OUTPUT_LIMIT             1000.0f  /* PLACEHOLDER - REPLACE */
/* |setpoint| below this resets the controller and forces 0 output. */
#define PI_ZERO_SETPOINT_EPS        0.01f    /* PLACEHOLDER - REPLACE */

/* CAN-FD protocol -----------------------------------------------------------*/
#define CAN_ID_DRIVE_CMD            0x100U  /* PLACEHOLDER - REPLACE */
#define CAN_ID_DRIVE_FB_BASE        0x100U  /* PLACEHOLDER - REPLACE (ID = base + wheel no.) */
#define CAN_DRIVE_DLC               8U
#define CAN_CMD_SCALE               1000.0f /* command int16 / 1000 */
#define CAN_FB_ANGLE_SCALE          10000.0 /* steering and position: rad x 10000 */
#define CAN_FB_VEL_SCALE            1000.0  /* velocity: rad/s x 1000 */
/* Raw steering field reported by fixed wheels (no steering). */
#define CAN_FB_STEERING_RAW         0       /* PLACEHOLDER - REPLACE */
/* Mode field tolerance in raw units (legacy +-0.1). */
#define CAN_MODE_TOLERANCE_RAW      100     /* PLACEHOLDER - REPLACE */

/* Standard IDs accepted by the hardware filters (exact match, two per filter
 * element). Everything else is rejected. */
#define CAN_RX_ACCEPT_IDS           { CAN_ID_DRIVE_CMD }

/* CAN-FD transport ----------------------------------------------------------*/
#define CAN_FRAME_MAX_LEN           64U
#define CAN_RX_RING_CAPACITY        16U     /* PLACEHOLDER - REPLACE (power of 2) */
/* Transmitter delay compensation, offset in mtq. To be verified on a scope. */
#define CAN_TDC_OFFSET(dbrp, dtseg1) ((dbrp) * (dtseg1)) /* PLACEHOLDER - REPLACE */
#define CAN_TDC_FILTER              0U
/* Bus-off recovery: wait before each restart, give up after N attempts
 * without a received frame (the IWDG then resets the MCU). */
#define CAN_BUSOFF_RETRY_DELAY_MS   100U    /* PLACEHOLDER - REPLACE */
#define CAN_BUSOFF_MAX_RETRIES      10U     /* PLACEHOLDER - REPLACE */

/* MAB PDS (documentation only, the STM never talks to the PDS):
 * CAN ID property 0x20, BAUDRATE property 0x21 (1M/2M/5M/8M),
 * default PDS CAN ID 0x64. */

/* HAL mapping (expanded only in drv_*.c) ------------------------------------*/
#define MOTOR_PWM_TIM               htim1
#define MOTOR_PWM_CHANNELS          { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 }
#define MOTOR_DIR_PINS              { { M1_DIR_GPIO_Port, M1_DIR_Pin }, { M2_DIR_GPIO_Port, M2_DIR_Pin }, \
                                      { M3_DIR_GPIO_Port, M3_DIR_Pin }, { M4_DIR_GPIO_Port, M4_DIR_Pin } }
#define MOTOR_EN_PINS               { { M1_EN_GPIO_Port, M1_EN_Pin }, { M2_EN_GPIO_Port, M2_EN_Pin }, \
                                      { M3_EN_GPIO_Port, M3_EN_Pin }, { M4_EN_GPIO_Port, M4_EN_Pin } }
#define MOTOR_ENC_TIMS              { &htim2, &htim3, &htim4, &htim5 }
#define MOTOR_ENC_IS_32BIT          { 1, 0, 0, 1 }
#define CAN_HANDLE                  hfdcan1
#define CONTROL_TICK_TIM            htim6
#define WATCHDOG_HANDLE             hiwdg

#endif /* APP_CONFIG_H */
