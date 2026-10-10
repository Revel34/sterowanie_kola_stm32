/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define E4_Pin GPIO_PIN_0
#define E4_GPIO_Port GPIOC
#define E3_Pin GPIO_PIN_1
#define E3_GPIO_Port GPIOC
#define E1_Pin GPIO_PIN_1
#define E1_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define KRK_DIR_Pin GPIO_PIN_4
#define KRK_DIR_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define BLDC_FR_Pin GPIO_PIN_5
#define BLDC_FR_GPIO_Port GPIOC
#define E2_Pin GPIO_PIN_0
#define E2_GPIO_Port GPIOB
#define E8_Pin GPIO_PIN_1
#define E8_GPIO_Port GPIOB
#define E7_Pin GPIO_PIN_2
#define E7_GPIO_Port GPIOB
#define E6_Pin GPIO_PIN_12
#define E6_GPIO_Port GPIOB
#define E5_Pin GPIO_PIN_15
#define E5_GPIO_Port GPIOB
#define KRK_EN_Pin GPIO_PIN_10
#define KRK_EN_GPIO_Port GPIOA
#define BLDC_BK_Pin GPIO_PIN_11
#define BLDC_BK_GPIO_Port GPIOA
#define BLDC_EN_Pin GPIO_PIN_12
#define BLDC_EN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define KRK_M2_Pin GPIO_PIN_10
#define KRK_M2_GPIO_Port GPIOC
#define KRK_RESET_Pin GPIO_PIN_11
#define KRK_RESET_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define KRK_M1_Pin GPIO_PIN_4
#define KRK_M1_GPIO_Port GPIOB
#define KRK_M0_Pin GPIO_PIN_5
#define KRK_M0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
