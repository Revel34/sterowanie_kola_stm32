/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MyStruct.h" //to moja bibliteka od struktury załączajacej mosfety
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//Tworze 12 obiektów - obwodów elektrycznych
ELECTRIC_CIRCUIT_Type C1 = {"C1", OFF, 0, 0, OBW_1_GPIO_Port, OBW_1_Pin};
ELECTRIC_CIRCUIT_Type C2 = {"C2", OFF, 0, 0, OBW_2_GPIO_Port, OBW_2_Pin};
ELECTRIC_CIRCUIT_Type C3 = {"C3", OFF, 0, 0, OBW_3_GPIO_Port, OBW_3_Pin};
ELECTRIC_CIRCUIT_Type C4 = {"C4", OFF, 0, 0, OBW_4_GPIO_Port, OBW_4_Pin};
ELECTRIC_CIRCUIT_Type C5 = {"C5", OFF, 0, 0, OBW_5_GPIO_Port, OBW_5_Pin};
ELECTRIC_CIRCUIT_Type C6 = {"C6", OFF, 0, 0, OBW_6_GPIO_Port, OBW_6_Pin};
ELECTRIC_CIRCUIT_Type C7 = {"C7", OFF, 0, 0, OBW_7_GPIO_Port, OBW_7_Pin};
ELECTRIC_CIRCUIT_Type C8 = {"C8", OFF, 0, 0, OBW_8_GPIO_Port, OBW_8_Pin};
ELECTRIC_CIRCUIT_Type C9 = {"C9", OFF, 0, 0, OBW_9_GPIO_Port, OBW_9_Pin};
ELECTRIC_CIRCUIT_Type C10 = {"C10", OFF, 0, 0, OBW_10_GPIO_Port, OBW_10_Pin};
ELECTRIC_CIRCUIT_Type C11 = {"C11", OFF, 0, 0, OBW_11_GPIO_Port, OBW_11_Pin};
ELECTRIC_CIRCUIT_Type C12 = {"C12", OFF, 0, 0, OBW_12_GPIO_Port, OBW_12_Pin};

// --- ZMIENNE DO WYSYŁKI ---
CAN_TxHeaderTypeDef TxHeader; // Struktura nagłówka ramki CAN
uint8_t TxData[8]; // Tablica z danymi (8 bajtów)
uint32_t TxMailbox; // Zmienna przechowująca przypisany mailbox

// --- ZMIENNE DO ODBIORU ---
CAN_RxHeaderTypeDef RxHeader; // Struktura nagłówka ramki odebranej
uint8_t RxData[8]; // Tablica na odebrane dane
volatile uint32_t RxCounter = 0; // Licznik odebranych ramek o ID 0x100
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  __HAL_RCC_CAN1_CLK_ENABLE();

  // --- KONFIGURACJA FILTRA SPRZĘTOWEGO DLA CAN2 (TYLKO ID 0x50) ---
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 14;              // Bank 14 dla CAN2
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST; // Tryb listy (dokładne dopasowanie ID)
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    // Zmiana z 0x100 na 0x50:
    sFilterConfig.FilterIdHigh = (0x50 << 5);   // Akceptujemy wyłącznie ID: 0x050 w hex
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = (0x50 << 5); // W trybie IDLIST to pole oznacza drugie akceptowane ID (też ustawiamy 0x50)
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
    {
       Error_Handler();
    }

  // 1. Konfiguracja struktury ramki nadawczej CAN (dla CAN2)
  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x051;
  TxHeader.TransmitGlobalTime = DISABLE;

  // 2. Wypełnienie bufora danych samymi jedynkami (0xFF)
  for(int i = 0; i < 8; i++)
  {
    TxData[i] = 0xFF;
  }

  // 3. Start modułu CAN2
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

  // 4. Włączenie przerwań odbioru dla CAN2
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_BUSOFF);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);

  // Start timerów i PWM
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  TurnOn(&C1);//Zaraz po załączeniu włączam obwód 1
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
* Struktura ramki sterującej CAN (ID: 0x100, DLC: >= 4)
* * Kodowanie stanu (2 bity na obwód):
* 00 (0) - Ignoruj / Brak zmian
* 01 (1) - Włącz (TurnOn)
* 10 (2) - Wyłącz (TurnOff)
* * RxData[0]: [Bit 7-6]: C4 | [Bit 5-4]: C3 | [Bit 3-2]: C2 | [Bit 1-0]: C1
* RxData[1]: [Bit 7-6]: C8 | [Bit 5-4]: C7 | [Bit 3-2]: C6 | [Bit 1-0]: C5
* RxData[2]: [Bit 7-6]: C12 | [Bit 5-4]: C11 | [Bit 3-2]: C10 | [Bit 1-0]: C9
* RxData[3]: Komenda specjalna (0xAA = GLOBALNY STOP, 0x00 = Normalna praca)
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  // Pętla while opróżnia całe FIFO, żeby się nie zapchało
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);// Testowe mignięcie diodą przy odbiorze ramki

      // POPRAWIONE: Warunek dopasowany do ID z filtra i komentarza (0x100)
      if (RxHeader.StdId == 0x50 && RxHeader.DLC >= 3)
      {
        RxCounter++;

        // --- SPRAWDZENIE KOMENDY SPECJALNEJ (Bajt 3) ---
        if (RxHeader.DLC >= 4 && RxData[3] == 0xAA)
        {
          // Globalny wymuszony STOP dla wszystkich obwodów
          TurnOff(&C1); TurnOff(&C2); TurnOff(&C3); TurnOff(&C4);
          TurnOff(&C5); TurnOff(&C6); TurnOff(&C7); TurnOff(&C8);
          TurnOff(&C9); TurnOff(&C10); TurnOff(&C11); TurnOff(&C12);
          continue; // Pomiń dalsze dekodowanie bitów
        }

        uint8_t stan_obwodu = 0;

        // 2. DEKODOWANIE BAJTU 0 (Obwody C1, C2, C3, C4)
        // --- Obwód C1 (Bity 1-0) ---
        stan_obwodu = (RxData[0] >> 0) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C1); }
        else if (stan_obwodu == 2) { TurnOff(&C1); }

        // --- Obwód C2 (Bity 3-2) ---
        stan_obwodu = (RxData[0] >> 2) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C2); }
        else if (stan_obwodu == 2) { TurnOff(&C2); }

        // --- Obwód C3 (Bity 5-4) ---
        stan_obwodu = (RxData[0] >> 4) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C3); }
        else if (stan_obwodu == 2) { TurnOff(&C3); }

        // --- Obwód C4 (Bity 7-6) ---
        stan_obwodu = (RxData[0] >> 6) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C4); }
        else if (stan_obwodu == 2) { TurnOff(&C4); }

        // 3. DEKODOWANIE BAJTU 1 (Obwody C5, C6, C7, C8)
        // --- Obwód C5 (Bity 1-0) ---
        stan_obwodu = (RxData[1] >> 0) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C5); }
        else if (stan_obwodu == 2) { TurnOff(&C5); }

        // --- Obwód C6 (Bity 3-2) ---
        stan_obwodu = (RxData[1] >> 2) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C6); }
        else if (stan_obwodu == 2) { TurnOff(&C6); }

        // --- Obwód C7 (Bity 5-4) ---
        stan_obwodu = (RxData[1] >> 4) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C7); }
        else if (stan_obwodu == 2) { TurnOff(&C7); }

        // --- Obwód C8 (Bity 7-6) ---
        stan_obwodu = (RxData[1] >> 6) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C8); }
        else if (stan_obwodu == 2) { TurnOff(&C8); }

        // 4. DEKODOWANIE BAJTU 2 (Obwody C9, C10, C11, C12)
        // --- Obwód C9 (Bity 1-0) ---
        stan_obwodu = (RxData[2] >> 0) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C9); }
        else if (stan_obwodu == 2) { TurnOff(&C9); }

        // --- Obwód C10 (Bity 3-2) ---
        stan_obwodu = (RxData[2] >> 2) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C10); }
        else if (stan_obwodu == 2) { TurnOff(&C10); }

        // --- Obwód C11 (Bity 5-4) ---
        stan_obwodu = (RxData[2] >> 4) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C11); }
        else if (stan_obwodu == 2) { TurnOff(&C11); }

        // --- Obwód C12 (Bity 7-6) ---
        stan_obwodu = (RxData[2] >> 6) & 0x03;
        if (stan_obwodu == 1) { TurnOn(&C12); }
        else if (stan_obwodu == 2) { TurnOff(&C12); }
      }
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM10) // Ten zegar tyka co 10ms
  {
    static uint16_t cnt = 0;
    cnt++;

    // Wykonaj wysyłkę statusu co 100ms
    if (cnt % 10 == 0)
    {
      // 1. Ustawienie ID ramki na 0x051 (Status zasilania oczekiwany przez ROS2)
      TxHeader.StdId = 0x051;
      TxHeader.DLC = 2; // Wysyłamy 2 bajty, które pokryją 12 obwodów

      // 2. Zerowanie bufora przed wpisaniem aktualnych stanów bitowych
      TxData[0] = 0x00; // Dla obwodów C1 - C8
      TxData[1] = 0x00; // Dla obwodów C9 - C12

      // 3. Pakowanie stanów obwodów C1 - C8 (Bajt 0)
      // Sprawdzamy pole 'current_state' według Twojej definicji w strukturze
      if (C1.current_state == ON)  { TxData[0] |= (1 << 0); }
      if (C2.current_state == ON)  { TxData[0] |= (1 << 1); }
      if (C3.current_state == ON)  { TxData[0] |= (1 << 2); }
      if (C4.current_state == ON)  { TxData[0] |= (1 << 3); }
      if (C5.current_state == ON)  { TxData[0] |= (1 << 4); }
      if (C6.current_state == ON)  { TxData[0] |= (1 << 5); }
      if (C7.current_state == ON)  { TxData[0] |= (1 << 6); }
      if (C8.current_state == ON)  { TxData[0] |= (1 << 7); }

      // 4. Pakowanie stanów obwodów C9 - C12 (Bajt 1)
      if (C9.current_state == ON)  { TxData[1] |= (1 << 0); }
      if (C10.current_state == ON) { TxData[1] |= (1 << 1); }
      if (C11.current_state == ON) { TxData[1] |= (1 << 2); }
      if (C12.current_state == ON) { TxData[1] |= (1 << 3); }

      // 5. Wysłanie ramki przez moduł CAN2
      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0)
      {
        if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
        {
          /* Opcjonalnie: Obsługa błędu nadawania */
        }
      }
    }

    if (cnt >= 100)
    {
      cnt = 0;
    }
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
