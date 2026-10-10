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
#include <math.h>
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
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
// --- ZMIENNE DO WYSYŁKI ---
CAN_TxHeaderTypeDef TxHeader;  // Struktura nagłówka ramki CAN
uint8_t TxData[8];             // Tablica z danymi (8 bajtów)
uint32_t TxMailbox;            // Zmienna przechowująca przypisany mailbox

// --- ZMIENNE DO ODBIORU ---
CAN_RxHeaderTypeDef RxHeader;  // Struktura nagłówka ramki odebranej
uint8_t RxData[8];             // Tablica na odebrane dane
volatile uint32_t RxCounter = 0; // Licznik odebranych ramek
uint16_t g = 0;

// --- JEDYNE OBSŁUGIWANE ID 0x300 (6 wartości od -100 do 100) ---
float PwmJoint1=0;
float PwmJoint2=0;
float PwmJoint3=0;
float PwmJoint4=0;
float PwmJoint5=0;
float PwmJoint6=0;

// --- ZMIENNE DO TIMEOUTU (ZABEZPIECZENIE) ---
volatile uint32_t CAN_LastRxTime = 0; // Przechowuje czas odebrania ostatniej ramki w
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
  MX_TIM14_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */

    // W STM32F4 kontroler CAN2 jest Slavem. Aby działał, MUSISZ włączyć zegar dla nadrzędnego CAN1!
    __HAL_RCC_CAN1_CLK_ENABLE();

    // --- POPRAWIONA, MAKSYMALNIE RESTRYKCYJNA KONFIGURACJA FILTRA: TYLKO ID 0x300 ---
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 14;                    // Dla CAN2 filtry zaczynają się domyślnie od banku 14
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // Używamy maski do dokładnego odcięcia jednego adresu
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    // Szukamy dokładnie ID 0x300
    sFilterConfig.FilterIdHigh = (0x300 << 5);
    sFilterConfig.FilterIdLow = 0x0000;

    // Maska 0x7FF oznacza: wszystkie 11 bitów ID musi się idealnie zgadzać z wartością 0x300
    sFilterConfig.FilterMaskIdHigh = (0x7FF << 5);
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    // Rejestracja filtra w strukturze hcan2
    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
    {
       Error_Handler();
    }

    // 1. Konfiguracja struktury ramki nadawczej CAN (Telemetria dla ROS2)
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x301;               // ID wysyłane z STM32, na które czeka bridge ROS2
    TxHeader.TransmitGlobalTime = DISABLE;

    // 2. Wypełnienie bufora danych zerami na start
    for(int i = 0; i < 8; i++)
    {
        TxData[i] = 0xFF;
    }

    // 3. Konfiguracja priorytetów NVIC (Timer wyżej, CAN niżej) oraz aktywacja linii przerwań
    HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);

    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);

    // 4. Start modułu CAN2
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    // 5. Włączenie powiadomień przerwań (Tylko dla odbioru danych z FIFO0, reszta zablokowana)
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    // Uruchomienie timera systemowego
    HAL_TIM_Base_Start_IT(&htim14);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

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
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            g = RxHeader.StdId;
            RxCounter++;

            CAN_LastRxTime=0;

            // Mignięcie diodą na płytce przy odebraniu poprawnej ramki 0x300
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

            // Rzutujemy dane bezpośrednio na liczby ze znakiem (int8_t)
            int8_t *signed_data = (int8_t*)RxData;

            // Przepisujemy 6 wartości (zakres -100 do 100) przesłanych z terminala/ROS2
            PwmJoint1 = signed_data[4];
            PwmJoint2 = signed_data[5];
            PwmJoint3 = signed_data[3];
            PwmJoint4 = signed_data[2];
            PwmJoint5 = signed_data[1];
            PwmJoint6 = signed_data[0];
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14)
    {
        static uint16_t cnt = 0;
        cnt++;

        CAN_LastRxTime=CAN_LastRxTime+10;

        if (CAN_LastRxTime >= 1500)
        {
        	PwmJoint1 = 0.0f;
            PwmJoint2 = 0.0f;
            PwmJoint3 = 0.0f;
            PwmJoint4 = 0.0f;
            PwmJoint5 = 0.0f;
            PwmJoint6 = 0.0f;
        }

        PwmJoint1 = constrain(PwmJoint1, -100.0f, 100.0f);
        PwmJoint2 = constrain(PwmJoint2, -100.0f, 100.0f);
        PwmJoint3 = constrain(PwmJoint3, -100.0f, 100.0f);
        PwmJoint4 = constrain(PwmJoint4, -100.0f, 100.0f);
        PwmJoint5 = constrain(PwmJoint5, -100.0f, 100.0f);
        PwmJoint6 = constrain(PwmJoint6, -100.0f, 100.0f);


       	HAL_GPIO_WritePin(Kierunek11_GPIO_Port, Kierunek11_Pin, PwmJoint1 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
       	HAL_GPIO_WritePin(Kierunek12_GPIO_Port, Kierunek12_Pin, PwmJoint1 >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,fabs(PwmJoint1));

        HAL_GPIO_WritePin(Kierunek21_GPIO_Port, Kierunek21_Pin, PwmJoint2 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Kierunek22_GPIO_Port, Kierunek22_Pin, PwmJoint2 >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2,fabs(PwmJoint2));

        //HAL_GPIO_WritePin(Kierunek31_GPIO_Port, Kierunek31_Pin, PwmJoint3 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET); //nie uzywam bo tutaj jest led
        HAL_GPIO_WritePin(Kierunek32_GPIO_Port, Kierunek32_Pin, PwmJoint3 >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4,fabs(PwmJoint3));

        HAL_GPIO_WritePin(Kierunek41_GPIO_Port, Kierunek41_Pin, PwmJoint4 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Kierunek42_GPIO_Port, Kierunek42_Pin, PwmJoint4 >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,fabs(PwmJoint4));

        HAL_GPIO_WritePin(Kierunek51_GPIO_Port, Kierunek51_Pin, PwmJoint5 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Kierunek52_GPIO_Port, Kierunek52_Pin, PwmJoint5 >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,fabs(PwmJoint5));

        HAL_GPIO_WritePin(Kierunek61_GPIO_Port, Kierunek61_Pin, PwmJoint6 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Kierunek62_GPIO_Port, Kierunek62_Pin, PwmJoint6 >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,fabs(PwmJoint6));

        if (cnt % 5 == 0) // --- WYKONYWANE CO 50ms ---
        {
            // Miejsce na szybką telemetrię
        }

        if (cnt % 10 == 0) // --- WYKONYWANE CO 100ms ---
        {
            // Pętla wysyłająca dane zwrotne (0x210) do komputera
            if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0)
            {
                if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
                {
                    /* Obsługa błędu nadawania */
                }
            }
        }

        if (cnt % 100 == 0) // --- WYKONYWANE CO 1000ms ---
        {

        }

        if (cnt >= 200) // --- RESET LICZNIKA CO 2s ---
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
