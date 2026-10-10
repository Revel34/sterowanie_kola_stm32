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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>

#include "tim.h"
#include "ALL_STRUCT.h"          //to moja bibliteka od ogólnego sterowania
#include "BLDC_MOTOR.h"          //to moja bibliteka od sterowania silnikami bldc
#include "ABS_ENCODER.h"         //to moja bibliteka od enodera absolutnego
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//int tab[11] = {53, -1, -1, -1, 1, 1, 0, 0, 0, 0, 1}; //to prawe przód
//int tab[11] = {-109, -1, -1, 1, 2, 2, 0, 0, 0, 0, -1 };//to prawe tył
//int tab[11] = {42, 1, 1, 1, 3, 3, 0, 0, 0, 0, -1};//to lewe przód
int tab[11] = {-125, 1, 1, -1, 4, 4, 0, 0, 0, 0, 1 };//to lewe tył
//0 wartosc to kat zero stopni
//1 wartosc to zmiana predkosci x_lin na przeciwny
//2 wartosc to zmiana predkosci y_lin
//3 wartosc to zmiana predkosci z_ang
//4 wartośc to wybór koła w trybie jazdy po łuku
//5 wartośc to nazwa w joint_states

volatile uint32_t PG_counter = 0;//to zmienna do zapisywania ilosci impulsów ze sterownika bldc - do predkosci
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t flag_10ms = 0;   //to flagi z przerwania co określony czas
volatile uint8_t flag_50ms = 0;
volatile uint8_t flag_100ms = 0;
volatile uint8_t flag_1000ms = 0;

volatile uint16_t can_timeout_counter = 0; //to zmienna która mówi nami kiedy ostani raz przyszła poprawna ramka danych z can wartosc 10 to 100ms

float lin_x = 0.0f;
float lin_y= 0.0f;
float lin_z= 0.0f;
float ang_z= 0.0f;

float x=0; // to kąt z abs
float y=0;

uint8_t bity_enkodera[8];
int dec=0;
float kat;

WHEEL_Type Wheel; //tworzę obiekt

// --- ZMIENNE DO WYSYŁKI ---
CAN_TxHeaderTypeDef TxHeader;  // Struktura nagłówka ramki CAN
uint8_t TxData[8];             // Tablica z danymi (8 bajtów)
uint32_t TxMailbox;            // Zmienna przechowująca przypisany mailbox

// --- ZMIENNE DO ODBIORU ---
CAN_RxHeaderTypeDef RxHeader;  // Struktura nagłówka ramki odebranej
uint8_t RxData[8];             // Tablica na odebrane dane
volatile uint32_t RxCounter = 0; // Licznik odebranych ramek o ID 0x100
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void emergency_stop(void);
void PackWheelJointStateData(uint8_t *target_buffer, float current_angle_deg, WHEEL_Type *key); // to prototyp funkcji która wysyła dane do jointstates
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
  GenerateVelTab(&Wheel,tab);        //Generuje tablicę zmiany prędkosci - teraz tylko używam podstawienia wartosci kąta zero stopni

  //dwie linikji niżej powoduja ze gdy nie ma agenta to silnik bldc nie skacze
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,0);//ustawiam pwm na zero
  HAL_GPIO_WritePin(BLDC_BK_GPIO_Port, BLDC_BK_Pin, SET); //oraz nie zezwalam na prace
  //dwie linikji niżej powoduja ze gdy nie ma agenta to silnik krokowy nie skacze
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  HAL_GPIO_WritePin(KRK_RESET_GPIO_Port,KRK_RESET_Pin, RESET);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM10_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // --- KONFIGURACJA FILTRA SPRZĘTOWEGO (TYLKO ID 0x100) ---
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = (0x100 << 5); // Akceptujemy wyłącznie ID: 0x100
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = (0x100 << 5);
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
       Error_Handler();
    }

    // 1. Konfiguracja struktury ramki nadawczej CAN
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    // DYNAMICZNE USTAWIANIE ID OD 101 DO 105 NA PODSTAWIE tab[4]
    TxHeader.StdId = 0x100 + tab[4];
    TxHeader.TransmitGlobalTime = DISABLE;

    // 2. Wypełnienie bufora danych samymi jedynkami (0xFF)
    for(int i = 0; i < 8; i++)
    {
        TxData[i] = 0xFF;
    }

    // 3. Start modułu CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    // 4. Włączenie przerwań odbioru
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

    HAL_TIM_Base_Start_IT(&htim10);  //to start timera 10 w trybie przerwaniowym
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); //to start PWM do silnika bldc
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //to start PWM do silnika krokowego
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //to start timera od enkoderów
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
    // Pętla while opróżnia całe FIFO, żeby się nie zapchało
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // Testowe mignięcie diodą przy odbiorze ramki
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

            if (RxHeader.StdId == 0x100)
            {
                RxCounter++; // Zwiększ licznik testowy

                int16_t *rawValues = (int16_t*)RxData;

                lin_x = rawValues[0] / 1000.0f;
                lin_y = rawValues[1] / 1000.0f;
                lin_z = rawValues[2] / 1000.0f;
                ang_z = rawValues[3] / 1000.0f;

                ReadNewCmdVel(lin_x, lin_y, lin_z, ang_z, &Wheel, tab);

                can_timeout_counter = 0; // Reset timeoutu
            }
        }
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // to do przerwań z timerów
{
	if (htim->Instance == TIM10) //ten jest co 10ms
	{
		static uint16_t cnt = 0;
		cnt++;

		flag_10ms = 1; // Zawsze co 10ms

		if (can_timeout_counter < 250)
		{
			can_timeout_counter++;
		}
		else if (can_timeout_counter == 250)
		{// Minęła dokładnie 2,5 sekundy bez nowej ramki!
			lin_x = 0.0f; //wymuszamy same zera
			lin_y = 0.0f;
			lin_z = 0.0f;
			ang_z = 0.0f;
			ReadNewCmdVel(lin_x, lin_y, lin_z, ang_z, &Wheel, tab);

			can_timeout_counter++;
		}

		if (cnt % 5 == 0)
		{
			flag_50ms = 1;

			bity_enkodera[0] = (HAL_GPIO_ReadPin(E1_GPIO_Port, E1_Pin) == GPIO_PIN_SET) ? 1 : 0;
	        bity_enkodera[1] = (HAL_GPIO_ReadPin(E2_GPIO_Port, E2_Pin) == GPIO_PIN_SET) ? 1 : 0;
			bity_enkodera[2] = (HAL_GPIO_ReadPin(E3_GPIO_Port, E3_Pin) == GPIO_PIN_SET) ? 1 : 0;
			bity_enkodera[3] = (HAL_GPIO_ReadPin(E4_GPIO_Port, E4_Pin) == GPIO_PIN_SET) ? 1 : 0;
			bity_enkodera[4] = (HAL_GPIO_ReadPin(E5_GPIO_Port, E5_Pin) == GPIO_PIN_SET) ? 1 : 0;
			bity_enkodera[5] = (HAL_GPIO_ReadPin(E6_GPIO_Port, E6_Pin) == GPIO_PIN_SET) ? 1 : 0;
			bity_enkodera[6] = (HAL_GPIO_ReadPin(E7_GPIO_Port, E7_Pin) == GPIO_PIN_SET) ? 1 : 0;
			bity_enkodera[7] = (HAL_GPIO_ReadPin(E8_GPIO_Port, E8_Pin) == GPIO_PIN_SET) ? 1 : 0;

			dec=bity_enkodera[0]*1 + bity_enkodera[1]*2 + bity_enkodera[2]*4 + bity_enkodera[3]*8 + bity_enkodera[4]*16 + bity_enkodera[5]*32 + bity_enkodera[6]*64 + bity_enkodera[7]*128;

			//y=Odczytaj_Kat_Enkodera(dec);
			y = AbsoluteEncoder_getContinuousAngle();
			x = NormalizeAngle(y - Wheel.zero_angle);

			// Kontrola silnika krokowego
			//HAL_GPIO_WritePin(KRK_EN_GPIO_Port, KRK_EN_Pin, RESET);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, StepperMotor(&Wheel, x));
			flag_50ms = 1;
		}
		if (cnt % 10 == 0)
		{
			flag_100ms = 1;
			ReadVelocityFromPGPin_Hall(PG_counter,htim3.Instance->CNT,&Wheel); //ta funkcja oblicza predkosc z halli czyli enkoder_mode i pinu PG

			PID(&Wheel);//wywołanie PID
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,abs(Wheel.motor_siganl));//ustawienie PWM na koła
			HAL_GPIO_WritePin(BLDC_FR_GPIO_Port, BLDC_FR_Pin,(Wheel.motor_siganl>0)?RESET:SET);//ustawinie kierunku w zależnosci od znaku predkosci

			PackWheelJointStateData(TxData, x, &Wheel); //pakowanie ramki do wysyłki

			// WPISANIE WŁAŚCIWEGO ID KOŁA TUŻ PRZED WYSYŁKĄ
			TxHeader.StdId = 0x100 + tab[4];

			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
			    {
			        HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
			    }

			    // Teraz bezpiecznie wysyłamy najnowszą ramkę cykliczną
			    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
			    {
			        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			    }
		}
		if (cnt % 100 == 0)
		{
			//y = AbsoluteEncoder_getContinuousAngle();
			flag_1000ms = 1;
		}
		if (cnt >= 200) cnt = 0;
		if(Wheel.motor_siganl!=0)//to zapobiega tykaniu oraz powoduje że koło odrazu sie zatrzymuje
		{
			HAL_GPIO_WritePin(BLDC_BK_GPIO_Port, BLDC_BK_Pin, RESET);
		}
		if(Wheel.motor_siganl==0)
		{
			HAL_GPIO_WritePin(BLDC_BK_GPIO_Port, BLDC_BK_Pin, SET);
		}
	}
}

void emergency_stop(void)
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0); //Ustawiam PWM na silnik BLDC na zero
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); //Ustawiam PWM na silnik krokowy na zero
	HAL_GPIO_WritePin(KRK_RESET_GPIO_Port,KRK_RESET_Pin, RESET);
	HAL_GPIO_WritePin(KRK_RESET_GPIO_Port,KRK_RESET_Pin, SET);
	HAL_GPIO_WritePin(KRK_EN_GPIO_Port, KRK_EN_Pin, SET);
	HAL_GPIO_WritePin(BLDC_BK_GPIO_Port, BLDC_BK_Pin, SET);
}


void PackWheelJointStateData(uint8_t *target_buffer, float current_angle_deg, WHEEL_Type *key)
{
	// 1. Normalizacja kąta do zakresu -180 do +180 stopni
	    while (current_angle_deg > 180.0f)  current_angle_deg -= 360.0f;
	    while (current_angle_deg < -180.0f) current_angle_deg += 360.0f;

	    // 2. Konwersja na radiany (zakres od -3.1415 do +3.1415)
	    float angle_rad = current_angle_deg * (M_PI / 180.0f);

	    // Mnożnik 10000 da nam zakres od -31415 do +31415.
	    // Idealnie mieści się to w int16_t (który ma zakres -32768 do 32767)!
	    int16_t msg_steering_pos = (int16_t)(angle_rad * 10000.0f);

	    // 3. Pozycja obrotowa koła w radianach (mnożnik 10000)
	    int32_t msg_wheel_pos = (int32_t)(key->bldc_pg_data.Turns * 10000.0f);

	    // 4. Konwersja prędkości z [obr/s] na [rad/s] (mnożnik 1000)
	    float omega_rad_s = key->bldc_pg_data.RotationalSpeed * 2.0f * M_PI;
	    int16_t msg_wheel_vel = (int16_t)(omega_rad_s * 1000.0f);

	    // Kopiowanie do bufora CAN
	    memcpy(&target_buffer[0], &msg_steering_pos, 2);
	    memcpy(&target_buffer[2], &msg_wheel_pos, 4);
	    memcpy(&target_buffer[6], &msg_wheel_vel, 2);
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
