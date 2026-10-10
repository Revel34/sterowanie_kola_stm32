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
#include <stdlib.h> // Wymagane do funkcji abs() (wartość bezwzględna) używanej w sterowaniu silnikami
#include <stdbool.h> // Wymagane dla logiki bool z oryginalnego kodu
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t id;
    GPIO_TypeDef *step_port;   uint16_t step_pin;
    GPIO_TypeDef *dir_port;    uint16_t dir_pin;
    GPIO_TypeDef *en_port;     uint16_t en_pin;
    UART_HandleTypeDef *huart;
    volatile int32_t steps_to_go;
    uint32_t speed_cnt;
    uint32_t speed_reload;
    bool step_state;
} StepperTMC_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- TMC STEPPERS ---
#define TMC_REG_GCONF      0x00
#define TMC_REG_IHOLD_IRUN 0x10
#define TMC_REG_CHOPCONF   0x6C
#define TMC_ADDRESS        0x00

// --- WAVESHARE / ST3215 BUS SERVO DEFINES ---
#define INST_WRITE          0x03
#define SMS_STS_ACC         41
#define STS_POS_MIN         0
#define STS_POS_MAX         4095
#define STS_DEFAULT_SPEED   2000    
#define STS_DEFAULT_ACC     50      
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

// --- NOWE ZMIENNE DLA RGB ---
volatile uint8_t LED_R = 255;
volatile uint8_t LED_G = 0;
volatile uint8_t LED_B = 0;

// --- NOWE ZMIENNE DLA SILNIKÓW (ID 0x201) ---
volatile int8_t motor_1 = 0;
volatile int8_t motor_2 = 0;
volatile int8_t motor_3 = 0;
volatile int8_t motor_4 = 0;

// --- NOWE ZMIENNE DLA SERW PWM (ID 0x202) ---
volatile uint8_t servo_1 = 90;
volatile uint8_t servo_2 = 90;
volatile uint8_t servo_3 = 90;
volatile uint8_t servo_4 = 90;

// --- NOWE ZMIENNE DLA SERW UART (ID 0x204) ---
volatile uint8_t uart_servo_id = 0;
volatile uint16_t uart_servo_angle = 0;
uint16_t g = 0;

// --- FLAGI DLA FUNKCJI BLOKUJĄCYCH W PĘTLI GŁÓWNEJ ---
volatile uint8_t flag_new_tmc_cmd = 0;
volatile uint8_t flag_new_uart_servo_cmd = 0;

// --- ZMIENNA DO WATCHDOGA CAN ---
volatile uint32_t last_can_rx_time = 0;

StepperTMC_t tmc1;
StepperTMC_t tmc2;

// --- NOWE ZMIENNE DLA DRIVERÓW TMC (ID 0x203) ---
volatile uint8_t  tmc_id = 0;
volatile int32_t  tmc_steps = 0;
volatile uint8_t  tmc_direction = 0;
volatile uint16_t tmc_speed_reload = 0;

TIM_HandleTypeDef htim5;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MX_TIM5_Init_Custom(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MX_TIM5_Init_Custom(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Włączamy taktowanie TIM5 w RCC
  __HAL_RCC_TIM5_CLK_ENABLE();
  
  // Włączamy przerwania w NVIC
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
}


// Funkcja aktualizująca jasność diod RGB na podstawie danych odebranych z magistrali CAN
void Update_RGB_LEDs(void)
{
    // Maksymalna wartość koloru przychodząca z CAN to 255. 
    // Nasze timery PWM pracują w zakresie od 0 do 9999 (co daje rozdzielczość 10000 kroków).
    // Przeliczamy (skalujemy) wartość 0-255 na 0-5000 (limit 50% mocy ze względów bezpieczeństwa).
    
    // Czerwony i Niebieski są na zwykłych kanałach PWM - im wyższa liczba, tym jaśniej.
    uint32_t pulse_r = (LED_R * 5000) / 255;
    uint32_t pulse_b = (LED_B * 5000) / 255;

    // Zielony jest na kanale KOMPLEMENTARNYM (TIM1_CH3N). Logika jest ODWRÓCONA.
    // 100% wypełnienia z głównego timera = 0% na pinie. 50% = 50%.
    // Dlatego od maksimum (10000) odejmujemy wyliczoną wartość, by zgaszone LED_G=0 dało 0V na wyjściu.
    uint32_t pulse_g = 10000 - ((LED_G * 5000) / 255);

    // Aktualizacja rejestrów CCR (Capture/Compare Register) timerów, co fizycznie zmienia szerokość impulsu PWM (Duty Cycle)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse_r);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_g); 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse_b);
}

// Funkcja sterująca mostkami H (silnikami prądu stałego DC)
void Update_DC_Motors(void)
{
    // Silnik 1 (m0)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, motor_1 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, abs(motor_1) * 100);

    // Silnik 2 (m1)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, motor_2 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, abs(motor_2) * 100);

    // Silnik 3 (m2)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, motor_3 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, abs(motor_3) * 100);

    // Silnik 4 (m3)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, motor_4 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, abs(motor_4) * 100);
}

// Funkcja aktualizująca wychylenie klasycznych serwomechanizmów (PWM)
void Update_PWM_Servos(void) 
{
    // Dzięki temu, że Timer tyka dokładnie 1 milion razy na sekundę (1 MHz),
    // jedna jednostka w rejestrze PWM to równo 1 mikrosekunda (1 us) w czasie rzeczywistym.
    // Standardowe serwa pracują w impulsach od 500 us (0 stopni) do 2400 us (180 stopni).
    // Poniższa matematyka to po prostu funkcja liniowa mapująca kąt 0-180 na mikrosekundy 500-2400.
    
    uint32_t pulse_s1 = 500 + (servo_1 * (2400 - 500) / 180);
    uint32_t pulse_s2 = 500 + (servo_2 * (2400 - 500) / 180);
    uint32_t pulse_s3 = 500 + (servo_3 * (2400 - 500) / 180);
    uint32_t pulse_s4 = 500 + (servo_4 * (2400 - 500) / 180);

    // Wysłanie wyliczonych mikrosekund bezpośrednio do odpowiednich timerów i kanałów przypisanych serwom
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse_s1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_s2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_s3);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pulse_s4);
}

// ==========================================================
// ===== ZAAWANSOWANE SERWA UART (WAVESHARE ST3215) =========
// ==========================================================

// Rozkładamy 16 bitów na dwa oddzielne bajty (Little Endian)
void Host2Servo(uint8_t *dataL, uint8_t *dataH, uint16_t data) {
  *dataL = (data & 0xFF);
  *dataH = ((data >> 8) & 0xFF);
}

// Suma kontrolna zapobiegająca przekłamaniom na kablu
uint8_t ServoCalculateChecksum(uint8_t *buffer, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum += buffer[i];
  return ~sum;
}

// Budowa fizycznej ramki polecenia (Pakiet UART)
void ServoWritePosition(uint8_t id, uint16_t position, uint16_t speed, uint8_t acc)
{
  uint8_t txBuf[14];
  uint8_t posL, posH, spdL, spdH;

  Host2Servo(&posL, &posH, position);
  Host2Servo(&spdL, &spdH, speed);

  txBuf[0] = 0xFF; // Header
  txBuf[1] = 0xFF; // Header
  txBuf[2] = id;   // ID

  // Length = Parameters + 2
  // Parametry: Addr(1) + Acc(1) + Pos(2) + Time(2) + Speed(2) = 8 bajtów
  // Length = 8 + 2 = 10 (0x0A)
  txBuf[3] = 0x0A;

  txBuf[4] = INST_WRITE;
  txBuf[5] = SMS_STS_ACC; // Start Address (41) - Przyspieszenie
  txBuf[6] = acc;         // P41
  txBuf[7] = posL;        // P42 (Goal Pos Low)
  txBuf[8] = posH;        // P43 (Goal Pos High)
  txBuf[9] = 0x00;        // P44 (Time Low - 0 oznacza ruch bazowany na prędkości)
  txBuf[10] = 0x00;       // P45 (Time High)
  txBuf[11] = spdL;       // P46 (Speed Low)
  txBuf[12] = spdH;       // P47 (Speed High)

  // Checksum liczone od ID do końca danych
  txBuf[13] = ServoCalculateChecksum(&txBuf[2], 11);

  // ZWRÓĆ UWAGĘ: Timeout 20ms sprawia, że jest to kod silnie "blokujący" czas.
  HAL_UART_Transmit(&huart1, txBuf, 14, 20);
}

// Funkcja aktywująca "trzymanie" i gotowość do ruchu (Torque Enable)
void ServoEnableTorque(uint8_t id, uint8_t enable)
{
  uint8_t txBuf[8];
  txBuf[0] = 0xFF;
  txBuf[1] = 0xFF;
  txBuf[2] = id;
  txBuf[3] = 0x04; // Length: Addr(1) + Val(1) + 2 = 4
  txBuf[4] = INST_WRITE;
  txBuf[5] = 40;   // Address 40 (Torque Enable)
  txBuf[6] = enable ? 1 : 0;
  txBuf[7] = ServoCalculateChecksum(&txBuf[2], 5);
  HAL_UART_Transmit(&huart1, txBuf, 8, 10);
}

// Wywoływane z pętli while(1), bezpiecznie sprawdza, czy przyszła flaga z przerwania CAN.
void Process_Bus_Servo_Command(void)
{
    if (flag_new_uart_servo_cmd)
    {
        flag_new_uart_servo_cmd = 0; // Kasujemy flagę

        // Atomowy odczyt współdzielonych zmiennych
        __disable_irq();
        uint8_t local_id = uart_servo_id;
        uint16_t local_angle = uart_servo_angle;
        __enable_irq();

        // Jeśli wysyłasz ID = 0 z ROSa, domyślnie serwa Waveshare mają ID = 1.
        // Używamy 0xFE (Broadcast), żeby "złapało" każde podpięte serwo bez względu na jego własne ID.
        if (local_id == 0) {
            local_id = 0xFE; 
        }

        // Ustawiamy sztywne, bezpieczne ramy (mapowanie 0-180 na kroki 0-2047, ponieważ pełny zakres 4095 to 360 stopni)
        uint16_t target_pos = (uint16_t)((local_angle * STS_POS_MAX) / 360);

        // Aktywujemy moment obrotowy (aby serwo mogło się ruszać i trzymać pozycję)
        ServoEnableTorque(local_id, 1);
        
        // Zabezpieczenie przed przepełnieniem buforów i kolizjami sygnału
        HAL_Delay(1);

        // Wysyłamy polecenie ruchu (używamy huart1)
        ServoWritePosition(local_id, target_pos, STS_DEFAULT_SPEED, STS_DEFAULT_ACC);
    }
}

// ==========================================================
// ===== SILNIKI KROKOWE (STEROWNIKI TMC po UART) ===========
// ==========================================================

uint8_t calcCRC(uint8_t *datagram, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t currentByte = datagram[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) crc = (crc << 1) ^ 0x07;
            else crc = (crc << 1);
            currentByte = currentByte >> 1;
        }
    }
    return crc;
}

void TMC_Write(UART_HandleTypeDef *huart, uint8_t reg, uint32_t data) {
    uint8_t frame[8];
    frame[0] = 0x05; frame[1] = TMC_ADDRESS; frame[2] = reg | 0x80;
    frame[3] = (data >> 24) & 0xFF; frame[4] = (data >> 16) & 0xFF;
    frame[5] = (data >> 8) & 0xFF;  frame[6] = data & 0xFF;
    frame[7] = calcCRC(frame, 7);
    HAL_UART_Transmit(huart, frame, 8, 10);
}

void TMC_Hardware_Init(StepperTMC_t *tmc) {
    // Aktywacja układu i wgranie konfiguracji GCONF
    HAL_GPIO_WritePin(tmc->en_port, tmc->en_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    TMC_Write(tmc->huart, TMC_REG_GCONF, 0x000000C1);
}

void Process_TMC_Command(void)
{
    if (flag_new_tmc_cmd)
    {
        flag_new_tmc_cmd = 0; // Kasujemy flagę

        // Atomowy odczyt zmiennych z CAN
        __disable_irq();
        uint8_t local_id = tmc_id;
        uint8_t local_dir = tmc_direction;
        uint16_t local_reload = tmc_speed_reload;
        int32_t local_steps = tmc_steps;
        __enable_irq();

        StepperTMC_t *target = NULL;
        if (local_id == 1) target = &tmc1;
        else if (local_id == 2) target = &tmc2;

        if (target != NULL) {
            // 1. Ustawiamy DIR Pin sprzętowo (obrót w lewo/prawo)
            HAL_GPIO_WritePin(target->dir_port, target->dir_pin, local_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
            
            // 2. Transmitujemy konfigurację CHOPCONF (UART) - twardo na profil stealthChop jak w oryginale
            TMC_Write(target->huart, TMC_REG_CHOPCONF, 0x10000053);
            
            // 3. Atomowe zapisanie nowych parametrów docelowych (używane przez TIM5_IRQHandler)
            __disable_irq();
            target->speed_reload = local_reload;
            target->steps_to_go = local_steps;
            __enable_irq();
        }
    }
}

// Funkcja sprawdzająca utratę połączenia z głównym komputerem (Watchdog)
void Check_CAN_Timeout(void)
{
    // Zabezpieczenie: jeśli od ponad 1000 milisekund (1 sekundy) nie przyszła
    // żadna nowa wiadomość CAN, natychmiast zerujemy wysterowanie układów ruchomych.
    
    // --- FUNKCJA ZAKOMENTOWANA NA PROŚBĘ UŻYTKOWNIKA (DO TESTÓW) ---
    // Aby z powrotem włączyć bezpiecznik po utracie sygnału, po prostu usuń /* i */.
    
    /*
    if ((HAL_GetTick() - last_can_rx_time) > 1000)
    {
        // Wyłączamy silniki główne (H-Bridge)
        motor_1 = 0;
        motor_2 = 0;
        motor_3 = 0;
        motor_4 = 0;
        
        // Zatrzymujemy silniki krokowe TMC
        tmc_steps = 0;
        tmc_speed_reload = 0;

        // Ustawiamy "awaryjny" kolor LED na czerwony
        LED_R = 255;
        LED_G = 0;
        LED_B = 0;
    }
    */
}

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
  MX_CAN1_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  // --- RESTRYKCYJNA KONFIGURACJA FILTRA TYLKO DLA ZAKRESU 0x200 - 0x207 ---
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

    // [OKNO 1] Obsługuje zakres peryferiów 0x200 - 0x207
    sFilterConfig.FilterIdHigh = (0x200 << 5);
    sFilterConfig.FilterMaskIdHigh = (0x7F8 << 5); // Maska 0x7F8 pilnuje, by bity pasowały do 0x20X

    // [OKNO 2] Kopiujemy dokładnie to samo ustawienie, aby zamknąć "otwartą bramkę"
    // Dzięki temu dolna część filtra nie przepuści przypadkowych ramek (np. 0x100)
    sFilterConfig.FilterIdLow = (0x200 << 5);
    sFilterConfig.FilterMaskIdLow = (0x7F8 << 5);

    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

  /*// --- KONFIGURACJA FILTRA SPRZĘTOWEGO (TYLKO ID 0x100) ---
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
    sFilterConfig.SlaveStartFilterBank = 14;*/

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
       Error_Handler();
    }

    // 1. Konfiguracja struktury ramki nadawczej CAN
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x210;               // <--- ZMIENIONE Z 0x123 NA 0x100!
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

    // === URUCHOMIENIE GENERACJI SPRZĘTOWEJ PWM ===
    // Aby mikrokontroler wysyłał sygnał na piny, musimy fizycznie aktywować generację PWM 
    // dla wszystkich używanych Timerów i przypisanych do nich kanałów.
    
    // PWM dla LED RGB
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // RED
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // GREEN
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); // GREEN (Complementary pin)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // BLUE

    // PWM dla mostków H i Serwomechanizmów
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    // Start Timer-a bazowego, który zgłasza przerwania (u nas docelowo co 10ms)
    // UWAGA: Obecnie jego prescaler w tim.c wynosi 8999, co daje ~10.7ms. By było to równo 10ms, prescaler powinien wynosić 8399.
    HAL_TIM_Base_Start_IT(&htim14); 

    // === INICJALIZACJA STRUKTUR TMC ===
    // Wpisujemy tu piny z CubeIDE, które przypisałeś dla TMC (tak jak było w starym pliku)
    tmc1 = (StepperTMC_t){.id=1, .step_port=GPIOA, .step_pin=GPIO_PIN_15, .dir_port=GPIOB, .dir_pin=GPIO_PIN_7, .en_port=GPIOC, .en_pin=GPIO_PIN_4, .huart=&huart3};
    tmc2 = (StepperTMC_t){.id=2, .step_port=GPIOA, .step_pin=GPIO_PIN_6,  .dir_port=GPIOA, .dir_pin=GPIO_PIN_7, .en_port=GPIOA, .en_pin=GPIO_PIN_8, .huart=&huart5};
    
    // Start komunikacji sprzętowej z TMC (ustawienie GCONF po UART)
    TMC_Hardware_Init(&tmc1);
    TMC_Hardware_Init(&tmc2);

    MX_TIM5_Init_Custom();
    HAL_TIM_Base_Start_IT(&htim5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Obsługa flag komunikacyjnych (przeniesiona z przerwań do pętli głównej).
      // Funkcje UART Transmit mają timeouty oparte na HAL_GetTick(). Wywoływanie ich z przerwania
      // o tym samym priorytecie co SysTick (Priorytet 0) powodowało zablokowanie inkrementacji 
      // HAL_GetTick(), przez co timeouty nigdy nie wygasały (System zawieszał się na zawsze!).
      // W pętli głównej SysTick działa poprawnie, więc układ nigdy się nie zawiesi.
      Process_Bus_Servo_Command();
      Process_TMC_Command();

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
            // Kopiujemy ID do lokalnej zmiennej, zapobiegając problemom z optymalizacją rejestrów przez kompilator
        	volatile uint16_t received_id = RxHeader.StdId;
            g = received_id;
            // Mignięcie diodą na płytce przy odebraniu JAKIEJKOLWIEK ramki, która przeszła przez filtr
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

            // --- ZAPIS ZNACZNIKA CZASU (WATCHDOG) ---
            // Odnotowujemy dokładny czas (w milisekundach) nadejścia ostatniej poprawnej ramki
            last_can_rx_time = HAL_GetTick();

            // Klasyczna drabina IF-ELSE zamiast SWITCH
            if (received_id == 0x200) // --- RGB LED ---
            {
                LED_R = RxData[0];
                LED_G = RxData[1];
                LED_B = RxData[2];
            }
            else if (received_id == 0x201) // --- /Hbridge (Silniki DC) ---
            {
            	// Rzutujemy tablicę uint8_t na int8_t, żeby obsługiwać wartości ujemne
            	int8_t *signed_data = (int8_t*)RxData;

            	motor_1 = signed_data[0]; // Wartość od -100 do 100
                motor_2 = signed_data[1];
            	motor_3 = signed_data[2];
                motor_4 = signed_data[3];
            }
            else if (received_id == 0x202) // --- /Serwa (Standardowe PWM) ---
            {
            	servo_1 = RxData[0];
            	servo_2 = RxData[1];
                servo_3 = RxData[2];
            	servo_4 = RxData[3];

            	// Zabezpieczenie programowe, żeby nie przekroczyć 180 stopni
            	if (servo_1 > 180) servo_1 = 180;
            	if (servo_2 > 180) servo_2 = 180;
            	if (servo_3 > 180) servo_3 = 180;
            	if (servo_4 > 180) servo_4 = 180;
            }
            else if (received_id == 0x203) // --- /tmc_cmd (Driver TMC) ---
            {
            	// 1. Pobranie ID sterownika (1 bajt)
                tmc_id = RxData[0];

                // 2. Składanie 32-bitowej liczby kroków (Bajty 1, 2, 3, 4 - Little Endian)
                tmc_steps = (int32_t)RxData[1]         |
                ((int32_t)RxData[2] << 8)  |
                ((int32_t)RxData[3] << 16) |
                ((int32_t)RxData[4] << 24);

                // 3. Pobranie kierunku (1 bajt)
                tmc_direction = RxData[5];

                // 4. Składanie 16-bitowej prędkości/reload (Bajty 6, 7 - Little Endian)
                tmc_speed_reload = (uint16_t)RxData[6] | ((uint16_t)RxData[7] << 8);

                // --- USTAWIENIE FLAGI DLA PĘTLI GŁÓWNEJ ---
                flag_new_tmc_cmd = 1;
            }
            else if (received_id == 0x204) // --- /serwoUART (Bus Servo 0-180) ---
            {
            	// Pobieramy ID serwa (1 bajt)
                uart_servo_id = RxData[0];

                // Składamy kąt 16-bitowy z dwóch bajtów (Little Endian: najpierw młodszy, potem starszy)
                uart_servo_angle = (uint16_t)RxData[1] | ((uint16_t)RxData[2] << 8);

                // Zabezpieczenie programowe przed przekroczeniem fizycznej bariery!
                // Mechanika blokuje się na ~180 stopniach. Ograniczamy do 170, żeby serwo nie uderzyło w "ścianę" i nie weszło w błąd zasilania.
                if (uart_servo_angle > 170) uart_servo_angle = 170;

                // --- USTAWIENIE FLAGI DLA PĘTLI GŁÓWNEJ ---
                flag_new_uart_servo_cmd = 1;
            }
            else if (received_id == 0x205) // --- Przyszłościowe ID ---
            {

            }
        }
        else
        {
            // BARDZO WAŻNE: Jeśli wystąpi błąd sprzętowy CAN i GetRxMessage zwróci błąd (HAL_ERROR),
            // musimy przerwać pętlę, inaczej FIFO Fill Level zawsze będzie > 0, 
            // a procesor wpadnie w morderczą, nieskończoną pętlę zawieszając cały system!
            break;
        }
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        StepperTMC_t *motors[2] = {&tmc1, &tmc2};
        for(int i=0; i<2; i++) {
            if (motors[i]->steps_to_go > 0) {
                motors[i]->speed_cnt++;
                if (motors[i]->speed_cnt >= motors[i]->speed_reload) {
                    motors[i]->step_state = !motors[i]->step_state;
                    HAL_GPIO_WritePin(motors[i]->step_port, motors[i]->step_pin, motors[i]->step_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
                    motors[i]->speed_cnt = 0;
                    if(!motors[i]->step_state) motors[i]->steps_to_go--;
                }
            }
        }
    }
    
    if (htim->Instance == TIM14)
    {
    	static uint16_t cnt = 0;
    	cnt++;
    	// Tutaj wpisujemy rzeczy, które mają się wykonać przy każdym przerwaniu (ok. 10ms).
        // Wywołujemy nasze funkcje, by hardware gładko i cyklicznie aktualizował piny na podstawie tego, co odebrał CAN.
        // Niezależnie od obciążenia pętli głównej (while), te polecenia wykonają się zawsze "na czas".
        Update_RGB_LEDs();
        Update_DC_Motors();
        Update_PWM_Servos();
        Check_CAN_Timeout(); // Zabezpieczenie przed ucieczką również wywoływane niezawodnie z timera!

    	if (cnt % 5 == 0)//tutaj wpisać rzeczy które mają sie wykonać co 50ms
    	{
    		// Pozostawione puste - obsługa komend przeniesiona do while(1)
    		// aby zapobiec deadlockom (blokowaniu HAL_GetTick() wewnątrz przerwania).
   		}
    	if (cnt % 10 == 0)//tutaj wpisać rzeczy które mają sie wykonać co 100ms
    	{
    		// Sprawdzenie, czy w peryferium CAN jest wolne miejsce n	a nadanie nowej wiadomości
    		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
            {
    		    // Wysłanie ramki
    		    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    		    {
    		    	/* Obsługa błędu nadawania (np. brak fizycznego połączenia z inną jednostką w trybie Normal) */
    		    }
    		}
    	}
    	if (cnt % 100 == 0)//tutaj wpisać rzeczy które mają sie wykonać co 1000ms
    	{

    	}
    	if (cnt >= 200)//tutaj wpisać rzeczy które mają sie wykonać co 2000ms
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
  * @retval Nonels 
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
