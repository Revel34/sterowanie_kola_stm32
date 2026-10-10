/**
  ******************************************************************************
  * @file    AbsoluteEncoder_Driver.c
  * @brief   Driver dla enkodera absolutnego LIKA C50MA-BG1-17-10-L2.
  *          PARAMETRY ENKODERA - SSI w kodzie binarnym, rozdzielczość 17bitów, jednoobrotowy
  ******************************************************************************
  * @author Mateusz Sałęga
  ******************************************************************************
  */
 #include "main.h"
 #include "spi.h"

 #include "ABS_ENCODER.h"

 #define ENCODER_MAX_VALUE   (131072.0f) // 2^17
 #define DATA_SIZE           (3u)
 #define FULL_ANGLE          (360.0f)
 #define DEFALUT_VALUE       (1111.1f)

 static HAL_StatusTypeDef SPI_ReadSSI(uint32_t* value)
 {
     uint8_t receivedData[DATA_SIZE] = {0u}; // Bufor na 3 bajty
     uint32_t u32result = 0u;
     HAL_StatusTypeDef status = HAL_OK;

     status = HAL_SPI_Receive(&hspi2, receivedData, DATA_SIZE, HAL_MAX_DELAY);

     for (int i = 0; i < DATA_SIZE; i++)
     {
         u32result <<= 8;                // Przesunięcie wyniku
         u32result |= receivedData[i];   // Dołączenie kolejnego bajtu
     }

     // Odrzucenie najstarszego bitu i pozostawienie 17-bitowej wartości
     u32result &= 0x7FFFFF; // tylko 23 bity
     u32result >>= 6;       // zostaje 17 najstarszych bitów

     *value = u32result;

     return status;
 }

 static float ConvertToAngle(uint32_t rawValue)
 {
     return ((float)rawValue / ENCODER_MAX_VALUE) * FULL_ANGLE;
 }

 float AbsoluteEncoder_readAngle(void)
 {
     HAL_StatusTypeDef status = HAL_OK;
     uint32_t rawValue = 0u;
     float retValue = 0.0f;

     status = SPI_ReadSSI(&rawValue);

     if (status == HAL_OK)
     {
         retValue = ConvertToAngle(rawValue);
     }
     else
     {
         retValue = DEFALUT_VALUE;
     }
     return retValue;
 }

 float AbsoluteEncoder_getContinuousAngle(void)
 {
     static float continuousAngle = 0.0f;    // kąt absolutny
     static float lastRawAngle = 0.0f;       // ostatni kąt 0–360°
     float rawAngle = AbsoluteEncoder_readAngle();   // aktualny 0–360

     float diff = rawAngle - lastRawAngle;

     // Korekcja przejścia przez 0/360
     if (diff > 180.0f)
         diff -= 360.0f;
     else if (diff < -180.0f)
         diff += 360.0f;

     continuousAngle += diff;   // kumulacja obrotów
     lastRawAngle = rawAngle;

     return continuousAngle;
 }

