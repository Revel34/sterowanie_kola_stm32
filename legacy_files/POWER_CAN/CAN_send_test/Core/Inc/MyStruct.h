/*
 * MyStruct.h
 *
 *  Created on: AUG 20, 2025
 *      Author: Andrzej Ozga
 */
#include <stdint.h>
#include "stm32f4xx_hal.h"
#ifndef INC_MYSTRUCT_H_
#define INC_MYSTRUCT_H_

//w tym pliku bede umieszczal struktury i funkcje

#define MOTOR_SETTING_TIME 200        //to czas co ile wywołuje funkcje od ustawiania synału na silniki - potem zmiana na ms
#define FRAME_TIME 1
#define PWM_MOTOR_MIN -1000
#define PWM_MOTOR_MAX 1000


typedef enum //tworzę nowy typ zmiennej enumerycznej - ELETRIC_STATE_Type - czyli stan w jakim znajduje sie dany obwód
{
	OFF=0,              //to stan wyłączony
	ON                  //to stan włączony
 }ELECTRIC_STATE_Type;

typedef struct //toworze strukture-ELECTRIC_CIRCUIT_Type sa tutaj wszytkie parametry obwodu elektrycznego
{
	char                  name[10];         //to tablica 10el z nazwą
	ELECTRIC_STATE_Type   current_state;    //to struktura z aktualnym stanem
	int                   turned_on;        //to zmienna mówiąca ile razy włączono obwód
	int 				  turned_off;		//to zmienna mówiąca ile razy wyłączono obwód

	GPIO_TypeDef*       DirectionPort;     //to wskaźnik na port od obwodu elektrycznego
	uint16_t            DirectionPin;      //to pin od obwodu elektrycznego
}ELECTRIC_CIRCUIT_Type;

void TurnOn(ELECTRIC_CIRCUIT_Type* key);   // tworze funkcje włączającą
void TurnOff(ELECTRIC_CIRCUIT_Type* key);  // tworze funkcję wyłączającą
void Continius_ON_OFF(ELECTRIC_CIRCUIT_Type* key); //to funkcja do ciaglego właczania

#endif /* INC_MYSTRUCT_H_ */
