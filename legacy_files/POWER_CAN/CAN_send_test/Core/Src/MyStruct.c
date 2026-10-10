/*
 * MyStruct.c
 *
 *  Created on: AUG 20, 2025
 *      Author: Andrzej Ozga
 */
#include "MyStruct.h"
#include "main.h"

void TurnOn(ELECTRIC_CIRCUIT_Type* key)
{
	HAL_GPIO_WritePin(key->DirectionPort,key->DirectionPin, SET);
	key->current_state=ON;
	key->turned_on++;
}
void TurnOff(ELECTRIC_CIRCUIT_Type* key)
{
	HAL_GPIO_WritePin(key->DirectionPort,key->DirectionPin, RESET);
	key->current_state=OFF;
	key->turned_off++;
}
void Continius_ON_OFF(ELECTRIC_CIRCUIT_Type* key)
{
	if (key->current_state == ON)
		HAL_GPIO_WritePin(key->DirectionPort,key->DirectionPin, SET);
	else
		HAL_GPIO_WritePin(key->DirectionPort,key->DirectionPin, RESET);
}
