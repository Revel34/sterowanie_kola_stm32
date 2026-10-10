/*
 * BLDC_MOTOR.h
 *  to kod od sterowania silnikiem BLDC Maxon
 *  Created on: Apr 14, 2025
 *      Author: Andrzej Ozga
 */

#ifndef INC_BLDC_MOTOR_H_
#define INC_BLDC_MOTOR_H_

#include <stdint.h>
#include "ALL_STRUCT.h"          //to moja bibliteka od ogólnego sterowania

void Motor(void);
void IncrementalEncoder_readVelocity(uint32_t EncoderSiganl,WHEEL_Type* key);      //tworzę funkcje która oblicza aktualną prędkość(przyjumje wartosci z timera i kolo)

void ReadVelocityFromPGPin_Hall(uint32_t PG_counter,uint32_t EncoderSignal,WHEEL_Type* key);

#endif /* INC_BLDC_MOTOR_H_ */

