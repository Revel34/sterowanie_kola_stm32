/*
 * BLDC_MOTOR.c
 * to kod od sterowania silnikiem BLDC Maxon
 *  Created on: Apr 14, 2025
 *      Author: Andrzej Ozga
 */
#include "BLDC_MOTOR.h"
#include "main.h"

void IncrementalEncoder_readVelocity(uint32_t EncoderSignal,WHEEL_Type* key)      //tworzę funkcje która oblicza aktualną prędkość(przyjumje wartosci z timera i kolo)
{
		int32_t delta = (int32_t)EncoderSignal - (int32_t)key->encoder_data.LastValue;
		float sample_time_sec = MOTOR_SETTING_TIME / 1000.0f;
		    // Obsługa przepełnienia (overflow) 16-bitowego licznika timera
		    if (delta < -32768)  // zakładamy, że timer ma zakres 0 - 65535
		    {
		        delta += 65536;
		    }
		    else if (delta > 32768)
		    {
		        delta -= 65536;
		    }
		    key->encoder_data.LastValue = EncoderSignal;
		    key->encoder_data.TotalTurns += abs(delta);
		    //key->encoder_data.RotationalSpeed=delta/(4.0f*ENCODER_RESOLUTION*GEAR_RATIO*sample_time_sec); //obliczam predkosc obr w [obr/s]
		    key->encoder_data.LinearSpeed=key->encoder_data.RotationalSpeed*WHEEL_TRAVEL_DISTANCE;        //obliczam predkosc obr w [m/s]
}

void ReadVelocityFromPGPin_Hall(uint32_t PG_counter,uint32_t EncoderSignal,WHEEL_Type* key) //ta funkcja oblicza predkosc z halli czyli enkoder_mode i pinu PG
{
	int32_t delta = (int32_t)EncoderSignal - (int32_t)key->bldc_pg_data.LastValue;   // to poniżej jest dla enkoder mode
	// Obsługa przepełnienia (overflow) 16-bitowego licznika timera
	if (delta < -32768)  // zakładamy, że timer ma zakres 0 - 65535
	{
		delta += 65536;
	}
	else if (delta > 32768)
	{
		delta -= 65536;
	}
	key->bldc_pg_data.LastValue = EncoderSignal;


	int32_t delta_1 = (int32_t)PG_counter - (int32_t)key->bldc_pg_data.LastValue_1; //obliczam delte czyli zmiane od ostaniego razu
	// Obsługa przepełnienia (overflow) 16-bitowego licznika timera
    if (delta_1 < -32768)  // zakładamy, że timer ma zakres 0 - 65535
    {
	    //delta_1 += 65536;
	}
    else if (delta_1 > 32768)
    {
    	//delta_1 -= 65536;
    }
    key->bldc_pg_data.LastValue_1 = PG_counter;
    delta_1=0;//dołożone

    float sample_time_sec = PG_SETTING_TIME / 1000.0f;
    if(delta<0)
    {
    	 key->bldc_pg_data.RotationalSpeed = ((float)delta - (float)delta_1) / (28 * GEAR_RATIO * sample_time_sec); //obliczam predkosc obrotowa [obr/sek]  PG_RESOLTION
    	 key->bldc_pg_data.TotalTurns = key->bldc_pg_data.TotalTurns + delta - delta_1;// zliczanie wszystkich impulsów
    	 key->bldc_pg_data.Turns += ((float)delta - (float)delta_1) / (28 * GEAR_RATIO) * 2.0f * PI;
    }
    else if(delta>0)
    {
    	 key->bldc_pg_data.RotationalSpeed = ((float)delta + (float)delta_1) / (21 * GEAR_RATIO * sample_time_sec); //obliczam predkosc obrotowa [obr/sek]
    	 key->bldc_pg_data.TotalTurns = key->bldc_pg_data.TotalTurns + delta + delta_1; // zliczanie wszystkich impulsów
    	 key->bldc_pg_data.Turns += ((float)delta + (float)delta_1)/ (28 * GEAR_RATIO) * 2.0f * PI;
    }
    else if(delta==0)
	{
    	key->bldc_pg_data.RotationalSpeed = 0; //gdy delta wynosi zero to predkosc wynosi zero
	}

    // --- NOWY WARUNEK: Anty-przepełnieniowe zerowanie pozycji ---
        // 200000.0 radiana to bezpieczny limit przed przekroczeniem zakresu int32_t w CAN (ok. 26 km jazdy)
        if (fabs(key->bldc_pg_data.Turns) >= 200000.0f)
        {
             key->bldc_pg_data.Turns = 0.0f;
        }
        // Ograniczenie za dużych prędkości
        if (fabs(key->bldc_pg_data.RotationalSpeed) > 4.0f)
        {
            key->bldc_pg_data.RotationalSpeed = 0.0f;
        }

    key->bldc_pg_data.LinearSpeed =key->bldc_pg_data.RotationalSpeed * WHEEL_TRAVEL_DISTANCE;  // prędkość liniowa z obrotowej
}


