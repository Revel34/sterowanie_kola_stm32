/*
 * ALL_STRUCT.c
 *
 *  Created on: Apr 14, 2025
 *      Author: lenovo
 */
#include "ALL_STRUCT.h"
#include "main.h"



void ReadNewCmdVel(float x_lin,float y_lin,float z_lin,float z_ang,WHEEL_Type *key, int tab[])    //ta funkcja interpretuje cmd vel na nasze sterowanie
{
	//będę używał tylko x_lin y_lin i z_ang oraz z_lin jako paramtr pomocniczy - do jazdy PID, PWM, lub X_STOP
	//prekosci liniowe to w m/s a rotacyjne w rad/s
	float velocity; // deklaruje zmienną do tymczasowego przechowania prekosci przed sprawdzeniem limitów
	ReadExtraParametres(z_lin, key);
	if(key->MOTOR_STATE!=X_STOP||((z_lin>-0.1)&&(z_lin<0.1)))//jeśl mamy stać w miejscu to nic nie liczymy
	{
		if((x_lin!=0)&&(y_lin==0)&&(z_ang==0))//to jazda do przodu/tyłu
		{
			key->MOVE_STATE=FORWARD;        //wybieram tryb jazdy - do przody/tyłu
			velocity=x_lin*tab[1];
			key->setpoint_velocity=VelocityLimit(velocity,100);   //ustawaim żądaną prędkosć
			key->setpoint_angle=0;          //ustawiam żądany kąt na 0stopni czyli do przodu
		}
		else if((x_lin==0)&&(y_lin!=0)&&(z_ang==0))//to tryb jazdy krab czyli prawo/lewo
		{
			key->MOVE_STATE=CRAB_WALK;      //wybieram tryb jazdy - krab czyli prawo/lewo
			velocity=y_lin*tab[2];
			key->setpoint_velocity=VelocityLimit(velocity,100);   //ustawaim żądaną prędkosć
			key->setpoint_angle=90;          //ustawiam żądany kąt na 90 stopni czyli w bok
		}
		else if((x_lin!=0)&&(y_lin!=0)&&(z_ang==0))//to tryb jazdy po przekątnej
		{
			key->MOVE_STATE=DIAGONAL;       //wybieram tryb jazdy po przekatnej
			velocity=sqrt(x_lin*x_lin+y_lin*y_lin);   //ustawaim żądaną prędkosć - to z Tw Pitagorasa
			//key->setpoint_angle=atan2(y_lin,x_lin)*180/PI;          //ustawiam żądany kąt
			AngleLimit(key,atan2(y_lin*tab[2],x_lin*tab[1])*180/PI,VelocityLimit(velocity,100));                     //to ogranicznie kata
		}
		else if((x_lin==0)&&(y_lin==0)&&(z_ang!=0))//to tryb jazdy wokół własnej osi
		{
			key->MOVE_STATE=AROUND;         //wybieram tryb jazdy wokół własnej osi
			velocity=0.5f * sqrtf(ROVER_LENGTH * ROVER_LENGTH +ROVER_WIDTH  * ROVER_WIDTH) * z_ang;// Obliczenie wspólnej prędkości liniowej narożnych kół
			key->setpoint_velocity=VelocityLimit(velocity,100);   //ustawaim żądaną prędkosć
			key->setpoint_angle=tab[3]*(atan2f(ROVER_LENGTH * 0.5f, ROVER_WIDTH * 0.5f))*180/PI; // Kąt skrętu względem środka
		}
		else if((x_lin!=0)&&(y_lin==0)&&(z_ang!=0))//to tryb jazdy połuku
		{
			key->MOVE_STATE=ROUND;          //wybieram tryb jazdy po łuku
			float R;
			R=fabsf(x_lin/z_ang);   //chce aby obliczny promień aby bez znaku
			float K;
			K=x_lin/fabsf(x_lin);   //K to tylko kierunek predkosci
			if(z_ang<0) //ruch zgdonie do wskazówek zegara
			{
				switch(tab[4])
				{
					case 1:   //to PP to ok
						key->setpoint_angle = atan2f(ROVER_LENGTH, R - ROVER_WIDTH * 0.5f) * 180.0f / PI;
						velocity=K*-fabsf(z_ang)*(sqrt( (R - ROVER_WIDTH/2)*(R - ROVER_WIDTH/2) + ROVER_LENGTH*ROVER_LENGTH));//Obliczenie prędkości liniowej narożnych kół
						break;
					case 2:   //to PT ok
						key->setpoint_angle = -atan2f(ROVER_LENGTH, R - ROVER_WIDTH * 0.5f) * 180.0f / PI;
						velocity=K*-fabsf(z_ang)*(sqrt( (R - ROVER_WIDTH/2)*(R - ROVER_WIDTH/2) + ROVER_LENGTH*ROVER_LENGTH ));//Obliczenie prędkości liniowej narożnych kół
						break;
					case 3:   //to LP
						key->setpoint_angle = atan2f(ROVER_LENGTH, R + ROVER_WIDTH * 0.5f) * 180.0f / PI;
						velocity=K*fabsf(z_ang)*(sqrt( (R + ROVER_WIDTH/2)*(R + ROVER_WIDTH/2) + ROVER_LENGTH*ROVER_LENGTH ));//Obliczenie prędkości liniowej narożnych kół
						break;
					case 4:   //to LT ok
						key->setpoint_angle = -atan2f(ROVER_LENGTH, R + ROVER_WIDTH * 0.5f) * 180.0f / PI;
						velocity=K*fabsf(z_ang)*(sqrt( (R + ROVER_WIDTH/2)*(R + ROVER_WIDTH/2) + ROVER_LENGTH*ROVER_LENGTH ));//Obliczenie prędkości liniowej narożnych kół
						break;
					default:// Możesz dać np. błąd lub wyzerowanie
						key->setpoint_angle = 0;
						velocity=0;
						break;
				}
			}
			if(z_ang>0) //ruch przeciwnie do wskazówek zegara
			{
				switch(tab[4])
				{
				case 1:  //to PP ok
					key->setpoint_angle = -atan2f(ROVER_LENGTH, R + ROVER_WIDTH * 0.5f) * 180.0f / PI;
					velocity=K*-fabsf(z_ang)*(sqrt( (R + ROVER_WIDTH/2)*(R + ROVER_WIDTH/2) + ROVER_LENGTH*ROVER_LENGTH ));//Obliczenie prędkości liniowej narożnych kół
					break;
				case 2:  //to PT ok
					key->setpoint_angle = atan2f(ROVER_LENGTH, R + ROVER_WIDTH * 0.5f) * 180.0f / PI;
					velocity=K*-fabsf(z_ang)*(sqrt( (R + ROVER_WIDTH/2)*(R + ROVER_WIDTH/2) + ROVER_LENGTH*ROVER_LENGTH ));//Obliczenie prędkości liniowej narożnych kół
					break;
				case 3:  //to LP
					key->setpoint_angle = -atan2f(ROVER_LENGTH, R - ROVER_WIDTH * 0.5f) * 180.0f / PI;
					velocity=K*fabsf(z_ang)*(sqrt( (R - ROVER_WIDTH/2)*(R - ROVER_WIDTH/2) + ROVER_LENGTH*ROVER_LENGTH ));//Obliczenie prędkości liniowej narożnych kół
					break;
				case 4:  //to LT ok
					key->setpoint_angle = atan2f(ROVER_LENGTH, R - ROVER_WIDTH * 0.5f) * 180.0f / PI;
					velocity=K*fabsf(z_ang)*(sqrt( (R - ROVER_WIDTH/2)*(R - ROVER_WIDTH/2) + ROVER_LENGTH*ROVER_LENGTH ));//Obliczenie prędkości liniowej narożnych kół
					break;
				default:// Możesz dać np. błąd lub wyzerowanie
					key->setpoint_angle = 0;
					velocity=0;
					break;
				}
			}
			key->setpoint_velocity = VelocityLimit(velocity, 100);  // wspólne dla wszystkich przypadków
		}
		else
		{
			key->MOVE_STATE=IDLE_2;        //to jeszce do przemyślenia
			key->setpoint_velocity=0;      //ustawaim żądaną prędkosć 0 bo nie ustalony
			key->setpoint_angle=0;         //ustawiam żądany kąt na 0stopni czyli do przodu
		}
	}
	else //tutaj mamy stać w miejscu czyli skrecam koła na literke X i predkosć 0
	{
		key->setpoint_velocity=0;      //ustawaim żądaną prędkosć 0
		key->setpoint_angle=45*tab[10];        //to pewnie do poprawienia w zaleznosci od koła
	}
}

void ReadExtraParametres(float z_lin, WHEEL_Type *key) // to funkcja do czytania trybów sterowania czyl z_lin
{
	if((z_lin>-0.1)&&(z_lin<0.1)) key->MOTOR_STATE=DRIVE_WITHOUT_PID;
	else if((z_lin>0.9)&&(z_lin<1.1)) key->MOTOR_STATE=DRIVE_WITH_PID;
	else if((z_lin>1.9)&&(z_lin<2.1)) key->MOTOR_STATE=X_STOP;
	else key->MOTOR_STATE=DRIVE_WITHOUT_PID;
}

void AngleLimit(WHEEL_Type *key, float AngelWithoutLimit,float velo)//to funkcja która obcina kąt do +90 i -90 i zwraca prekość z minusem lub plusem
{
	if((AngelWithoutLimit<=90)&&(AngelWithoutLimit>=-90))
		{
			key->setpoint_angle=AngelWithoutLimit;
			key->setpoint_velocity=velo;
		}
	else if(AngelWithoutLimit>90)
	{
		key->setpoint_angle=AngelWithoutLimit-180;
		key->setpoint_velocity=velo*(-1);
	}
	else if(AngelWithoutLimit<-90)
	{
		key->setpoint_angle=AngelWithoutLimit+180;
		key->setpoint_velocity=velo*(-1);
	}
};

float VelocityLimit(float vel, float limit) //to funkacja do ustawienia limitu predkosci
{
	vel = (vel > limit) ? limit : vel;
	vel = (vel < -limit ) ? -limit : vel;
	return vel;
}

void GenerateVelTab(WHEEL_Type *key, int tab[]) //uzupełniam tablice do płynnej zmiany prędkosci - wartosci są w procentach zmiany jaka ma nastapic

{
	key->zero_angle=tab[0];
/*    if (key == NULL)
        return;

    // --- PARAMETRY KRZYWEJ S ---
    const float total_time = 3.0f;        // czas rampowania [s] (możesz zmienić)
    const float k  = 6.0f;                // stromość (większe = bardziej strome)
    const float t0 = total_time * 0.5f;   // środek rampy (połowa czasu)
    const int   N  = VELOCITY_TAB_SIZE;   // liczba punktów w tablicy

    if (N <= 1)
        return;

    const float dt = total_time / (float)(N - 1);

    // --- GENEROWANIE TABLICY ---
    for (int i = 0; i < N; ++i)
    {
        float t   = i * dt;                         // czas w sekundach
        float arg = k * ((t - t0) / total_time);    // normalizacja argumentu
        float s   = 0.5f * (1.0f + tanhf(arg));     // wartość w zakresie [0..1]
        key->ramp_data.velocity_tab[i] = s;
    }

    // --- KOREKTA BRZEGÓW ---
    key->ramp_data.velocity_tab[0]     = 0.0f;
    key->ramp_data.velocity_tab[N - 1] = 1.0f;

    // reset indeksu rampy
    key->ramp_data.CurrentIndex = 0;*/
}

void SpeedRamp(WHEEL_Type *key)
{
	if(key->ramp_data.LastSetpointVelocity!=key->setpoint_velocity)
	{
		key->ramp_data.LastSetpointVelocity=key->setpoint_velocity;     //przypisuje starej predkosci nową prędkość
		key->ramp_data.StartVelocity=key->encoder_data.LinearSpeed;     //predkosc startująca to aktualna predkość
		key->ramp_data.CurrentIndex=0;
	}
	if(key->ramp_data.LastSetpointVelocity==key->setpoint_velocity)
	{
		if(key->ramp_data.CurrentIndex<VELOCITY_TAB_SIZE)
		{
			key->ramp_data.Delta=key->ramp_data.LastSetpointVelocity-key->ramp_data.StartVelocity; //Obliczam o ile ma się zmienic predkośc
			if(key->ramp_data.Delta>=0)      //gdy przyśpieszamy
			{
				float x=key->ramp_data.StartVelocity+(key->ramp_data.Delta*key->ramp_data.velocity_tab[key->ramp_data.CurrentIndex]); //Obliczam predkosć jaką mam ustawic
				key->set_velocity=x;//=constrain(x,-1*(key->ramp_data.LastSetpointVelocity),key->ramp_data.LastSetpointVelocity); // to taki bezpiecznik raczej nie potrzebny
			}
			else if(key->ramp_data.Delta<=0) //gdy zwalniamy
			{
				float x=key->ramp_data.StartVelocity+(key->ramp_data.Delta*key->ramp_data.velocity_tab[key->ramp_data.CurrentIndex]); //Obliczam predkosć jaką mam ustawic
				key->set_velocity=x;//constrain(x,key->ramp_data.LastSetpointVelocity,-1*(key->ramp_data.LastSetpointVelocity)); // to taki bezpiecznik raczej nie potrzebny
			}
			key->ramp_data.CurrentIndex++;
		}
		else //
		{
		    key->set_velocity = key->ramp_data.LastSetpointVelocity;
		}
	}
}
void PID(WHEEL_Type* key)
{   //to dla enkoderów maxon
	/*float P=32.629;     //definiuje współczynnik P,I,D
	float I=995.281;
	float D=0.0;
	key->pid_data.Deviation=key->setpoint_velocity-key->encoder_data.LinearSpeed;    //obliczam uchyb regulacji
	if(key->pid_data.PidSignal>-1000&&key->pid_data.PidSignal<1000)     //to zabezpieczenie przed windup
	{
		key->pid_data.Integral+=key->pid_data.Deviation*MOTOR_SETTING_TIME/1000;//obliczam całkę z uchybu 0.2 ponieważ bede tą funkcję wykonywał co 0.2s
	}
	else
	{
		if(key->encoder_data.LinearSpeed==0)
		{
			key->pid_data.Integral=0;
		}
		if(key->encoder_data.LinearSpeed!=0)
		{
			key->pid_data.Integral=key->pid_data.Integral;
		}
	}

	key->pid_data.PidSignal = key->pid_data.Deviation*P + key->pid_data.Integral*I;//+ key->pid_data.Derivative*D;//to standardowy regulator PID
	key->motor_siganl=key->pid_data.PidSignal;
	//key->motor_siganl=key->setpoint_velocity*250; //to jazda bez PID*/
	if(key->MOTOR_STATE==DRIVE_WITH_PID)
	{//to dlapredkosci liczonej ze sterownika bldc
		float P=32.629;     //definiuje współczynnik P,I,D  //32.629    //12.629
		float I=995.281; //995                                          //695
		float D=0.0;
		key->pid_data.Deviation=key->setpoint_velocity-key->bldc_pg_data.LinearSpeed;    //obliczam uchyb regulacji
		if(key->pid_data.PidSignal>-1000&&key->pid_data.PidSignal<1000)     //to zabezpieczenie przed windup
		{
			key->pid_data.Integral+=key->pid_data.Deviation*MOTOR_SETTING_TIME/1000;//obliczam całkę z uchybu 0.2 ponieważ bede tą funkcję wykonywał co 0.2s
			if(key->pid_data.Integral>1) key->pid_data.Integral=1;  //saturacja wyjscia
			if(key->pid_data.Integral<-1) key->pid_data.Integral=-1;
			if (fabs(key->setpoint_velocity) < 0.01f && fabs(key->bldc_pg_data.LinearSpeed) < 0.1f)
			{
				key->pid_data.Integral=0;
				//key->pid_data.Deviation=0;
			}
			/*if(fabs(key->setpoint_velocity) < 0.01f && fabs(key->pid_data.PidSignal) < 20.0f)
			{
			    key->pid_data.Integral = 0;
			    key->pid_data.Deviation = 0;
			}*/
		}
		else
		{
			if(key->bldc_pg_data.LinearSpeed==0)
			{
				//if(key->motor_siganl)
				key->pid_data.Integral=0;
			}
			if(key->bldc_pg_data.LinearSpeed!=0)
			{
				key->pid_data.Integral=key->pid_data.Integral;
				if(key->pid_data.Integral>1) key->pid_data.Integral=1;  //saturacja wyjscia
				if(key->pid_data.Integral<-1) key->pid_data.Integral=-1;
			}
		}
		key->pid_data.Derivative =(key->pid_data.Deviation - key->pid_data.PreviousDeviation) / (MOTOR_SETTING_TIME/1000.0f);  //to obliczanie pochodnej
		key->pid_data.PreviousDeviation = key->pid_data.Deviation;

		key->pid_data.PidSignal = key->pid_data.Deviation*P + key->pid_data.Integral*I + key->pid_data.Derivative *D;//+ key->pid_data.Derivative*D;//to standardowy regulator PID

		if(key->pid_data.PidSignal>1000) key->pid_data.PidSignal=1000;  //saturacja wyjscia
		if(key->pid_data.PidSignal<-1000) key->pid_data.PidSignal=-1000;

		if(fabs(key->setpoint_velocity) < 0.01f) key->pid_data.PidSignal = 0;

		key->motor_siganl=key->pid_data.PidSignal;
	}
	else if(key->MOTOR_STATE==X_STOP) //to sterowanie gdy mamy stać w miejscu
	{
		key->motor_siganl=0;
	}
	else //to sterowanie za pomoca tylko PWM bez PID
	{
		key->motor_siganl=key->setpoint_velocity*330;
	}
}

float NormalizeAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle <= -180.0f)
        angle += 360.0f;
    return angle;
}

int StepperMotor(WHEEL_Type* key, float current_angle)
{
	int speed=0;
	static int i = 0; // to zmienna odpowiadjaca za przechowywanie wartosci czy jest ruch 0-brak ruchu 1-ruch
	float error; //to zmienna przechowujaca błąd
	error = key->setpoint_angle - current_angle; // czyli róznica pomiedzy zadanym kątem a aktualnym
	if((error>4)&&((i==0)||(i==1)))   //ruch w prawo z pełna predkoscia
	{
		SetSteps(1);
		HAL_GPIO_WritePin(KRK_DIR_GPIO_Port,KRK_DIR_Pin, RESET);
		HAL_GPIO_WritePin(KRK_RESET_GPIO_Port,KRK_RESET_Pin, SET);
		HAL_GPIO_WritePin(KRK_EN_GPIO_Port, KRK_EN_Pin, RESET);
		i=1;
		speed=50;
	}
	else if((error<-4)&&((i==0)||(i==1)))   //ruch w lewo z pełna prdkoscia
	{
		SetSteps(1);
		HAL_GPIO_WritePin(KRK_DIR_GPIO_Port,KRK_DIR_Pin, SET);
		HAL_GPIO_WritePin(KRK_RESET_GPIO_Port,KRK_RESET_Pin, SET);
		HAL_GPIO_WritePin(KRK_EN_GPIO_Port, KRK_EN_Pin, RESET);
		i=1;
		speed=50;
	}
	else if((error>1)&&(error<4)&&(i==1)) //ruch w prawo z 1/4 predkoscia
	{
		SetSteps(4);
		HAL_GPIO_WritePin(KRK_DIR_GPIO_Port,KRK_DIR_Pin, RESET);
		HAL_GPIO_WritePin(KRK_RESET_GPIO_Port,KRK_RESET_Pin, SET);
		HAL_GPIO_WritePin(KRK_EN_GPIO_Port, KRK_EN_Pin, RESET);
		i=1;
		speed=50;
	}
	else if((error>-4)&&(error<-1)&&(i==1)) //ruch w lewo z 1/4 predkoscia
	{
		SetSteps(4);
		HAL_GPIO_WritePin(KRK_DIR_GPIO_Port,KRK_DIR_Pin, SET);
		HAL_GPIO_WritePin(KRK_RESET_GPIO_Port,KRK_RESET_Pin, SET);
		HAL_GPIO_WritePin(KRK_EN_GPIO_Port, KRK_EN_Pin, RESET);
		i=1;
		speed=50;
	}
	else
	{
		if((key->motor_siganl==0)&&(key->bldc_pg_data.LinearSpeed >= -0.2f) && (key->bldc_pg_data.LinearSpeed <= 0.2f))//tutaj nie trzymamy kąta
		{
			HAL_GPIO_WritePin(KRK_EN_GPIO_Port, KRK_EN_Pin, SET);
		}
		else//tutaj trzymanie kata
		{
			HAL_GPIO_WritePin(KRK_EN_GPIO_Port, KRK_EN_Pin, RESET);
		}
		HAL_GPIO_WritePin(KRK_RESET_GPIO_Port,KRK_RESET_Pin, RESET);
		i=0;
		speed=0;
	}
	/*if((x)>(Wheel.setpoint_angle+10.0))
	       {
	       	HAL_GPIO_WritePin(KRK_DIR_GPIO_Port,KRK_DIR_Pin, RESET);
	       	HAL_GPIO_WritePin(KRK_RESET_GPIO_Port,KRK_RESET_Pin, SET);
	       	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
	       }
	       else if((x)<Wheel.setpoint_angle)
	       {
	       	HAL_GPIO_WritePin(KRK_DIR_GPIO_Port,KRK_DIR_Pin, SET);
	       	HAL_GPIO_WritePin(KRK_RESET_GPIO_Port,KRK_RESET_Pin, SET);
	       	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
	       }
	       else
	       {
	       	HAL_GPIO_WritePin(KRK_M0_GPIO_Port,KRK_M0_Pin, RESET);
	       	HAL_GPIO_WritePin(KRK_M1_GPIO_Port,KRK_M1_Pin, SET);
	       	HAL_GPIO_WritePin(KRK_M2_GPIO_Port,KRK_M2_Pin, RESET);
	       	HAL_GPIO_WritePin(KRK_RESET_GPIO_Port,KRK_RESET_Pin, RESET);
	       	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	       }*/
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	return speed;
}

void SetSteps(int i)//to funkcja ktora ustawia mikrokroki
{
	if(i==1)
	{
		HAL_GPIO_WritePin(KRK_M0_GPIO_Port,KRK_M0_Pin, RESET);
		HAL_GPIO_WritePin(KRK_M1_GPIO_Port,KRK_M1_Pin, RESET);
		HAL_GPIO_WritePin(KRK_M2_GPIO_Port,KRK_M2_Pin, RESET);
	}
	if(i==2)
	{
		HAL_GPIO_WritePin(KRK_M0_GPIO_Port,KRK_M0_Pin, SET);
		HAL_GPIO_WritePin(KRK_M1_GPIO_Port,KRK_M1_Pin, RESET);
		HAL_GPIO_WritePin(KRK_M2_GPIO_Port,KRK_M2_Pin, RESET);
	}
	if(i==4)
	{
		HAL_GPIO_WritePin(KRK_M0_GPIO_Port,KRK_M0_Pin, RESET);
		HAL_GPIO_WritePin(KRK_M1_GPIO_Port,KRK_M1_Pin, SET);
		HAL_GPIO_WritePin(KRK_M2_GPIO_Port,KRK_M2_Pin, RESET);
	}
	if(i==8)
	{
		HAL_GPIO_WritePin(KRK_M0_GPIO_Port,KRK_M0_Pin, RESET);
		HAL_GPIO_WritePin(KRK_M1_GPIO_Port,KRK_M1_Pin, RESET);
		HAL_GPIO_WritePin(KRK_M2_GPIO_Port,KRK_M2_Pin, SET);
	}

}
void ReadVelocityFromPGPin(WHEEL_Type* key, uint32_t PG_counter)
{
    //static uint32_t noPulseTime = 0;
	int kierunek = (key->motor_siganl >= 0) ? 1 : -1; //okreslam kierunek obrotow

    int32_t delta = (int32_t)PG_counter - (int32_t)key->bldc_pg_data.LastValue; //obliczam delte czyli zmiane od ostaniego razu

    // Obsługa przepełnienia (overflow) 16-bitowego licznika timera
    if (delta < -32768)  // zakładamy, że timer ma zakres 0 - 65535
    {
    	delta += 65536;
    }
    else if (delta > 32768)
    {
    	delta -= 65536;
    }
    key->bldc_pg_data.LastValue = PG_counter;
    float sample_time_sec = PG_SETTING_TIME / 1000.0f;
    key->bldc_pg_data.RotationalSpeed = kierunek * ((float)delta / (PG_RESOLUTION * GEAR_RATIO * sample_time_sec)); //obliczam predkosc obrotowa [obr/sek]
    key->bldc_pg_data.LinearSpeed =key->bldc_pg_data.RotationalSpeed * WHEEL_TRAVEL_DISTANCE;  // prędkość liniowa z obrotowej
    key->bldc_pg_data.TotalTurns += delta;// zliczanie wszystkich impulsów
    // Jeden obrót = 2*PI rad
    key->bldc_pg_data.Turns += kierunek * ((float)delta / (PG_RESOLUTION * GEAR_RATIO)) * 2.0f * PI;
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Przetwarza bity enkodera na kąt za pomocą 128 instrukcji IF
  * @param  bity: Wskaźnik na 8-elementową tablicę stanów pinów (0 lub 1)
  * @retval Kąt w stopniach (0.0f do 357.1875f) lub -1.0f w przypadku błędu
  */
float Odczytaj_Kat_Enkodera(int raw_bits)
{
	if (raw_bits == 127) return 0.0f;
	    if (raw_bits == 63)  return 2.8125f;
	    if (raw_bits == 62)  return 5.625f;
	    if (raw_bits == 58)  return 8.4375f;
	    if (raw_bits == 56)  return 11.25f;
	    if (raw_bits == 184) return 14.0625f;
	    if (raw_bits == 152) return 16.875f;
	    if (raw_bits == 24)  return 19.6875f;
	    if (raw_bits == 8)   return 22.5f;
	    if (raw_bits == 72)  return 25.3125f;
	    if (raw_bits == 73)  return 28.125f;
	    if (raw_bits == 77)  return 30.9375f;
	    if (raw_bits == 79)  return 33.75f;
	    if (raw_bits == 15)  return 36.5625f;
	    if (raw_bits == 47)  return 39.375f;
	    if (raw_bits == 175) return 42.1875f;
	    if (raw_bits == 191) return 45.0f;
	    if (raw_bits == 159) return 47.8125f;
	    if (raw_bits == 31)  return 50.625f;
	    if (raw_bits == 29)  return 53.4375f;
	    if (raw_bits == 28)  return 56.25f;
	    if (raw_bits == 92)  return 59.0625f;
	    if (raw_bits == 76)  return 61.875f;
	    if (raw_bits == 12)  return 64.6875f;
	    if (raw_bits == 4)   return 67.5f;
	    if (raw_bits == 36)  return 70.3125f;
	    if (raw_bits == 164) return 73.125f;
	    if (raw_bits == 166) return 75.9375f;
	    if (raw_bits == 167) return 78.75f;
	    if (raw_bits == 135) return 81.5625f;
	    if (raw_bits == 151) return 84.375f;
	    if (raw_bits == 215) return 87.1875f;
	    if (raw_bits == 223) return 90.0f;
	    if (raw_bits == 207) return 92.8125f;
	    if (raw_bits == 143) return 95.625f;
	    if (raw_bits == 142) return 98.4375f;
	    if (raw_bits == 14)  return 101.25f;
	    if (raw_bits == 46)  return 104.0625f;
	    if (raw_bits == 38)  return 106.875f;
	    if (raw_bits == 6)   return 109.6875f;
	    if (raw_bits == 2)   return 112.5f;
	    if (raw_bits == 18)  return 115.3125f;
	    if (raw_bits == 82)  return 118.125f;
	    if (raw_bits == 83)  return 120.9375f;
	    if (raw_bits == 211) return 123.75f;
	    if (raw_bits == 195) return 126.5625f;
	    if (raw_bits == 203) return 129.375f;
	    if (raw_bits == 235) return 132.1875f;
	    if (raw_bits == 239) return 135.0f;
	    if (raw_bits == 231) return 137.8125f;
	    if (raw_bits == 199) return 140.625f;
	    if (raw_bits == 71)  return 143.4375f;
	    if (raw_bits == 7)   return 146.25f;
	    if (raw_bits == 23)  return 149.0625f;
	    if (raw_bits == 19)  return 151.875f;
	    if (raw_bits == 3)   return 154.6875f;
	    if (raw_bits == 1)   return 157.5f;
	    if (raw_bits == 9)   return 160.3125f;
	    if (raw_bits == 41)  return 163.125f;
	    if (raw_bits == 169) return 165.9375f;
	    if (raw_bits == 233) return 168.75f;
	    if (raw_bits == 225) return 171.5625f;
	    if (raw_bits == 229) return 174.375f;
	    if (raw_bits == 245) return 177.1875f;
	    if (raw_bits == 247) return 180.0f;
	    if (raw_bits == 243) return 182.8125f;
	    if (raw_bits == 227) return 185.625f;
	    if (raw_bits == 163) return 188.4375f;
	    if (raw_bits == 131) return 191.25f;
	    if (raw_bits == 139) return 194.0625f;
	    if (raw_bits == 137) return 196.875f;
	    if (raw_bits == 129) return 199.6875f;
	    if (raw_bits == 128) return 202.5f;
	    if (raw_bits == 132) return 205.3125f;
	    if (raw_bits == 148) return 208.125f;
	    if (raw_bits == 212) return 210.9375f;
	    if (raw_bits == 244) return 213.75f;
	    if (raw_bits == 240) return 216.5625f;
	    if (raw_bits == 242) return 219.375f;
	    if (raw_bits == 250) return 222.1875f;
	    if (raw_bits == 251) return 225.0f;
	    if (raw_bits == 249) return 227.8125f;
	    if (raw_bits == 241) return 230.625f;
	    if (raw_bits == 209) return 233.4375f;
	    if (raw_bits == 193) return 236.25f;
	    if (raw_bits == 197) return 239.0625f;
	    if (raw_bits == 196) return 241.875f;
	    if (raw_bits == 192) return 244.6875f;
	    if (raw_bits == 64)  return 247.5f;
	    if (raw_bits == 66)  return 250.3125f;
	    if (raw_bits == 74)  return 253.125f;
	    if (raw_bits == 106) return 255.9375f;
	    if (raw_bits == 122) return 258.75f;
	    if (raw_bits == 120) return 261.5625f;
	    if (raw_bits == 121) return 264.375f;
	    if (raw_bits == 125) return 267.1875f;
	    if (raw_bits == 253) return 270.0f;
	    if (raw_bits == 252) return 272.8125f;
	    if (raw_bits == 248) return 275.625f;
	    if (raw_bits == 232) return 278.4375f;
	    if (raw_bits == 224) return 281.25f;
	    if (raw_bits == 226) return 284.0625f;
	    if (raw_bits == 98)  return 285.0432f;
	    if (raw_bits == 96)  return 286.875f;
	    if (raw_bits == 32)  return 292.5f;
	    if (raw_bits == 33)  return 295.3125f;
	    if (raw_bits == 37)  return 298.125f;
	    if (raw_bits == 53)  return 300.9375f;
	    if (raw_bits == 61)  return 303.75f;
	    if (raw_bits == 60)  return 306.5625f;
	    if (raw_bits == 188) return 309.375f;
	    if (raw_bits == 190) return 312.1875f;
	    if (raw_bits == 254) return 315.0f;
	    if (raw_bits == 126) return 317.8125f;
	    if (raw_bits == 124) return 320.625f;
	    if (raw_bits == 116) return 323.4375f;
	    if (raw_bits == 112) return 326.25f;
	    if (raw_bits == 113) return 329.0625f;
	    if (raw_bits == 49)  return 331.875f;
	    if (raw_bits == 48)  return 334.6875f;
	    if (raw_bits == 16)  return 337.5f;
	    if (raw_bits == 144) return 340.3125f;
	    if (raw_bits == 146) return 343.125f;
	    if (raw_bits == 154) return 345.9375f;
	    if (raw_bits == 158) return 348.75f;
	    if (raw_bits == 30)  return 351.5625f;
	    if (raw_bits == 94)  return 354.375f;
	    if (raw_bits == 95)  return 357.1875f;

	    return -1.0f; // Zwracane w przypadku nieprawidłowego stanu styków (błąd odczytu)
}
