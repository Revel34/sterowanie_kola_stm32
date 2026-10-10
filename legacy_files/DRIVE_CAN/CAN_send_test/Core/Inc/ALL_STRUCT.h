/*
 * ALL_STRUCT.h
 *                                  w tym pliku bede umieszczal struktury dotyczące moich kół
 Created on: Dec 20, 2024
 *      Author: Andrzej Ozga
 */

#ifndef INC_ALL_STRUCT_H_
#define INC_ALL_STRUCT_H_
#include <math.h> //to bibliteka do funkcji sqrt
#include <stdint.h>
#include <stdint.h>

#define FRAME_TIME 1
#define MOTOR_SETTING_TIME 100        //to czas co ile wywołuje funkcje od ustawiania synału na silniki - potem zmiana na ms
#define PG_SETTING_TIME 100          //to czas co ile licze predkosc ze sterownika PG
#define STEPPER_SETTING_TIME 50      //to czas co ile steruje krokowcami - potem zamiana na ms
#define JOINT_STATE_TIME 1000        //to czas co ile publikuje joint_state

#define PWM_MOTOR_MIN -1000          //to wartość min dla pwm
#define PWM_MOTOR_MAX 1000           //to wartość max dla pwm

#define ROVER_WIDTH  0.713            //szerokosc osi lazika [m]
#define ROVER_LENGTH 0.789            //długosc osi lazika [m]

#define PI 3.14159265358979323846   //definuje PI

#define GEAR_RATIO        19
#define PG_RESOLUTION     42
#define ENCODER_RESOLUTION 4096
#define WHEEL_DIAMETER    0.26
#define WHEEL_TRAVEL_DISTANCE (PI * WHEEL_DIAMETER)

#define VELOCITY_TAB_SIZE 5000/MOTOR_SETTING_TIME   //definuje jaki rozmiar ma mieć tabica do inkrementacji predkosci  5000 bo 5s
//w tym pliku bede umieszczal struktury i funkcje


typedef enum //tworzę nowy typ zmiennej enumerycznej-MOTOR_STATE_Type - czyli stan w jakim znajduje sie dane koło
{
	IDLE_1=0,              //to stan jałowy
	DRIVE_WITH_PID,        //to stan jazdy w zamknietej petli czyli PID
	DRIVE_WITHOUT_PID,     //to stan jazdy bez pid czyli na PWM tylko
	X_STOP                 //w tym stanie koła ustawiają sie na znak X czyli mozemy stać na górce
 }MOTOR_STATE_Type;

 typedef enum //tworzę nowy typ zmiennej-MOVE_STATE_Type czyli sposob w jaki auto sie porusza
 {
 	IDLE_2= 0,             //to stan nie ustalony
 	FORWARD,               //to stan jazdy do przodu/tyłu
	CRAB_WALK,             //to stan jazdy krab-czyli prawo/lewo
	DIAGONAL,              //czyli ruch po przekątnej
	AROUND,                //to stan jazdy wokół wlasnej osi
	ROUND                  //to stan jazdy po okregu
 }MOVE_STATE_Type;

 typedef struct //toworze strukture-ENCODER_Type sa tutaj wszytkie parametry dla enkodera
 {
	 uint32_t LastValue;         //tworzę zmienną która pamięta ostanią wartość
	 uint32_t TotalTurns;        //tworze zmienną która przechowuje wszytkie obroty które zrobił silnik
	 uint32_t LastValue_1;         //tworzę zmienną która pamięta ostanią wartość
	 uint32_t TotalTurns_2;        //tworze zmienną która przechowuje wszytkie obroty które zrobił silnik
	 float Turns;                //tworze zmienna która przechowuje pozycje koła w [rad]
	 float RotationalSpeed;      //tworze zmienną ktora przechowuje aktualną prędkosc obrotową - znak to kierunek obrotów [obr/s]
	 float LinearSpeed;			 //tworze zmienną ktora przechowuje aktualną prędkosc linową - znak to kierunek obrotów	  [m/s]
	 // tutaj bedą jeszce inne parametry które można obliczyć z enkodera
 }ENCODER_Type;


  typedef struct //tworzę strukture PID_Type są tutaj wszystkie dane potrzebne do reg. PID
    {
    float 				PidSignal;     //to syganł obliczony przez nas regulator PID
    float               Deviation;     //to uchyb regulacji - wartość zadana - wartość bieżąca
    float               Integral;      //to całka z uchybu
    float               Derivative;    //to pochodna z uchybu
    float 				PreviousDeviation; //to poprzedni uchyb regulacji - potrzebne do Pochodnej
    }PID_Type;

  typedef struct //tworze strukture RAMP_Type są tutaj wszystkie dane potrzebne do obliczania rampy
  {
	  int          StartVelocity;       //to prędkość początkowa
	  int		   LastSetpointVelocity;//to ostatnia predkosc do której chcieliśmy dojść
	  float        Delta;               //to różnica prekosci
	  float        velocity_tab[VELOCITY_TAB_SIZE]; //to tablica do inrementowania prekosci
	  int    	   CurrentIndex;          //to numer kroku od inkrementacji
  }RAMP_Type;
typedef struct //toworze strukture-WHEEL_Type sa tutaj wszytkie parametry kola
{
	MOTOR_STATE_Type      MOTOR_STATE;      //to stan silnika
	MOVE_STATE_Type       MOVE_STATE;       //to stan poruszania sie lazik
	ENCODER_Type          encoder_data;     //to struktura z parametrami odczytanymi z enkodera
	ENCODER_Type          bldc_pg_data;     //to struktura z parametrami odczytanymi ze sterownika bldc
	int                   motor_siganl;     //to sygnal jaki jest ustawiony na silniku
	PID_Type   			  pid_data;         //to struktura z wszystkimi parametrami regulatora PID
	RAMP_Type  			  ramp_data;        //to struktura z wszystkimi parametrami do rampy narastania predkosci
	float 				  setpoint_velocity;//to żądana prędkosć[m/s]
	float 				  set_velocity;     //to prędkość do ustawienia
	float 				  setpoint_angle;   //to żądany kąt     [stopnie] kąt 0 jest do przodu
	float 				  current_angle;    //to aktualny kąt   [stopnie]
	float 				  zero_angle;       //to kąt początkowy [stopnie]
	float 				  pg_velocity;      //to predkosc obliczona z pinu PG
	int 				  closed_loop;      //to zmienna od sterowania przy pomocy PWM(0) lub PID(1)

}WHEEL_Type;

void ReadNewCmdVel(float x_lin,float y_lin,float z_lin,float z_ang,WHEEL_Type *key, int tab[]); //to funkcja od odczytania wektora predkosci na nasze wartosci

void ReadExtraParametres(float z_lin, WHEEL_Type *key); // to funkcja do czytania trybów sterowania czyl z_lin

void AngleLimit(WHEEL_Type *key, float AngelWithoutLimit,float velo);//to funkcja która obcina kąt do +90 i -90 i zwraca prekość z minusem lub plusem

float VelocityLimit(float vel, float limit); //to funkcja do ograniczenia predkosci do podanych limitów

void GenerateVelTab(WHEEL_Type *key, int tab[]); //uzupełniam tablice do płynnej zmiany prędkosci - wartosci są w procentach zmiany jaka ma nastapic

void SpeedRamp(WHEEL_Type *key);

float NormalizeAngle(float angle);

int StepperMotor(WHEEL_Type* key, float current_angle);

void TIM2_SetPrescaler(uint16_t new_psc);

float Odczytaj_Kat_Enkodera(int raw_bit);

void ReadVelocityFromPGPin(WHEEL_Type* key,uint32_t PG_counter);
#endif /* INC_ALL_STRUCT_H_ */
