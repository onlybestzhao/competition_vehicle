#ifndef _SERVO_H
#define _SERVO_H
#include "sys.h"



void Servo_PWM_Init(void);
void TIM12_Init(void);
void servo2(int pulse);
void servo3(int pulse);
void servo1(int pulse);
void servo4(int pulse);
void servo5(int pulse);

void turntable_take1(void);
void turntable_take2(void);
void turntable_take3(void);

void turntable_take4(void);
void turntable_take5(void);
void turntable_take6(void);
void ground_put_1(void);

void ground_put_2(void);

void ground_put_3(void);
void ground_take_1(void);
	void ground_take_2(void);
		void ground_take_3(void);
#endif
