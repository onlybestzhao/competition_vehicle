#include "stm32f4xx.h"                  // Device header

extern float global_angle;
float current_angle;
float error;
	float target_tangle;
	
void errortest(void)
{
	
	current_angle=global_angle;
	error=target_tangle-current_angle;
	
}

void adjustpwm(void)
{
	if(error>0)
	{
		
	}
}
