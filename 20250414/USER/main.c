#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "stm32f4xx.h"                  
#include "ENCODER.h" 
#include "HWT101.h"
#include "SERVO.h"
#include "sport.h"

#include "vision.h"
#include "stdio.h"
volatile  int16_t x ; 
volatile int16_t y ;
volatile  uint8_t data_ready=0;
volatile uint8_t buffer[6];
volatile uint8_t sequence[9];
volatile float  yawrate;
volatile uint8_t new_data_received = 0;
volatile uint8_t rx_buffer[11];
 volatile float velocitz;
extern volatile uint32_t system_millis;
 volatile float outvelocity;
 volatile float output;
 volatile float global_angle;
 volatile float  error;
uint8_t received_data_packet[11] = {0};

uint8_t distance[4];
 //uint8_t sequence[4];

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
  PWM_INIT();
  Servo_PWM_Init ();
	 SysTick_Init();
	
		 SysTick_Handler();
	TIM12_Init();
	
	UART5_Init();
USART2_Init();
	USART3_Init();

	maxhz(); 
	servo4(5000);
	//servo4(3950);
	//Delay_s(1);
	//servo4(4100);
//anglezreset();
allgo();	
//turntable_take3();
	//Delay_s(3);
	//Delay_ms(3000);
	//Serial_SendByte(9);


	while(1) 
{
		
}
}
