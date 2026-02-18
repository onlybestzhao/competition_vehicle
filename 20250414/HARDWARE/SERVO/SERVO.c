#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "delay.h"
#include "sport.h"
void TIM12_Init(void) {
   GPIO_InitTypeDef GPIO_InitStructure;          
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
TIM_OCInitTypeDef TIM_OCInitStructure;        


RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);  
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  


GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;            
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          
GPIO_Init(GPIOB, &GPIO_InitStructure);                
GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12); 


TIM_TimeBaseStructure.TIM_Period = 39999;             
TIM_TimeBaseStructure.TIM_Prescaler = 41;            
TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);      


TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;     
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
TIM_OCInitStructure.TIM_Pulse = 0;                    
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; 

TIM_OC1Init(TIM12, &TIM_OCInitStructure);            


TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);    

TIM_Cmd(TIM12, ENABLE);                              
}

void Servo_PWM_Init(void)	
{ 
	GPIO_InitTypeDef GPIO_InitStructure;           //IO
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //定时器	
	TIM_OCInitTypeDef  TIM_OCInitStructure;        //PWM输出 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM8时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTC时钟	
		
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; 	
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);
	
	

	TIM_TimeBaseStructure.TIM_Period = 39999; 
	//Pre-divider //预分频器 
	TIM_TimeBaseStructure.TIM_Prescaler =83; 	
	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//TIM up count mode //TIM向上计数模式	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
  //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	
	//比较输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	
	//设置待装入捕获比较寄存器的脉冲值
	 TIM_OCInitStructure.TIM_Pulse = 0; 
	 TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	 
	 TIM_OCInitStructure.TIM_Pulse = 0;
   TIM_OC2Init(TIM8, &TIM_OCInitStructure); 	 
	 TIM_OCInitStructure.TIM_Pulse = 0; 
	 TIM_OC3Init(TIM8, &TIM_OCInitStructure); 
	 TIM_OCInitStructure.TIM_Pulse = 0; 
	 TIM_OC4Init(TIM8, &TIM_OCInitStructure); 
 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
//Channel preload enable
	//通道预装载使能	 
	
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	//Advanced timer output must be enabled //高级定时器输出必须使能这句	
	TIM_CtrlPWMOutputs(TIM8,ENABLE); 	
	//Enable timer //使能定时器
	TIM_Cmd(TIM8, ENABLE); 		 
	
}
void servo1(int pulse) {
    
    if(pulse < 1000) pulse = 1000;
    if(pulse > 5000) pulse = 5000;
    TIM8->CCR1 = pulse;
}
void servo2(int pulse)
{
	if(pulse < 1000) pulse = 1000;
  if(pulse > 5000) pulse = 5000;
	TIM8->CCR2=pulse;
}
void servo3(int pulse)
{
	
	TIM8->CCR3=pulse;
}
void servo4(int pulse)
{
	
	TIM8->CCR4=pulse;
}
void servo5(int pulse)
{
	TIM12->CCR1=pulse;
}



void turntable_take1(void)
{
	servo5(1000);
	servo1(3100);
	servo2(1100);
	Delay_ms(300);
	servo3(2400);
	Delay_s(1);
	servo3(3500);
	Delay_s(1);
	servo1(4000);
	servo2(2800);
	
	Delay_ms(1100);
	servo3(2200);
	Delay_s(1);
}
void turntable_take2(void)
{
	servo5(2850);
	servo1(3100);
	servo2(1100);
	Delay_ms(500);
	servo3(2200);
	Delay_s(1);
	servo3(3200);
	Delay_s(1);
	servo1(4000);
	servo2(2800);
	
	Delay_ms(1100);
	servo3(2200);
	Delay_s(1);
	 //1
	//servo5(4100);//2
	//servo5(1000);
}
void turntable_take3(void)
{
	servo5(2777+1777+50);
	servo1(3100);
	servo2(1100);
	Delay_ms(500);
	servo3(2200);
	Delay_s(1);
	servo3(3500);
	Delay_s(1);
	servo1(4000);
	servo2(2800);
	
	Delay_ms(1100);
	servo3(2500);
	Delay_s(1); 
	servo1(3100);
	servo2(1100);
	
	servo5(1000); 
	Delay_ms(2000);
	servo1(4000);
	servo2(2900);
	Delay_ms(2500);
	//servo5(2400); //1
	//servo5(4100);//2
	
}
void ground_put_1(void)
{
	servo3(3500);
	Delay_ms(500);
	servo1(2100); 
  servo2(2100);

	servo5(2777);//2
	Delay_ms(1500);
  servo3(2800);
	Delay_ms(500);

	servo3(2500);
	servo1(4000);
	servo2(2900);
	Delay_ms(1500);
	servo3(3500);
	
}
void ground_put_2(void)
{ 
	servo3(3500);
	Delay_ms(500);
	servo1(2100); 
  servo2(2100);
  
	servo5(2777+1777);//3
	Delay_ms(1500);
	servo3(2800);
	Delay_ms(500);
	
	servo3(2500); 
	servo1(4000);
	servo2(2900);
	
	Delay_ms(1500);
	servo3(3500);
	
}
void ground_put_3(void)
{
	servo3(3500);
	Delay_ms(1000);
	servo1(2100); 
  servo2(2100);
	Delay_ms(2000);
	servo3(2500);
	
	Delay_ms(2000);
	servo1(4000); 
	servo2(2900);
	Delay_ms(1500);
  servo5(1000);
}

void ground_take_1(void)
{//servo3(3500);
  servo1(2100); 
  servo2(2000);
	Delay_ms(700);
	servo3(2500);
	Delay_s(1);
	servo3(3500);
	Delay_ms(1500);
	servo1(4000);
	servo2(2900);
	
	Delay_s(2);
	servo3(2000);
	//Delay_s(1);
	//servo5(2777);
Delay_s(1);
//servo3(3500);	//2
}
void ground_take_2(void)
{
  servo1(2100); 
  servo2(2000);
	Delay_s(1);
	servo3(2200);
	Delay_s(1);
	servo5(2777+100);
	servo3(3500);
	Delay_s(1);
	servo1(4000);
	servo2(2900);
	
	Delay_s(2);
	servo3(2200);
	Delay_s(1);
//	servo5(2777); //3
}
void ground_take_3(void)
{
  servo1(2100);         
  servo2(2000);
	Delay_s(1);
	servo3(2200);
	Delay_s(1);
	servo5(2777+1777+50);
	servo3(3500);
	Delay_s(1);
	servo1(4000);
	servo2(2900);
	
	Delay_s(2);
	servo3(2200);
	Delay_s(1);
	
	servo1(2100); 
  servo2(2000);
	Delay_ms(1500);
	servo5(1000); //1
	Delay_s(1);
	servo1(4000);
	servo2(2900);
	servo3(2500);
}
