#include "delay.h"
#include "stm32f4xx_it.h"
#include "system_stm32f4xx.h"
#include "SERVO.h"
#include "vision.h"
#include "hwt101.h"
#define PWMA1   TIM10->CCR1  
#define PWMA2   TIM11->CCR1 

#define PWMB1   TIM9->CCR1 
#define PWMB2   TIM9->CCR2

#define PWMC1   TIM1->CCR1  
#define PWMC2   TIM1->CCR2 

#define PWMD1   TIM1->CCR3 
#define PWMD2   TIM1->CCR4 
#define Servo_PWM  TIM8->CCR4
#define JUDGE_INIT    0
#define JUDGE_RUNNING 1
#define JUDGE_DONE    2
#define JUDGE_INITB    0
#define JUDGE_RUNNINGB 1
#define JUDGE_DONEB    2
#define PHASE_INIT       0
#define PHASE_LEFT_TURN  1
#define PHASE_ADVANCE    2
#define PHASE_TURNTABLE  3
#define PHASE_STOP2      4
#define PHASE_RECOIL     5
#define PHASE_TURNLEFT   6
#define PHASE_ADVANCE2   7
#define PHASE_TURNRIGHT  8
#define PHASE_PUT1       9
#define PHASE_STOP       10
extern uint8_t reset_z_axis[];
uint8_t judge_state = JUDGE_INIT; 
uint8_t judge_stateb=JUDGE_INITB;
uint32_t judge_delay_start = 0;    
uint8_t judge_step = 0;           
uint8_t judge_sequence_1 = 0;   
uint8_t judge_sequence_2 = 0;    
extern volatile  int16_t x ; 
extern volatile int16_t y ; 
extern volatile  uint8_t data_ready;
extern volatile float output;
extern volatile uint8_t sequence[9];
 volatile uint32_t system_millis=0;
extern volatile int a;
extern volatile float outvelocity;
extern volatile float output;
extern volatile float global_angle;
extern volatile float  error;
extern volatile float  yawrate;
extern  volatile float velocitz;
 volatile float target_angle=0;
 extern  uint8_t new_data_received;
 uint8_t readyagain=0;
 uint8_t changeready=0;
 uint8_t changeready2=0;
 uint8_t  readyjudgement2 = 0;
float current_angle;
float KP=38.5f;
float maxoutput=12000.0f;
float KD=7.2f;
uint8_t readyjudgement=0;
uint8_t judgeput1=0;
uint8_t judgeput2=0;
uint8_t judgeput3=0;
uint8_t readyprocessput=0;
uint8_t judgetake1=0;
uint8_t judgetake2=0;
uint8_t judgetake3=0;
uint8_t judgeput4=0;
uint8_t judgeput5=0;
uint8_t judgeput6=0;
uint8_t  readyagain2= 0;
void SysTick_Init(void) {
    
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1);  
    }
}

void SysTick_Handler(void) {
    system_millis++;
}

uint32_t get_millis(void) {
    return system_millis;
}

/**************************************************************************
Function: The motor PWM initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
函数功能：电机PWM引脚初始化
入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void PWM10_Init(uint16_t arr,uint16_t psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  	  //TIM8时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTB时钟	
	
 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM10); 
 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;   //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化PB口
	
	//Sets the value of the auto-reload register cycle for the next update event load activity
	//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Sets the pre-divider value used as the TIMX clock frequency divisor
	//设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	//Set the clock split :TDTS = Tck_tim
	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	//Up counting mode 
	//向上计数模式  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TIMEBASEINITSTRUCT
	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure); 

  //Select Timer mode :TIM Pulse Width Modulation mode 1
  //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//比较输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse=0;
  //Output polarity :TIM output polarity is higher	
  //输出极性:TIM输出比较极性高	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx	
	TIM_OC1Init(TIM10, &TIM_OCInitStructure); 
	//CH1 is pre-loaded and enabled
	//CH1预装载使能	 
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable); 
	

  // Enable the TIMX preloaded register on the ARR
  //使能TIMx在ARR上的预装载寄存器	
	TIM_ARRPreloadConfig(TIM10, ENABLE); 
	
	//Enable TIM8
	//使能TIM8
	TIM_Cmd(TIM10, ENABLE);  
} 


/**************************************************************************
Function: The motor PWM initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
函数功能：电机PWM引脚初始化
入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void PWM11_Init(uint16_t arr,uint16_t psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);  	  //TIM11时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTB时钟	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM11);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;   //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化PB口
	
	//Sets the value of the auto-reload register cycle for the next update event load activity
	//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Sets the pre-divider value used as the TIMX clock frequency divisor
	//设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	//Set the clock split :TDTS = Tck_tim
	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	//Up counting mode 
	//向上计数模式  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	
	TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure); 

 
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse=0;
  //Output polarity :TIM output polarity is higher	
  //输出极性:TIM输出比较极性高	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
		
	TIM_OC1Init(TIM11, &TIM_OCInitStructure); 
	//CH1 is pre-loaded and enabled
	//CH1预装载使能	 
	TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable); 
	

  // Enable the TIMX preloaded register on the ARR
  //使能TIMx在ARR上的预装载寄存器	
	TIM_ARRPreloadConfig(TIM11, ENABLE); 
	
	//Enable TIM8
	//使能TIM8
	TIM_Cmd(TIM11, ENABLE);  
} 


/**************************************************************************
Function: The motor PWM initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
函数功能：电机PWM引脚初始化
入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void PWM9_Init(uint16_t arr,uint16_t psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	  //TIM9时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTE时钟	
	
 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;   //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PE口
	
	 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  

	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure); 

 
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	
	TIM_OCInitStructure.TIM_Pulse=0;
  	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
		
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);	
	 
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable); 
	

  // Enable the TIMX preloaded register on the ARR
  //使能TIMx在ARR上的预装载寄存器	
	TIM_ARRPreloadConfig(TIM9, ENABLE); 
	
	//Enable TIM8
	//使能TIM8
	TIM_Cmd(TIM9, ENABLE);  
} 


/**************************************************************************
Function: The motor PWM initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
函数功能：电机PWM引脚初始化
入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void PWM1_Init(uint16_t arr,uint16_t psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	  //TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTE时钟	
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;   //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PE口
	
	//Sets the value of the auto-reload register cycle for the next update event load activity
	//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Sets the pre-divider value used as the TIMX clock frequency divisor
	//设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	//Set the clock split :TDTS = Tck_tim
	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	//Up counting mode 
	//向上计数模式  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TIMEBASEINITSTRUCT
	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

  //Select Timer mode :TIM Pulse Width Modulation mode 1
  //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//比较输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse=0;
 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);	
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);	
	//CH1 is pre-loaded and enabled
	//CH1预装载使能	 
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);  //高级定时器输出需要设置这句
  // Enable the TIMX preloaded register on the ARR
  //使能TIMx在ARR上的预装载寄存器	
	TIM_ARRPreloadConfig(TIM1, ENABLE); 
	
	//Enable TIM8
	//使能TIM8
	TIM_Cmd(TIM1, ENABLE);  
} 


float P_calculate(void)
{
	

    float current_angle = global_angle;

    yawrate=KD*velocitz;
    error = target_angle - current_angle;
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }

 
    outvelocity = yawrate;
    output=KP*error+outvelocity;

    if (output > maxoutput) {
        return maxoutput;
    } else if (output < -maxoutput) {
        return -maxoutput;
    } else {
			//printf("PID Output: %.2f\r\n", output);
        return output;
			
    }
		
}

void Set_ZPWMA(float motor_left)
{
   
	   PWMA1=motor_left;
	   PWMA2=0;
   
 }

 void Set_ZPWMB(float motor_left)
{
   
	 PWMB1=motor_left;
	   PWMB2=0;
   
 }
void Set_ZPWMC(float motor_left)
{
   
	   PWMC1=motor_left;
	   PWMC2=0;
   
 }
void Set_ZPWMD(float motor_left)
{
   
	   PWMD1=motor_left;
	   PWMD2=0;
   
 }

 
 void Set_SPWMA(int motor_left)
 {
	  if(motor_left==0)   
   {
	   PWMA2=0;
	   PWMA1=0;
   }
 }
 
  void Set_SPWMB(int motor_left)
{
   if(motor_left==0)   
   {
	   PWMB2=0;
	   PWMB1=0;
   }
 }
void Set_SPWMC(int motor_left)
{
   if(motor_left==0)   
   {
	   PWMC2=0;
	   PWMC1=0;
   }
 }
void Set_SPWMD(int motor_left)
{
   if(motor_left==0)   
   {
	   PWMD2=0;
	   PWMD1=0;
   }
 }
 

 
void Set_FPWMA(float motor_left)
{
   if(motor_left>0)   
   {
	   PWMA2=motor_left;
	   PWMA1=0;
   }
 }

 void Set_FPWMB(float motor_left)
{
   if(motor_left>0)   
   {
	   PWMB2=motor_left;
	   PWMB1=0;
   }
 }
void Set_FPWMC(float motor_left)
{
   if(motor_left>0)   
   {
	   PWMC2=motor_left;
	   PWMC1=0;
   }
 }
void Set_FPWMD(float motor_left)
{
   if(motor_left>0)   
   {
	   PWMD2=motor_left;
	   PWMD1=0;
   }
 }

void PWM_INIT(void)
{
	PWM1_Init(16799,0);
	PWM9_Init(16799,0);
	PWM10_Init(16799,0);
	PWM11_Init(16799,0);
}
void advanceb(void)
{
	    float left_speed = 12000 +output;
    float right_speed = 12000 -output;

if(output>0)
{	
    Set_FPWMA(right_speed);
    Set_FPWMB(left_speed);
    Set_ZPWMC(12000);
    Set_ZPWMD(12000);
}
else if(output<0)
{
	  Set_FPWMA(left_speed);
    Set_FPWMB(right_speed);
    Set_ZPWMC(12000);
    Set_ZPWMD(12000);
}

}
	
	void advance(void)
{
	    float left_speed = 10200 +output;
    float right_speed = 10200 -output;

if(output>0)
{	
    Set_FPWMA(right_speed);
    Set_FPWMB(left_speed);
    Set_ZPWMC(10200);
    Set_ZPWMD(10200);
}
else if(output<0)
{
	  Set_FPWMA(left_speed);
    Set_FPWMB(right_speed);
    Set_ZPWMC(10200);
    Set_ZPWMD(10200);
}

}
	

void recoil(void)
{
	 float left_speed = 10200 +output;
    float right_speed = 10200-output;

if(output>0)
{	
    Set_ZPWMA(right_speed);
    Set_ZPWMB(left_speed);
    Set_FPWMC(10200);
    Set_FPWMD(10200);
}
else if(output<0)
{
	  Set_ZPWMA(left_speed);
    Set_ZPWMB(right_speed);
    Set_FPWMC(10200);
    Set_FPWMD(10200);
}

}
void recoilb(void)
{
	 float left_speed = 12000 +output;
    float right_speed = 12000-output;

if(output>0)
{	
    Set_ZPWMA(right_speed);
    Set_ZPWMB(left_speed);
    Set_FPWMC(12000);
    Set_FPWMD(12000);
}
else if(output<0)
{
	  Set_ZPWMA(left_speed);
    Set_ZPWMB(right_speed);
    Set_FPWMC(12500);
    Set_FPWMD(12000);
}

}
void turnright(void)
{
	Set_FPWMA(9800);
	Set_ZPWMB(9800);
	Set_ZPWMC(9800);
	Set_FPWMD(9800);
}
void turnleft(void)
{
	Set_ZPWMA(9800);
	Set_FPWMB(9800);
	Set_FPWMC(9800);
	Set_ZPWMD(9800);
	
}
void turnstop(void )
{
	Set_SPWMA(0);
	Set_SPWMB(0);
	Set_SPWMC(0);
	Set_SPWMD(0);
}

void turnout(void)
{
	Set_SPWMA(6000);
	Set_FPWMB(6000);
	Set_ZPWMC(6000);
	Set_SPWMD(6000);
}
void left(void)
{


	  Set_ZPWMA(10200);
    Set_FPWMB(10200);
    Set_ZPWMC(10200);
    Set_FPWMD(10200);



}
void right(void)
{
	Set_FPWMA(10200);
	Set_ZPWMB(10350);
	Set_FPWMC(10200);
	Set_ZPWMD(10200);
}	

void adjustadvance(void)
{ 
		
		Set_FPWMA(9900);
    Set_FPWMB(9750);
    Set_ZPWMC(9500);
    Set_ZPWMD(9750);
	
}
void adjustadvanceb(void)
{ 
		
		Set_FPWMA(9750);
    Set_FPWMB(10000);
    Set_ZPWMC(9500);
    Set_ZPWMD(9750);
	
}
void adjustrecoil(void)
{
	
		Set_ZPWMA(9750);
    Set_ZPWMB(9750);
    Set_FPWMC(9950);
    Set_FPWMD(9750);
	

}
void adjustrecoilb(void)
{
	
		Set_ZPWMA(9750);
    Set_ZPWMB(9750);
    Set_FPWMC(9650);
    Set_FPWMD(9850);
	

}
void adjustleft(void)
{
	Set_ZPWMA(9750);
	Set_FPWMB(9750);
	Set_ZPWMC(9750);
	Set_FPWMD(9750);
}
void adjustright(void)
{
	Set_FPWMA(9750);
	Set_ZPWMB(9750);
	Set_FPWMC(9750);
	Set_ZPWMD(9750);
}	

void change(void)
{
    static uint8_t stage = 0;

    if (data_ready) {
        switch (stage) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stage++;
                }
                break;
            default:
                stage = 0;
						changeready=1;
                break;
        }
        data_ready = 0;  
    }
}
void change2(void)
{
    static uint8_t stageg = 0;

    if (data_ready) {
        switch (stageg) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stageg++;
                }
                break;
            default:
                stageg = 0;
						changeready2=1;
                break;
        }
        data_ready = 0;  
    }
}

void judgementput1(void)
{
	static uint8_t stageb=0;
	
	if (data_ready) {
        switch (stageb) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stageb++;
                }
                break;

            case 1:
                
                if (x == 1 && y == 1) {
                   ground_put_1();
                    stageb=0;
									judgeput1=1;
                }
                break;
								 default:
                stageb = 0;
                break;
           }
	data_ready=0;
	
		}			
}
void judgementput2(void)
{
	static uint8_t stageb;
	
	if (data_ready) {
        switch (stageb) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stageb++;
                }
                break;

            case 1:
                
                if (x == 1 && y == 1) {
                   ground_put_2();
                    stageb=0;
									judgeput2=1;
                }
                break;
								 default:
                stageb = 0;
                break;
           }
	data_ready=0;
	
		}			
}
	void judgementput3(void)
{
	static uint8_t stageb;
	
	if (data_ready) {
        switch (stageb) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stageb++;
                }
                break;

            case 1:
                
                if (x == 1 && y == 1) {
                   ground_put_3();
                    stageb=0;
									judgeput3=1;
                }
                break;
								 default:
                stageb = 0;
                break;
           }
	data_ready=0;
	
		}			
}
void judgementput4(void)
{
	static uint8_t stageb=0;
	
	if (data_ready) {
        switch (stageb) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stageb++;
                }
                break;

            case 1:
                
                if (x == 1 && y == 1) {
                   ground_put_1();
                    stageb=0;
									judgeput4=1;
                }
                break;
								 default:
                stageb = 0;
                break;
           }
	data_ready=0;
	
		}			
}
void judgementput5(void)
{
	static uint8_t stageb;
	
	if (data_ready) {
        switch (stageb) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stageb++;
                }
                break;

            case 1:
                
                if (x == 1 && y == 1) {
                   ground_put_2();
                    stageb=0;
									judgeput5=1;
                }
                break;
								 default:
                stageb = 0;
                break;
           }
	data_ready=0;
	
		}			
}
	void judgementput6(void)
{
	static uint8_t stageb;
	
	if (data_ready) {
        switch (stageb) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stageb++;
                }
                break;

            case 1:
                
                if (x == 1 && y == 1) {
                   ground_put_3();
                    stageb=0;
									judgeput6=1;
                }
                break;
								 default:
                stageb = 0;
                break;
           }
	data_ready=0;
	
		}			
}
  void judgementtake1(void)
	{
		
	static uint8_t stagec=0;
	
	if (data_ready) {
        switch (stagec) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stagec++;
                }
                break;

            case 1:
                
                if (x == 1 && y == 1) {
                   ground_take_1();
                    stagec=0;
									judgetake1=1;
                }
                break;
								 default:
                stagec = 0;
                break;
           }
	data_ready=0;
	
		}			
}

  void judgementtake2(void)
	{
		
	static uint8_t stagec;
	
	if (data_ready) {
        switch (stagec) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stagec++;
                }
                break;

            case 1:
                
                if (x == 1 && y == 1) {
                   ground_take_2();
                    stagec=0;
									judgetake2=1;
                }
                break;
								 default:
                stagec = 0;
                break;
           }
	data_ready=0;
	
		}			
}

  void judgementtake3(void)
	{
		
	static uint8_t stagec;
	
	if (data_ready) {
        switch (stagec) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stagec++;
                }
                break;

            case 1:
                
                if (x == 1 && y == 1) {
                   ground_take_3();
                    stagec=0;
									judgetake3=1;
                }
                break;
								 default:
                stagec = 0;
                break;
           }
	data_ready=0;
	
		}			
}

void do_save_second_place(void)
{
	while(1)
	{
	if(data_ready)
	{
		if(x==0&&y==0)
			{
			turnstop();
			}
		else if((x!=0||y!=0)&&(x!=1&&y!=1))
			{ 		
					if(x != 0)
						 {								
							if(x > 1) 
								{
									adjustrecoil();   
								} 
							else if(x < 0)
								{
								 adjustadvance();
								} 
							else 
								{
                 turnstop();   
								}
             }
		     else
			      {								
					if(y!=0)
						{
            if(y > 1) 
							{
                adjustleft();      
              } 
						else if(y <0) 
							{
                adjustright();     
              } 
							else 
							{
                turnstop();   
              }
            }
           }
		}		
    else 
		{
			if(x==1&&y==1)
				{servo4(5000);
					Delay_s(2);
				servo4(4000);
			}				break;			 
		
			
		}
		data_ready=0;
	}

	}
}
void do_process_grab(void)
{
	while(1)
	{
	if(data_ready)
	{
		if(x==0&&y==0)
			{
			turnstop();
			}
		else if((x!=0||y!=0)&&(x!=1&&y!=1))
			{ 		
					if(x != 0)
						 {								
							if(x > 1) 
								{
									adjustrecoil();   
								} 
							else if(x < 0)
								{
								 adjustadvance();
								} 
							else 
								{
                 turnstop();   
								}
             }
		     else
			      {								
					if(y!=0)
						{
            if(y > 1) 
							{
                adjustleft();      
              } 
						else if(y <0) 
							{
                adjustright();     
              } 
							else 
							{
                turnstop();   
              }
            }
           }
		}		
    else 
		{
			if(x==1&&y==1)
				{servo4(5000);
					Delay_s(2);
				servo4(4000);
			}				break;			 
		
			
		}
		data_ready=0;
	}

	}
}



//void judgement(void)
//{
//	if(sequence[1]==1)
//	{
//		recoil();
//		Delay_ms(800);
//		turnstop();//red
//		Serial_SendByte(9);
//		judgementput();
//		Delay_ms(2000);
//		if(sequence[2]==2)
//		{
//			advance();
//			Delay_ms(800);
//			turnstop();//green
//			Serial_SendByte(9);
//		  judgementput();
//		  Delay_ms(2000);
//			advance();
//			Delay_ms(800);
//			turnstop();//blue
//			Serial_SendByte(9);
//		  judgementput();
//			Delay_ms(2000);
//			recoil();
//		  Delay_ms(800);
//		  turnstop();//reset
//		  Delay_ms(800);
//		}
//		else if(sequence[2]==3)
//		{
//			advance();
//			Delay_ms(1600);
//			turnstop();//blue
//			Serial_SendByte(9);
//		  judgementput();
//		  Delay_ms(2000);
//			recoil();
//		  Delay_ms(800);
//		  turnstop();//green
//		  Delay_ms(800);
//		}
//	}
//	if(sequence[1]==2)	 
//	{
//		turnstop();//green
//		Serial_SendByte(9);
//		judgementput();
//		Delay_ms(2000);
//		if(sequence[2]==1) 
//		{
//			recoil();
//			Delay_ms(800);
//			
//			turnstop();//red
//			Serial_SendByte(9);
//			judgementput();
//			Delay_ms(2000);
//			
//			advance();
//			Delay_ms(1600);
//			turnstop();//blue
//			Serial_SendByte(9);
//			judgementput(); 
//			Delay_ms(800);
//			recoil();
//		  Delay_ms(800);
//		  turnstop();//reset
//		  Delay_ms(2000); 
//		}
//	
//		else if(sequence[2]==3)
//		{
//			advance();
//			Delay_ms(800);
//			turnstop();//blue
//			Serial_SendByte(9);
//			judgementput();
//		  Delay_ms(2000);
//			recoil();
//		  Delay_ms(1600);
//		  turnstop();//red
//		  Delay_ms(2000);
//			advance();
//			Delay_ms(800);
//			turnstop();//reset
//		  Delay_ms(2000);
//		}
//	}
//	else if(sequence[1]==3)
//	{
//			advance();
//			Delay_ms(800);
//			turnstop();//blue
//		  Serial_SendByte(9);
//			judgementput();
//		  Delay_ms(2000);
//		 if(sequence[2]==1)
//		{
//			recoil();
//			Delay_ms(1600);
//			turnstop();//red
//			Serial_SendByte(9);
//			judgementput();
//		  Delay_ms(2000);
//			
//			advance();
//			Delay_ms(800);
//			turnstop();//green
//			Serial_SendByte(9);
//			judgementput();
//			Delay_ms(2000);
//		}
//		else if(sequence[2]==2)
//		{
//			recoil();
//			Delay_ms(800);
//			turnstop();//green
//			Serial_SendByte(9);
//			judgementput();
//		  Delay_ms(2000);
//			recoil();
//			Delay_ms(800);
//			turnstop();//red
//			Serial_SendByte(9);
//			judgementput();
//		  Delay_ms(2000);
//			advance();
//			Delay_ms(800);
//			turnstop();//reset
//		  Delay_ms(2000);
//		}
//	}
//	readyjudgement=1;
//}
// ??????
void do_turntable(void) 
			{
    static uint8_t stage = 0;

    if (data_ready) {
        switch (stage) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stage++;
                }
                break;

            case 1:
                
                if (x == 1 && y == 1) {
                    turntable_take1();
                    stage++;
                }
                break;

            case 2:
                
                  turntable_take2();      
                stage++;
                break;

            case 3:
               
                 turntable_take3();      
                readyagain = 1;    
                stage = 0;        
                break;

            default:
                stage = 0;
                break;
        }
        data_ready = 0;  
    }
}

void do_turntable2(void) 
			{
    static uint8_t stage = 0;

    if (data_ready) {
        switch (stage) {
            case 0:
                
                if (x != 0 || y != 0) {
                    if (x != 0) {
                        if (x > 5)       adjustrecoil();
                        else if (x < -5)  adjustadvance();
                        else              turnstop();
                    } else {
                        if (y > 5)       adjustleft();
                        else if (y < -5) adjustright();
                        else             turnstop();
                    }
                } else {
                   
                    turnstop();
                    stage++;
                }
                break;

            case 1:
                
                if (x == 1 && y == 1) {
                    turntable_take1();
                    stage++;
                }
                break;

            case 2:
                
                  turntable_take2();      
                stage++;
                break;

            case 3:
               
                 turntable_take3();      
                readyagain2= 1;    
                stage = 0;        
                break;

            default:
                stage = 0;
                break;
        }
        data_ready = 0;  
    }
}


void judgementput(void) {
	
    switch (judge_state) {
        case JUDGE_INIT:
            judge_step = 0;
            judge_sequence_1 = sequence[1];
            judge_sequence_2 = sequence[2];
            judge_state = JUDGE_RUNNING;
            break;

        case JUDGE_RUNNING:
            
            if (judge_sequence_1 == 0x31) {
                
                if (judge_sequence_2 == 0x32) {
                    switch (judge_step) {
                        case 0:
                           adjustrecoilb();
                           Delay_ms(1450);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        case 1:
                             
                                Serial_SendByte(9);
                             while(!judgeput1)
														{ judgementput1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 2:
                           adjustadvanceb();
                               Delay_ms(1450);
												       turnstop();
														
												       Delay_s(1);
													
                               judge_delay_start = get_millis();
                               judge_step++;                  
                            break;
                        case 3:
                            Serial_SendByte(9);
												
                                	while(!judgeput2)
															{ judgementput2();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
                        case 4:
                               adjustadvanceb();
                               Delay_ms(1600);
												       turnstop();
														
												       Delay_s(1);
													
                               judge_delay_start = get_millis();
                               judge_step++;
												case 5:
													Serial_SendByte(9);
												
                                	while(!judgeput3)
															{ judgementput3();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
												case 6:
													adjustrecoilb();
												Delay_ms(3000);
												turnstop();
												Delay_s(1);
												
                            judge_delay_start = get_millis();
                            judge_step++;   
												break;
                       default:
                            readyjudgement = 1;
                            judge_state = JUDGE_DONE;
                            break;
                       
                    }
                }
               
                else if (judge_sequence_2 == 0x33) {
                    switch (judge_step) {
                        case 0:
                           adjustrecoilb();
                           Delay_ms(1450);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        case 1:
                            
                                Serial_SendByte(9);
                             while(!judgeput1)
														{ judgementput1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 2:
                            adjustadvanceb();
                           Delay_ms(3000);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
												case 3:
													 
                                Serial_SendByte(9);
                             while(!judgeput2)
														{ judgementput2();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
												case 4: 
													adjustrecoilb();
                           Delay_ms(1600);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
												case 5:
                            
                                Serial_SendByte(9);
                             while(!judgeput3)
														{ judgementput3();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;				
                        case 6:
												adjustrecoilb();
												Delay_ms(1450);
												turnstop();
												Delay_s(1);
												
                                    judge_delay_start = get_millis();
                            judge_step++;   
												break;														
                        default:
                            readyjudgement = 1;
                            judge_state = JUDGE_DONE;
                            break;
                    }
                }
            }
           
            else if (judge_sequence_1 == 0x32) {
							
            
                if (judge_sequence_2 == 0x31) {
                    switch (judge_step) {
                        case 0:
                             
                                Serial_SendByte(9);
                             while(!judgeput1)
														{ judgementput1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 1:
                            adjustrecoilb();
                           Delay_ms(1450);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        case 2:
                           
                                Serial_SendByte(9);
                             while(!judgeput2)
														{ judgementput2();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                        case 3:
                           adjustadvanceb();
                           Delay_ms(3000);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
												case 4:
                                Serial_SendByte(9);
                             while(!judgeput3)
														{ judgementput3();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
												case 5: 
													adjustrecoilb();
                           Delay_ms(1600);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        default:
                            readyjudgement = 1;
                            judge_state = JUDGE_DONE;
                            break;
                    }
                }
                
                else if (judge_sequence_2 == 0x33) {
                    switch (judge_step) {
                        case 0:
                             
                                Serial_SendByte(9);
                             while(!judgeput1)
														{ judgementput1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 1:
                            adjustadvanceb();
                           Delay_ms(1600);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        case 2:
                           
                                Serial_SendByte(9);
                             while(!judgeput2)
														{ judgementput2();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 3:
                            adjustrecoilb();
                           Delay_ms(3000);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
												case 4:
													 Serial_SendByte(9);
                             while(!judgeput3)
														{ judgementput3();}
                                judge_delay_start = get_millis();
                                judge_step++;
														break;
												case 5:
													adjustadvanceb();
                           Delay_ms(1600);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        default:
                            readyjudgement = 1;
                            judge_state = JUDGE_DONE;
                            break;
                    }
                }
            }
           
            else if (judge_sequence_1 == 0x33) {
               
                if (judge_sequence_2 == 0x31) {
                    switch (judge_step) {
                        case 0:
													adjustadvanceb();
                           Delay_ms(1600);
                           turnstop();
												   Delay_s(1);
                            judge_delay_start = get_millis();
                            judge_step++;
                            break;
                        case 1:
                            
                                Serial_SendByte(9);
                             while(!judgeput1)
														{ judgementput1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 2:
                            adjustrecoilb();
                            Delay_ms(3000);
                            turnstop();
													
												    Delay_s(1);
													
                            judge_delay_start = get_millis();
                            judge_step++;
                            break;
                          
                        case 3:
                           	Serial_SendByte(9);
												
                                	while(!judgeput2)
															{ judgementput2();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
												case 4:adjustadvanceb();
                               Delay_ms(1450);
												       turnstop();
														
												       Delay_s(1);
													
                               judge_delay_start = get_millis();
                               judge_step++;                  
                            break;
												case 5:
														Serial_SendByte(9);
											
                              while(!judgeput3)
															{ judgementput3();}
                                judge_delay_start = get_millis();
                                judge_step++;    
															break;
                        default:
                            readyjudgement = 1;
                            judge_state = JUDGE_DONE;
                            break;
                    }
                }
              
                else if (judge_sequence_2 == 0x32) {
                    switch (judge_step) {
                        case 0:
                            adjustadvanceb();
												    Delay_ms(1600);
                            turnstop();
												    Delay_s(1);
                            judge_delay_start = get_millis();
                            judge_step++;
                            break;
                        case 1:
                                Serial_SendByte(9);
												
                                	while(!judgeput1)
															{ judgementput1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 2:
                            adjustrecoilb();
                            Delay_ms(1450);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                            break;
												case 3:
											Serial_SendByte(9);
												
                                	while(!judgeput2)
															{ judgementput2();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
                        case 4:
                            adjustrecoilb();
                             Delay_ms(1450);
												turnstop();
														
												Delay_s(1);
													
                                    judge_delay_start = get_millis();
                            judge_step++;                  
                            break;
												case 5:
														Serial_SendByte(9);
											
                                	while(!judgeput3)
															{ judgementput3();}
                                judge_delay_start = get_millis();
                                judge_step++;    
															break;
												case 6:
													adjustadvanceb();
												Delay_ms(3000);
												turnstop();
												Delay_s(1);
												
                                    judge_delay_start = get_millis();
                            judge_step++;   
												break;
                        default:
                            readyjudgement = 1;
                            judge_state = JUDGE_DONE;
                            break;
                    }
                }
            }
            break;

        case JUDGE_DONE:
            judge_state = JUDGE_INIT;
            readyjudgement = 1;
            break;
    }
}
void judgementputb(void) {
	
    switch (judge_state) {
        case JUDGE_INIT:
            judge_step = 0;
            judge_sequence_1 = sequence[1];
            judge_sequence_2 = sequence[2];
            judge_state = JUDGE_RUNNING;
            break;

        case JUDGE_RUNNING:
            
            if (judge_sequence_1 == 0x31) {
                
                if (judge_sequence_2 == 0x32) {
                    switch (judge_step) {
                        case 0:
                           adjustrecoil();
                           Delay_ms(1450);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        case 1:
                             
                                Serial_SendByte(9);
                             while(!judgeput4)
														{ judgementput4();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 2:
                           adjustadvance();
                               Delay_ms(1450);
												       turnstop();
														
												       Delay_s(1);
													
                               judge_delay_start = get_millis();
                               judge_step++;                  
                            break;
                        case 3:
                            Serial_SendByte(9);
												
                                	while(!judgeput2)
															{ judgementput2();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
                        case 4:
                               adjustadvance();
                               Delay_ms(1600);
												       turnstop();
														
												       Delay_s(1);
													
                               judge_delay_start = get_millis();
                               judge_step++;
												case 5:
													Serial_SendByte(9);
												
                                	while(!judgeput3)
															{ judgementput3();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
												case 6:
													adjustrecoil();
												Delay_ms(3000);
												turnstop();
												Delay_s(1);
												
                            judge_delay_start = get_millis();
                            judge_step++;   
												break;
                       default:
                            readyjudgement2 = 1;
                            judge_state = JUDGE_DONE;
                            break;
                       
                    }
                }
               
                else if (judge_sequence_2 == 0x33) {
                    switch (judge_step) {
                        case 0:
                           adjustrecoil();
                           Delay_ms(1450);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        case 1:
                            
                                Serial_SendByte(9);
                             while(!judgeput4)
														{ judgementput4();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 2:
                            adjustadvance();
                           Delay_ms(3000);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
												case 3:
													 
                                Serial_SendByte(9);
                             while(!judgeput5)
														{ judgementput5();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
												case 4: 
													adjustrecoil();
                           Delay_ms(1600);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
												case 5:
                            
                                Serial_SendByte(9);
                             while(!judgeput6)
														{ judgementput6();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;				
                        case 6:
												adjustrecoil();
												Delay_ms(1450);
												turnstop();
												Delay_s(1);
												
                                    judge_delay_start = get_millis();
                            judge_step++;   
												break;														
                        default:
                            readyjudgement2 = 1;
                            judge_state = JUDGE_DONE;
                            break;
                    }
                }
            }
           
            else if (judge_sequence_1 == 0x32) {
							
            
                if (judge_sequence_2 == 0x31) {
                    switch (judge_step) {
                        case 0:
                             
                                Serial_SendByte(9);
                             while(!judgeput4)
														{ judgementput4();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 1:
                            adjustrecoil();
                           Delay_ms(1450);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        case 2:
                           
                                Serial_SendByte(9);
                             while(!judgeput5)
														{ judgementput5();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                        case 3:
                           adjustadvance();
                           Delay_ms(3000);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
												case 4:
                                Serial_SendByte(9);
                             while(!judgeput6)
														{ judgementput6();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
												case 5: 
													adjustrecoil();
                           Delay_ms(1600);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        default:
                            readyjudgement2 = 1;
                            judge_state = JUDGE_DONE;
                            break;
                    }
                }
                
                else if (judge_sequence_2 == 0x33) {
                    switch (judge_step) {
                        case 0:
                             
                                Serial_SendByte(9);
                             while(!judgeput4)
														{ judgementput4();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 1:
                            adjustadvance();
                           Delay_ms(1600);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        case 2:
                           
                                Serial_SendByte(9);
                             while(!judgeput5)
														{ judgementput5();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 3:
                            adjustrecoil();
                           Delay_ms(3000);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
												case 4:
													 Serial_SendByte(9);
                             while(!judgeput6)
														{ judgementput6();}
                                judge_delay_start = get_millis();
                                judge_step++;
														break;
												case 5:
													adjustadvance();
                           Delay_ms(1600);
                           turnstop();
												   Delay_s(1);
                           judge_delay_start = get_millis();
                           judge_step++;
                            break;
                        default:
                            readyjudgement2 = 1;
                            judge_state = JUDGE_DONE;
                            break;
                    }
                }
            }
           
            else if (judge_sequence_1 == 0x33) {
               
                if (judge_sequence_2 == 0x31) {
                    switch (judge_step) {
                        case 0:
													adjustadvance();
                           Delay_ms(1600);
                           turnstop();
												   Delay_s(1);
                            judge_delay_start = get_millis();
                            judge_step++;
                            break;
                        case 1:
                            
                                Serial_SendByte(9);
                             while(!judgeput4)
														{ judgementput4();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 2:
                            adjustrecoil();
                            Delay_ms(3000);
                            turnstop();
													
												    Delay_s(1);
													
                            judge_delay_start = get_millis();
                            judge_step++;
                            break;
                          
                        case 3:
                           	Serial_SendByte(9);
												
                                	while(!judgeput5)
															{ judgementput5();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
												case 4:adjustadvance();
                               Delay_ms(1450);
												       turnstop();
														
												       Delay_s(1);
													
                               judge_delay_start = get_millis();
                               judge_step++;                  
                            break;
												case 5:
														Serial_SendByte(9);
											
                              while(!judgeput6)
															{ judgementput6();}
                                judge_delay_start = get_millis();
                                judge_step++;    
															break;
                        default:
                            readyjudgement2 = 1;
                            judge_state = JUDGE_DONE;
                            break;
                    }
                }
              
                else if (judge_sequence_2 == 0x32) {
                    switch (judge_step) {
                        case 0:
                            adjustadvance();
												    Delay_ms(1600);
                            turnstop();
												    Delay_s(1);
                            judge_delay_start = get_millis();
                            judge_step++;
                            break;
                        case 1:
                                Serial_SendByte(9);
												
                                	while(!judgeput4)
															{ judgementput4();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 2:
                            adjustrecoil();
                            Delay_ms(1450);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                            break;
												case 3:
											Serial_SendByte(9);
												
                                	while(!judgeput5)
															{ judgementput5();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
                        case 4:
                            adjustrecoil();
                             Delay_ms(1450);
												turnstop();
														
												Delay_s(1);
													
                                    judge_delay_start = get_millis();
                            judge_step++;                  
                            break;
												case 5:
														Serial_SendByte(9);
											
                                	while(!judgeput6)
															{ judgementput6();}
                                judge_delay_start = get_millis();
                                judge_step++;    
															break;
												case 6:
													adjustadvance();
												Delay_ms(3000);
												turnstop();
												Delay_s(1);
												
                                    judge_delay_start = get_millis();
                            judge_step++;   
												break;
                        default:
                            readyjudgement2 = 1;
                            judge_state = JUDGE_DONE;
                            break;
                    }
                }
            }
            break;

        case JUDGE_DONE:
            judge_state = JUDGE_INIT;
            readyjudgement2 = 1;
            break;
    }
}

void judgetakeprocess(void) {
	
    switch (judge_stateb) {
        case JUDGE_INITB:
            judge_step = 0;
            judge_sequence_1 = sequence[1];
            judge_sequence_2 = sequence[2];
            judge_stateb = JUDGE_RUNNINGB;
            break;

        case JUDGE_RUNNINGB:
            
            if (judge_sequence_1 == 0x31) {
                
                if (judge_sequence_2 == 0x32) {
                    switch (judge_step) {
                        case 0:
                            while(!judgetake1)
															{ judgementtake1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 1:
                             adjustadvanceb();
                            Delay_ms(1600);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                            break;
                        case 2:
                            while(!judgetake2)
															{ judgementtake2();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            break;
                        case 3:
                            adjustadvanceb();
                            Delay_ms(1600);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                            break;
                        case 4:
                            while(!judgetake3)
															{ judgementtake3();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            break;
												case 5:
													adjustrecoilb();
                            Delay_ms(1600);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                            break;
                        default:
                            readyprocessput = 1;
                            judge_stateb = JUDGE_DONEB;
                            break;
                    }
                }
               
                else if (judge_sequence_2 == 0x33) {
                    switch (judge_step) {
                        case 0:
                             while(!judgetake1)
															{ judgementtake1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 1:
                             adjustadvanceb();
                            Delay_ms(3150);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                            break;
                        case 2:
                            while(!judgetake2)
															{ judgementtake2();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
												case 3:
													adjustrecoilb();
                            Delay_ms(1600);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                            break;
												case 4:
												while(!judgetake3)
															{ judgementtake3();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
												
                        default:
                            readyprocessput = 1;
                            judge_state = JUDGE_DONEB;
                            break;
                    }
                }
            }
           
            else if (judge_sequence_1 == 0x32) {
				
                if (judge_sequence_2 == 0x31) {
                    switch (judge_step) {
                        case 0:
                            while(!judgetake1)
															{ judgementtake1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 1:
                             adjustrecoilb();
                            Delay_ms(1600);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                            break;
                        case 2:
                            	while(!judgetake2)
															{ judgementtake2();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
                        case 3:
                            adjustadvanceb();
                            Delay_ms(3150);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
												case 4:
													while(!judgetake3)
															{ judgementtake3();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
											  case 5:
													adjustrecoilb();
                            Delay_ms(1600);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                        default:
                            readyprocessput = 1;
                            judge_state = JUDGE_DONEB;
                            break;
                    }
                }
                
                else if (judge_sequence_2 == 0x33) {
                    switch (judge_step) {
                        case 0:
                           while(!judgetake1)
															{ judgementtake1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 1:
                            adjustadvanceb();
                            Delay_ms(1600);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                        case 2:
                            while(!judgetake2)
															{ judgementtake2();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
                        case 3:
                             adjustrecoilb();
                            Delay_ms(3150);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
												break;
												case 4:while(!judgetake3)
															{ judgementtake3();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
												case 5:
													adjustadvanceb();
                            Delay_ms(1600);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
												break;
                        default:
                            readyprocessput = 1;
                            judge_state = JUDGE_DONEB;
                            break;
                    }
                }
            }
           
            else if (judge_sequence_1 == 0x33) {
               
                if (judge_sequence_2 == 0x31) {
                    switch (judge_step) {
                        case 0:
                           while(!judgetake1)
															{ judgementtake1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 1:
                             adjustrecoilb();
                            Delay_ms(3150);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                            break;
                        case 2:
                               	while(!judgetake2)
															{ judgementtake2();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
                        case 3:
                            adjustadvanceb();
                            Delay_ms(1600);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                            break;
												 case 4:
													    while(!judgetake3)
															{ judgementtake3();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
                        default:
                            readyprocessput = 1;
                            judge_state = JUDGE_DONEB;
                            break;
                    }
                }
              
                else if (judge_sequence_2 == 0x32) {
                    switch (judge_step) {
                        
                        case 0:
                              
												
                              while(!judgetake1)
															{ judgementtake1();}
                                judge_delay_start = get_millis();
                                judge_step++;
                            
                            break;
                        case 1:
                            adjustrecoilb();
                            Delay_ms(1600);
                                turnstop();
													
												Delay_s(1);
													
                        judge_delay_start = get_millis();
                            judge_step++;
                            break;
												case 2:
											
												
                                	while(!judgetake2)
															{ judgementtake2();}
                                judge_delay_start = get_millis();
                                judge_step++;
															break;
                        case 3:
                            adjustrecoilb();
                             Delay_ms(1600);
												turnstop();
														
												Delay_s(1);
													
                                    judge_delay_start = get_millis();
                            judge_step++;                  
                            break;
												case 4:
														
											
                                	while(!judgetake3)
															{ judgementtake3();}
                                judge_delay_start = get_millis();
                                judge_step++;    
															break;
												case 5:
													adjustrecoilb();
												Delay_ms(1600);
												turnstop();
												Delay_s(1);
												
                                    judge_delay_start = get_millis();
                            judge_step++;   
												break;
                        default:
                            readyprocessput = 1;
                            judge_state = JUDGE_DONEB;
                            break;
                    }
                }
            }
            break;

        case JUDGE_DONEB:
            judge_state = JUDGE_INITB;
            readyprocessput = 1;
            break;
    }
}


void judgment_grab(void)
{
	if(sequence[1]==1)
	{
		recoil();
		Delay_ms(800);
		turnstop();//red
		Serial_SendByte(9);
		do_process_grab();
		Delay_ms(2000);
		if(sequence[2]==2)
		{
			advance();
			Delay_ms(800);
			turnstop();//green
			Serial_SendByte(9);
		  do_process_grab();
		  Delay_ms(2000);
			advance();
			Delay_ms(800);
			turnstop();//blue
			Serial_SendByte(9);
		  do_process_grab();
			Delay_ms(2000);
			recoil();
		  Delay_ms(800);
		  turnstop();//reset
		  Delay_ms(800);
		}
		else if(sequence[2]==3)
		{
			advance();
			Delay_ms(1600);
			turnstop();//blue
			Serial_SendByte(9);
		  do_process_grab();
		  Delay_ms(2000);
			recoil();
		  Delay_ms(800);
		  turnstop();//green
		  Delay_ms(800);
		}
	}
	if(sequence[1]==2)	 
	{
		turnstop();//green
		Serial_SendByte(9);
		do_process_grab();
		Delay_ms(2000);
		if(sequence[2]==1)
		{
			recoil();
			Delay_ms(800);
			
			turnstop();//red
			Serial_SendByte(9);
			do_process_grab();
			Delay_ms(2000);
			
			advance();
			Delay_ms(1600);
			turnstop();//blue
			Serial_SendByte(9);
			do_process_grab(); 
			Delay_ms(800);
			recoil();
		  Delay_ms(800);
		  turnstop();//reset
		  Delay_ms(2000); 
		}
	
		else if(sequence[2]==3)
		{
			advance();
			Delay_ms(800);
			turnstop();//blue
			Serial_SendByte(9);
			do_process_grab();
		  Delay_ms(2000);
			recoil();
		  Delay_ms(1600);
		  turnstop();//red
		  Delay_ms(2000);
			advance();
			Delay_ms(800);
			turnstop();//reset
		  Delay_ms(2000);
		}
	}
	else if(sequence[1]==3)
	{
			advance();
			Delay_ms(800);
			turnstop();//blue
		  Serial_SendByte(9);
			do_process_grab();
		  Delay_ms(2000);
		 if(sequence[2]==1)
		{
			recoil();
			Delay_ms(1600);
			turnstop();//red
			Serial_SendByte(9);
			do_process_grab();
		  Delay_ms(2000);
			
			advance();
			Delay_ms(800);
			turnstop();//green
			Serial_SendByte(9);
			do_process_grab();
			Delay_ms(2000);
		}
		else if(sequence[2]==2)
		{
			recoil();
			Delay_ms(800);
			turnstop();//green
			Serial_SendByte(9);
			do_process_grab();
		  Delay_ms(2000);
			recoil();
			Delay_ms(800);
			turnstop();//red
			Serial_SendByte(9);
			do_process_grab();
		  Delay_ms(2000);
			advance();
			Delay_ms(800);
			turnstop();//reset
		  Delay_ms(2000);
		}
	}
}



void allgo(void) 
{
    static uint8_t phase = PHASE_INIT;     
//    static uint32_t phase_start_time = 0;  
         
    

    while (1) 
    { 
			static uint8_t h;
			h=USART_ReceiveData(USART2);
        if (new_data_received) 
        {
            output = P_calculate();
            new_data_received = 0;

            switch (phase) 
            {
                case 0:
                   Usart3_SendArray(reset_z_axis,5); 
                    phase++;
                  
                    break;

                case 1:
									 
                    left(); 
                     Delay_ms(1500);
                        turnstop();
							Delay_s(1);
								advance();
								Delay_ms(3000);
							

								phase++;
                    break;

                case 2:							
                    advanceb();  
                   Delay_ms(1350);
                        turnstop();
									phase++;
                    break;
								case 3:
									  servo4(3950);
									while(!readyagain)
									{
									 do_turntable();
									}
									servo4(5000);
									
									phase++;
									
                 break;
								case 4:
									turnstop();
								if(h==251)
								{
									
								phase++;
								}
									
								break;
								case 5:
									Usart3_SendArray(reset_z_axis,5);
								turnstop();
								      phase++;
								break ;
								case 6:
									recoilb();
								Delay_ms(600);
                        turnstop();
                    Delay_s(1);
                    	phase++;
										break;
								case 7:
									output=0;
									turnleft();
								Delay_ms(3150);
								
                        turnstop();
								
                       	Usart3_SendArray(reset_z_axis,5);
								Delay_s(1);
											phase++;
                    
							break;
								case 8:
									Usart3_SendArray(reset_z_axis,5);
								turnstop();
								      phase++;
								break ;
								case 9:
										Usart3_SendArray(reset_z_axis,5);
									advanceb();
									Delay_ms(3070);
                        turnstop();
								Delay_s(1);
                   	phase++;  
										break;
								case 10:
									output=0;
									turnleft ();
								Delay_ms(2000);
									Usart3_SendArray(reset_z_axis,5);
								Delay_s(1);
                        turnstop();
                       	phase++; 
										
                    
										break;
							case 11:
									Usart3_SendArray(reset_z_axis,5);
								turnstop();
								      phase++;
								break ;
								case 12:
									//Usart3_SendArray(reset_z_axis,5);
										servo4(4100);
									Serial_SendByte(9);
							output=0;

									while(!changeready)
									{
										change();
									}
									while(!readyjudgement)
									{
									  judgementput();
									}
									while(!readyprocessput)
									{
										judgetakeprocess();
									}
									Usart3_SendArray(reset_z_axis,5);
	phase++;
									break;
								case 13:
									turnstop();
             if(h==250)	
						 {
							 phase++;
						 }					
                 break;						 
									case 14:
									Usart3_SendArray(reset_z_axis,5);
								turnstop();
								      phase++;
								break ;
								case 15:
									Usart3_SendArray(reset_z_axis,5);
									servo4(5000);
								recoilb();
								Delay_ms(3500);
									phase++;
								break ;
								case 16:
									turnright();
								Delay_ms(2750);
								turnstop();
									phase++;
								break;
							case 17:
									Usart3_SendArray(reset_z_axis,5);
								turnstop();
								      phase++;
								break ;
								case 18:
									recoil();
								Delay_ms(3500);
								turnstop();
								Delay_s(1);
									phase++;
								break;
								case 19:
									while(!changeready2)
									{
										change2();
									}
									while(!readyjudgement2)
									{
									  judgementputb(); 
									}
										phase++;
									break;
									case 20:
									Usart3_SendArray(reset_z_axis,5);
								turnstop();
								      phase++;
								break ;
								case 21:
									recoil();
								Delay_ms(3500);
								turnstop();
								Delay_s(1);
									phase++;
								break ;
								case 22:
									turnright();
								Delay_ms(2950);
								turnstop();
									phase++;
								break ;
								case 23:
									Usart3_SendArray(reset_z_axis,5);
								turnstop();
								      phase++;
								break ;
								case 24:
									  servo4(4100);
									while(!readyagain2)
									{
									 do_turntable2();
									}
										phase++;
									break;
									
                case 25:
									return;
                default:    
                    phase = PHASE_INIT;
                    break;
            }
        }
        
        
    }
}
void turntabletest (void)
{
	while(!readyagain)
									{
									 do_turntable();
									}
}

void turnlefttry(void)
{
	
	while(1)
	{
while(!changeready)
									{
										change();
									}
									while(!readyjudgement)
									{
									  judgementput(); 
									}
									while(!readyprocessput)
									{
										judgetakeprocess();
									}
								}
							}
