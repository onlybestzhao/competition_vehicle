#include "stm32f4xx.h"                  // Device header

void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 168 * xus;				//????????
	SysTick->VAL = 0x00;					//???????
	SysTick->CTRL = 0x00000005;				//??????HCLK,?????
	while(!(SysTick->CTRL & 0x00010000));	//?????0
	SysTick->CTRL = 0x00000004;				//?????
}

/**
  * @brief  ?????
  * @param  xms ????,??:0~4294967295
  * @retval ?
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  ????
  * @param  xs ????,??:0~4294967295
  * @retval ?
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
































