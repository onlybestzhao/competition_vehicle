#ifndef _VISION_H
#define _VISION_H
#include "sys.h"
void USART2_Init(void);
void USART2_IRQHandler(void);

void UART5_IRQHandler(void);

void UART5_Init(void);
    
   


void Serial_SendByte(uint8_t Byte);

void Serial_SendArray(volatile uint8_t *Array, uint16_t Length);

#endif
