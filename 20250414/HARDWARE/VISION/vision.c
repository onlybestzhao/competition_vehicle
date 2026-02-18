#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "sport.h"

#include "delay.h"
#include "SERVO.h"
#include "stdio.h"




volatile uint16_t rx_index = 0;
extern volatile uint8_t sequence[9];
extern volatile  int16_t x ; 
extern volatile int16_t y ; 
extern volatile  uint8_t data_ready;
extern  volatile uint8_t buffer[6];


void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);	
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	
}

void Serial_SendArray(volatile uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)	
	{
		Serial_SendByte(Array[i]);	
	}
}



void USART2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;        
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;    
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); 

   
    USART_InitStruct.USART_BaudRate = 115200;     
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;  
    USART_InitStruct.USART_StopBits = USART_StopBits_1;       
    USART_InitStruct.USART_Parity = USART_Parity_No;         
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStruct);

    USART_Cmd(USART2, ENABLE);
		
		 USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

}


void UART5_Init(void) {
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;       
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;     
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 115200;     
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx;    
    USART_Init(UART5, &USART_InitStruct);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);    

    USART_Cmd(UART5, ENABLE);
		NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;        
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}



void USART2_IRQHandler(void) 
{
	static uint16_t rx_index = 0;
	static uint8_t begin=0;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
			{
				uint8_t data = USART_ReceiveData(USART2);
			        if (begin == 0) 
        {
            if (data == 0x2C) 
            {
                buffer[0] = data;  
                rx_index = 1;      
                begin = 1;         
            }
        } 
        else if(begin==1)
        {
            if (rx_index < 6) 
            {
                buffer[rx_index++] = data;
            }
            if (rx_index == 6) 
            {
                if (buffer[5] == 0x5B) 
                {
                    x = (int16_t)((buffer[1] << 8) | buffer[2]);
                    y = (int16_t)((buffer[3] << 8) | buffer[4]);
                    data_ready = 1;
                }
								rx_index=0;
					begin=0;  
      }
			 }
				USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		 }
				
		}

 
 
 
 
 
 
 
 
void UART5_IRQHandler(void) 
{
 
	static uint8_t se_quence;
	static uint8_t  start=0;
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) 
			{
       uint8_t data = USART_ReceiveData(UART5);
			if(start==0)
			{
				sequence[se_quence++]=data;
				if(sequence[0]==0x2c&&sequence[8]==0x5b)
			{ 
			
				Serial_SendArray(sequence,9);
				
			}
        }
			USART_ClearITPendingBit(UART5,USART_IT_RXNE);   
			}
		
			
}

