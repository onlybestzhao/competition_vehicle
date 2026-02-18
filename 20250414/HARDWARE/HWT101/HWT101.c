#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "Delay.h"
#include "stdio.h"
#include "stdarg.h"

#include "stdint.h"

  extern volatile float global_angle; 
extern  uint8_t new_data_received;
//extern  volatile float velocity;
extern  volatile float velocitz;
extern uint8_t received_data_packet[11];
	
extern uint8_t rx_buffer[11];


volatile uint8_t reset_z_axis[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
volatile uint8_t HZ[]={0xFF,0XAA,0X03,0X0D,0X00};



void Usart3_SendByte(uint8_t Byte) {
    USART_SendData(USART3, Byte); 
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); 
}
 
void Usart3_SendString(char *String) {
    uint8_t i = 0;
    while (String[i]) { 
        Usart3_SendByte(String[i]); 
        i++;
    }
}


void Usart3_SendArray(volatile uint8_t *array, uint16_t length) 
	{
    for (uint16_t i = 0; i < length; i++) {
        Usart3_SendByte(array[i]);
    }	
			
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

void USART3_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;       
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;     
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
		 GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);

   
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;    
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStruct);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  
		
		

    
    NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

   
    USART_Cmd(USART3, ENABLE);
//		 Usart3_SendArray(unlock_register, sizeof(unlock_register));
//    Usart3_SendArray(set_output_200Hz, sizeof(set_output_200Hz));
//    Usart3_SendArray(set_baudrate_115200, sizeof(set_baudrate_115200));
//    Usart3_SendArray(save_settings, sizeof(save_settings));
//    Usart3_SendArray(restart_device, sizeof(restart_device));
	}



void anglezreset(void)
{
	
 Usart3_SendArray(reset_z_axis, sizeof(reset_z_axis));
	
	}
void maxhz(void)
{
	 Usart3_SendArray(HZ, sizeof(HZ));
}

uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type) 
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}


void ParseAndPrintData(uint8_t *data, uint16_t length)
	{
    if (length == 11) {
        uint8_t checksum =CalculateChecksum(data, length - 1, data[1]);
        if (checksum != data[length - 1]) {
          // Usart3_Printf("Checksum error\r\n");
            return;
        }
 
        if (data[0] == 0x55 && data[1] == 0x53)
					{
            uint8_t yaw_l = data[6];
            uint8_t yaw_h = data[7];
            int16_t yaw = (int16_t)((yaw_h << 8) | yaw_l);
            float angle = ((float)yaw / 32768.0f) * 180.0f;
            global_angle = angle;
						
            new_data_received = 1;
				}
							 else if (data[0] == 0x55 && data[1] == 0x52) {
//            uint8_t wy_l = data[4];
//            uint8_t wy_h = data[5];
//            int16_t wy = (int16_t)((wy_h << 8) | wy_l);
//            velocity = ((float)wy / 32768.0f) * 2000.0f;
 
            uint8_t wz_l = data[6];
            uint8_t wz_h = data[7];
            int16_t wz = (int16_t)((wz_h << 8) | wz_l);
            velocitz = ((float)wz / 32768.0f) * 2000.0f;
						
            new_data_received = 1;

			}
		}
	}
		
void USART3_IRQHandler(void) 
{
	 //static uint8_t rx_buffer[11];

	 static uint8_t  rx_index;
	 static uint8_t  start=0;
	
	
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
			{
				uint8_t data=USART_ReceiveData (USART3 );
//				data=1l;				
			//Usart2_SendArray(data,11);
				if(start==0)
				{
					if(data==0x55)
					{
						rx_buffer[0]=data ;
						rx_index =1;
						start =1;
					}
				}	
				else if(start ==1)
				{
					rx_buffer[rx_index++]=data;
					if(rx_index==2&&rx_buffer[1]!=0x53&&rx_buffer[1]!=0x52)
					{
						if(rx_buffer[1]==0x55)
						{rx_index =1;
						}
						else {start=0;
						rx_index=0;
						}
					}
					else if(rx_index ==11)	
					{
						//anglezreset();
//						for(int g = 0; g < 4; g++)
//						{
//						Usart2_SendArray(rx_buffer,11);
//						}
 					 ParseAndPrintData(rx_buffer, 11);
                rx_index = 0;
                start= 0;
					}
				}
				USART_ClearITPendingBit(USART3, USART_IT_RXNE);

    }
}
	
	



