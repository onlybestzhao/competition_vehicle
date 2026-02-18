#ifndef _HWT101_H
#define _HWT101_H
#include "sys.h"

	void Usart3_SendArray(uint8_t *array, uint16_t length);
void Usart3_SendString(char *String);
void Usart3_SendByte(uint8_t Byte);
void ParseAndPrintData(uint8_t *data, uint16_t length);
uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type) ;
		void anglezreset(void);
		void USART3_Init(void);
		void maxhz(void);
#endif
