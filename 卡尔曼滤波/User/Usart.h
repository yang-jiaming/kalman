#ifndef _USART_H_
#define _USART_H_
#include "stm32f10x.h"
void Usart_Configuration(void);			//����Usart1 Tx->PA9 Rx->PA10
void USART1_Putc(uint8_t data);			//����Usart1����һ��8λ����
void USART1_Puts(uint8_t * buffer);			//����Uart4����һ���ַ���
void NVIC_Configuration(void);
void USART1_Send_Enter(void);//���ô���1����һ���з�

#endif
