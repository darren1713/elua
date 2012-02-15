/*
 * uart.c
 *
 *  Created on: Jan 12, 2011
 *      Author: Administrator
 */

#include "platform.h"
#include "sprintf.h"

void UART_Init_Defaults()
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Configure USART1 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_Init(USART3, &USART_InitStructure); 

	/* Enable the USART1 */
	USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART2, ENABLE);
	USART_Cmd(USART3, ENABLE);
}


int UART_SendChar(char ch, ULONG ud)
{
	ud; //Unused

	//Don't send 0's, the terminal output doesn't look neat if 0's are sent
	if (ch==0) return 1;
	
	// Loop until USART1 DR register is empty
	while(USART_GetFlagStatus((USART_TypeDef*)ud, USART_FLAG_TXE) == RESET)	{ ; }
	USART_SendData((USART_TypeDef*)ud, ch);

	return 1;
}

void UART_SendData(USART_TypeDef* USARTx, void*buf,UINT32 len)
{
	f_putb(UART_SendChar, buf, len, (ULONG)USARTx);
}

void UART_SendString(USART_TypeDef* USARTx, const char*str)
{
	f_puts(UART_SendChar, str, (ULONG)USARTx);
}

void UART_printf_port(USART_TypeDef* USARTx, const char*str, ...)
{
	va_list arp;

	va_start(arp, str);
	funcprintfArp(UART_SendChar, (ULONG)USARTx, str, arp);
	va_end(arp);
}

void UART_printf(const char*str, ...)
{
	va_list arp;

	va_start(arp, str);
	funcprintfArp(UART_SendChar, (ULONG)USART1, str, arp);
	va_end(arp);
}