/*
 * uart.h
 *
 *  Created on: Jan 12, 2011
 *      Author: Omnima Limited
 */

#ifndef UART_H_
#define UART_H_

#include "platform-omniext.h"
#include "integer.h"

void UART_Init_Defaults();
int UART_SendChar(char ch, ULONG ud);
void UART_SendData(USART_TypeDef* USARTx, void*buf,UINT32 len);
void UART_SendString(USART_TypeDef* USARTx, const char*str);
void UART_printf_port(USART_TypeDef* USARTx, const char*str, ...);
void UART_printf(const char*str, ...);

#endif /* UART_H_ */
