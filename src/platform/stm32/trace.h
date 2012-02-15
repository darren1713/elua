#ifndef TRACEH

#include "uart.h"

#ifdef DEBUG
#define TRACE(x) UART_printf(x);
#define TRACE1(x,p1) UART_printf(x,p1);
#define TRACE2(x,p1,p2) UART_printf(x,p1,p2);
#define TRACE3(x,p1,p2,p3) UART_printf(x,p1,p2,p3);
#define TRACE4(x,p1,p2,p3,p4) UART_printf(x,p1,p2,p3,p4);
#define TRACE5(x,p1,p2,p3,p4,p5) UART_printf(x,p1,p2,p3,p4,p5);
#define TRACE6(x,p1,p2,p3,p4,p5,p6) UART_printf(x,p1,p2,p3,p4,p5,p6);
#else
#define TRACE(x)
#define TRACE1(x,p1)
#define TRACE2(x,p1,p2)
#define TRACE3(x,p1,p2,p3)
#define TRACE4(x,p1,p2,p3,p4)
#define TRACE5(x,p1,p2,p3,p4,p5)
#define TRACE6(x,p1,p2,p3,p4,p6)
#endif

#endif
