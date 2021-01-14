// This header lists all interrupts defined for this platform

#ifndef __PLATFORM_INTS_H__
#define __PLATFORM_INTS_H__

#include "elua_int.h"

#define INT_UART_RX          ELUA_INT_FIRST_ID	         //1
#define INT_UART_BUF_FULL    ( ELUA_INT_FIRST_ID + 1 )   //2
#define INT_UART_BUF_MATCH   ( ELUA_INT_FIRST_ID + 2 )   //3
#define INT_IRIDIUM_SIGNAL   ( ELUA_INT_FIRST_ID + 3 )   //4
#define INT_IRIDIUM_TX_OK    ( ELUA_INT_FIRST_ID + 4 )   //5
#define INT_IRIDIUM_TX_FAIL  ( ELUA_INT_FIRST_ID + 5 )   //6
#define INT_IRIDIUM_TIMEOUT  ( ELUA_INT_FIRST_ID + 6 )   //7
#define INT_GPS_VALID        ( ELUA_INT_FIRST_ID + 7 )   //8
#define INT_GPS_TIMEOUT      ( ELUA_INT_FIRST_ID + 8 )   //9
#define INT_BOOT             ( ELUA_INT_FIRST_ID + 9 )   //10
#define INT_CONTENTION       ( ELUA_INT_FIRST_ID + 10 )  //11
#define INT_TICKSECOND       ( ELUA_INT_FIRST_ID + 11 )  //12
#define INT_GSM_SIGNAL       ( ELUA_INT_FIRST_ID + 12 )  //13
#define INT_GSM_TX_OK        ( ELUA_INT_FIRST_ID + 13 )  //14
#define INT_GSM_TX_FAIL      ( ELUA_INT_FIRST_ID + 14 )  //15
#define INT_GSM_TIMEOUT      ( ELUA_INT_FIRST_ID + 15 )  //16
#define INT_SEN_ACCEL        ( ELUA_INT_FIRST_ID + 16 )  //17
#define INT_ELUA_LAST        INT_SEN_ACCEL

#endif // #ifndef __PLATFORM_INTS_H__
