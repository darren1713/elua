// STM32L4 sleep based on stop 2. Wake up based on RTC and some other 

#ifndef __PMU_H__
#define __PMU_H__

#include "stm32l4xx_ll_rtc.h"

void    stm32l4_SystemClock_Config(void);
void    stm32l4_Configure_RTC(void);
void    stm32l4_EnterStop2Mode(int rtc_timeout);
void 	stm32l4_UserButton_Init(void);
void    RTC_WKUP_IRQHandler(void);
#endif
