#ifndef HPLATFORM
#define HPLATFORM

#include "stm32f10x.h"
#include "type.h"

//#include "stm32l1xx_conf.h"   /* RAMFUNC */

/*static*/ __inline__ void delay_loop(volatile uint32_t __count)
{
    //for(;__count;__count--) asm volatile("nop");
    
    __asm__ volatile (
        "1: subs %1,%1,#1" "\n\t"
        "bne 1b"
        : "=r" (__count)
        : "0" (__count)
    );
}

//64Mhz clock delays?
//#define DELAY_1_CLK    asm volatile("nop");
//#define DELAY_9_CLK    delay_loop(3)
//#define DELAY_36_CLK   delay_loop(13)
//#define DELAY_50_US    delay_loop(134)
//#define DELAY_10_US    delay_loop(26)

//72MHZ clock delays
#define DELAY_1_CLK    delay_loop(2) //asm volatile("nop");
#define DELAY_9_CLK    delay_loop(3*4)
#define DELAY_36_CLK   delay_loop(13*4)
#define DELAY_1_US     delay_loop(10)
#define DELAY_10_US    delay_loop(26*4)
#define DELAY_50_US    delay_loop(134*4)
#define DELAY_1_MS     delay_loop(11989)

void delay_us(uint16_t msd);
void delay_ms(uint16_t msd);

#endif
