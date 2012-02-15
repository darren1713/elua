/*
 * bitbanding.h
 *
 *  Created on: Nov 26, 2010
 *      Author: Dean, Omnima Limited
 */

#ifndef BITBANDING_H_
#define BITBANDING_H_

#include "stddef.h"

//#define GPIOA_BB(b) (uint32_t*)((PERIPH_BB_BASE + ((GPIOA_BASE+offsetof(GPIO_TypeDef, ODR)-PERIPH_BASE)*32 + (b*4))))
//#define GPIOB_BB(b) (uint32_t*)((PERIPH_BB_BASE + ((GPIOB_BASE+offsetof(GPIO_TypeDef, ODR)-PERIPH_BASE)*32 + (b*4))))
//#define GPIOC_BB(b) (uint32_t*)((PERIPH_BB_BASE + ((GPIOC_BASE+offsetof(GPIO_TypeDef, ODR)-PERIPH_BASE)*32 + (b*4))))
//#define GPIOD_BB(b) (uint32_t*)((PERIPH_BB_BASE + ((GPIOD_BASE+offsetof(GPIO_TypeDef, ODR)-PERIPH_BASE)*32 + (b*4))))

//#define GPIO_BB_Addr(p,m,b) (uint32_t*)((PERIPH_BB_BASE + ((p+offsetof(GPIO_TypeDef, m)-PERIPH_BASE)*32 + (b*4))))

#define GPIO_BB_ODR(p,b) (uint32_t*)((PERIPH_BB_BASE + ((p+offsetof(GPIO_TypeDef, ODR)-PERIPH_BASE)*32 + (b*4))))
#define GPIO_BB_IDR(p,b) (uint32_t*)((PERIPH_BB_BASE + ((p+offsetof(GPIO_TypeDef, IDR)-PERIPH_BASE)*32 + (b*4))))
#define GPIO_BB_BSRR(p,b) (uint32_t*)((PERIPH_BB_BASE + ((p+offsetof(GPIO_TypeDef, BSRR)-PERIPH_BASE)*32 + (b*4))))
#define GPIO_BB_BRR(p,b) (uint32_t*)((PERIPH_BB_BASE + ((p+offsetof(GPIO_TypeDef, BRR)-PERIPH_BASE)*32 + (b*4))))

//#define GPIO_BB_SPISSI(p,b) (uint32_t*)((PERIPH_BB_BASE + ((p+offsetof(SPI_TypeDef, CR1)-PERIPH_BASE)*32 + (b*4))))
#define GPIO_BB_SPIRXNE(p) (uint32_t*)((PERIPH_BB_BASE + ((p+offsetof(SPI_TypeDef, SR)-PERIPH_BASE)*32 + (0*4))))

//#define GPIO_BB_Set(p,b) GPIO_BB_Addr(p,ODR,b)
//#define GPIO_BB_Set(p,b) GPIO_BB_Addr(p,IDR,b)

#endif /* BITBANDING_H_ */
