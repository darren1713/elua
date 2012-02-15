#ifndef __TYPE_H__
#define __TYPE_H__

#include "stm32f10x.h"

#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned long  DWORD;
typedef unsigned int   BOOL;

typedef unsigned long long u64;
typedef signed long long s64;

typedef uint8_t UINT8;
typedef int8_t INT8;
typedef uint16_t UINT16;
typedef int16_t INT16;
typedef uint32_t UINT32;
typedef int32_t INT32;
//typedef unsigned long ULONG;
//typedef uint8_t UCHAR;

typedef uint8_t uint8;
typedef int8_t int8;
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint32_t uint32;
typedef int32_t int32;

#define HIBYTE(p) &((uint8_t*)p)[1]
#define LOBYTE(p) &((uint8_t*)p)[0]

//#define FALSE 0
//#define TRUE 1

#ifndef NULL
#define NULL 	0
#endif

#endif
