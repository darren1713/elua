
#include <stdarg.h>

//---------


#ifndef SPRINTFH
#define SPRINTFH


#include "platform-omniext.h"
#include "integer.h"

//Function to output chars, returns 1 to continue, 0 to stop
typedef int (*funcPutChar)(char ch, ULONG ud);


//Streams string pointed by pStr to fPutChar
int f_puts(funcPutChar fPutChar, const char *pStr, ULONG ud);


//Streams bytes stored in pBuf to the length of len
int f_putb(funcPutChar fPutByte, const unsigned char *pBuf, ULONG len, ULONG ud);


#define NOTEOFPRINTF -1
#define EOFPRINTF 0

int funcprintfArp (
	funcPutChar fPutChar,
	ULONG ud,
	const char* str,	// Pointer to the format string
	va_list arp		    // Optional arguments...
);

int funcprintf (
	funcPutChar fPutChar,
	ULONG ud,
	const char* str,	// Pointer to the format string
	...					// Optional arguments... 
);


typedef struct 
{
	char *pBuf;
	UINT32 bufSize;
	UINT32 pos;
} printToStringType;

int osprintf(char* strBuf, UINT32 bufSize, const char*format, ...);

#endif
