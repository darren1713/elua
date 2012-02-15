
#include "sprintf.h"
#include <stdarg.h>

int f_puts(funcPutChar fPutChar, const char *pStr, ULONG ud)
{
	int cnt=0,ret=0;
	
	while(*pStr!='\0')
	{
		ret = fPutChar(*pStr++, ud);
		if (ret==EOFPRINTF) break;
		cnt += ret;
	}
	return cnt;
}

int f_putb(funcPutChar fPutByte, const unsigned char *pBuf, ULONG ud, ULONG len)
{
	int cnt,ret=0;
	
	for(cnt=0;cnt<len;cnt++)
	{
		ret = fPutByte(pBuf[cnt], len);
		if (ret==EOFPRINTF) break;
	}
	return cnt;	
}

int funcprintfArp (
	funcPutChar fPutChar,
	ULONG ud,
	const char* str,	// Pointer to the format string
	va_list arp		// Optional arguments...
)
{
	UCHAR c, f, r;
	ULONG val;
	char s[16];
	int i, w, res , cc;

	for (cc = res = NOTEOFPRINTF; cc != EOFPRINTF; res += cc) {
		c = *str++;
		if (c == 0) break;			// End of string
		if (c != '%') {				// Non escape cahracter
			cc = fPutChar(c, ud);
			if (cc != EOFPRINTF) cc = 1;
			continue;
		}
		w = f = 0;
		c = *str++;
		if (c == '0') {				// Flag: '0' padding
			f = 1; c = *str++;
		}
		while (c >= '0' && c <= '9') {	// Precision
			w = w * 10 + (c - '0');
			c = *str++;
		}
		if (c == 'l') {				// Prefix: Size is long int
			f |= 2; c = *str++;
		}
		if (c == 's') {				// Type is string
			cc = f_puts(fPutChar, va_arg(arp, char*), ud);
			continue;
		}
		if (c == 'c') {				// Type is character
			cc = fPutChar(va_arg(arp, int), ud);
			if (cc != EOFPRINTF) cc = 1;
			continue;
		}
		r = 0;
		if (c == 'd') r = 10;		// Type is signed decimal
		if (c == 'u') r = 10;		// Type is unsigned decimal
		if (c == 'X' || c == 'x') r = 16;		// Type is unsigned hexdecimal
		if (r == 0) break;			// Unknown type
		if (f & 2) {				// Get the value
			val = (ULONG)va_arg(arp, long);
		} else {
			val = (c == 'd') ? (ULONG)(long)va_arg(arp, int) : (ULONG)va_arg(arp, unsigned int);
		}
		// Put numeral string
		if (c == 'd') {
			if (val & 0x80000000) {
				val = 0 - val;
				f |= 4;
			}
		}
		i = sizeof(s) - 1; s[i] = 0;
		do {
			c = (UCHAR)(val % r + '0');
			if (c > '9') c += 7;
			s[--i] = c;
			val /= r;
		} while (i && val);
		if (i && (f & 4)) s[--i] = '-';
		w = sizeof(s) - 1 - w;
		while (i && i > w) s[--i] = (f & 1) ? '0' : ' ';
		cc = f_puts(fPutChar, &s[i], ud);
	}

	cc = fPutChar('\0', ud);
	if (cc != EOFPRINTF) cc = 1;

	return (cc == EOFPRINTF) ? cc : res;
}

int funcprintf (
	funcPutChar fPutChar,
	ULONG ud,
	const char* str,	//Pointer to the format string
	...			// Optional arguments...
)
{
	int retVal;

	va_list arp;
	va_start(arp, str);
	retVal=funcprintfArp(fPutChar, ud, str, arp);
	va_end(arp);
	return retVal;
}

int printToStringBuf(char ch, ULONG ud)
{
	printToStringType *pToS=(printToStringType *)ud;
	if (pToS->pos<pToS->bufSize)
	{
		pToS->pBuf[pToS->pos++]=ch;
	}
	return 1;
}

int osprintf(char* strBuf, UINT32 bufSize, const char*format, ...)
{
	int retVal;
	printToStringType pts;
	va_list arp;
	
	pts.pBuf=strBuf;
	pts.bufSize=bufSize;
	pts.pos=0;

	va_start(arp, format);
	retVal=funcprintfArp(printToStringBuf, (ULONG)&pts, format, arp);
	va_end(arp);
	return retVal;
}
