#ifndef OWHEADER
#define OWHEADER

#include "i2c-bb.h"

void OWInit(i2c_port_type *p);
int OWSearch();
int OWFirst();
int OWNext();
int OWReset();
void OWWriteByte(unsigned char byte_value);
void OWWriteByteSPU(unsigned char byte_value);
void OWWriteBit(unsigned char bit_value);
unsigned char OWReadBit();
unsigned char OWReadByte();
void OWSelectDevice(const unsigned char *addr);
unsigned int ds18b20_temp(const unsigned char *addr);
void OWSearchCache();

unsigned int OWGetDeviceValue(const unsigned char *addr);
unsigned char OWSetDeviceValue(const unsigned char *addr, unsigned int val);

#endif