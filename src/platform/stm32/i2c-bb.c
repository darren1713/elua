//
// Omnima Limited (c) 2008-2011 All Rights Reserved
//

#include "platform.h"
#include "i2c-bb.h"

/////////////////////////////////////////////////////////
//Declare DEBUG to include tracing of code in this module
//#define DEBUG
#include "trace.h"
/////////////////////////////////////////////////////////

const uint8_t I2CSPEED=0;

// Initialise I2C peripherial
void init_I2C(i2c_port_type *p) 
{
    if (!p->init)
    {
	p->init=1;
	    
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_WriteBit(p->scl_port, p->scl_pin, Bit_SET);
	GPIO_WriteBit(p->sda_port, p->sda_pin, Bit_SET);
			
	GPIO_InitStructure.GPIO_Pin = p->scl_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(p->scl_port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = p->sda_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(p->sda_port, &GPIO_InitStructure);

	TRACE("<<- init_I2C\n");
    }
}

//Set SDA line to read and return the value on it.
//Since the line has a pull-up on it, this pulls the
//SDA line high if no one else is driving it
uint8_t READSDA(i2c_port_type *p) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = p->sda_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(p->sda_port, &GPIO_InitStructure);

    return GPIO_ReadInputDataBit(p->sda_port, p->sda_pin);
}

//Set SDA line to write and pull it low
void CLRSDA(i2c_port_type *p) 
{
    GPIO_WriteBit(p->sda_port, p->sda_pin, Bit_RESET);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = p->sda_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(p->sda_port, &GPIO_InitStructure);
}

//Set SCL line to read and return the value on it.
//Since the line has a pull-up on it, this pulls the
//SCL line high if no one else is driving it
uint8_t READSCL(i2c_port_type *p) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = p->scl_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(p->scl_port, &GPIO_InitStructure);

    return GPIO_ReadInputDataBit(p->scl_port, p->scl_pin);
}

//Set SCL line to write and pull it low
void CLRSCL(i2c_port_type *p) 
{
    GPIO_WriteBit(p->scl_port, p->scl_pin, Bit_RESET);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = p->scl_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(p->scl_port, &GPIO_InitStructure);
}

void I2CDELAY(volatile uint16_t clock_period) 
{
    volatile uint16_t i;
    for(i=0;i<clock_period*100;i++)
    {
	  //l++;
    }
}

void ARBITRATION_LOST(void) 
{
  //Can't happen, only slaves on the bus
}

//==== Bit-level functions
//Translated from Wikipedia pseudocode - http://en.wikipedia.org/wiki/I2C

uint8_t read_bit(i2c_port_type *p) 
{
   uint8_t bit;

   // Let the slave drive data
   READSDA(p);
   I2CDELAY(I2CSPEED/2);
   /* Clock stretching */
   while (READSCL(p) == 0);
   /* SCL is high, now data is valid */
   bit = READSDA(p);
   I2CDELAY(I2CSPEED/2);
   CLRSCL(p);
   return bit;
}

void write_bit(i2c_port_type *p, uint8_t bit) 
{
  //Put the bit on SDA by either letting it rise or pulling it down
   if (bit) {
      READSDA(p);
   } else {
      CLRSDA(p);
   }
   I2CDELAY(I2CSPEED/2);
   /* Clock stretching - Let SCL rise and wait for slave to let it rise */
   while (READSCL(p) == 0);
   /* SCL is high, now data is being read */
   /* If SDA is high, check that nobody else is driving SDA */
   if (bit) {
    if (READSDA(p) == 0) { //Oops, someone else pulled SDA down
        ARBITRATION_LOST();
      }
   }
   I2CDELAY(I2CSPEED/2);
   CLRSCL(p);
}

void i2c_start(i2c_port_type *p) 
{
   if (p->start) {
      /* Let SDA rise */
      READSDA(p);
      I2CDELAY(I2CSPEED/2);
      /* Clock stretching */
      while (READSCL(p) == 0);
   }
   if (READSDA(p) == 0) {
      ARBITRATION_LOST();
   }
   /* SCL is high, we waited for slave to raise it, so pull SDA down */
   CLRSDA(p);
   I2CDELAY(I2CSPEED/2);
  // Now pull SCL down
   CLRSCL(p);
   p->start = 1;
}

void i2c_stop(i2c_port_type *p) 
{
   /* Pull SDA down */
   CLRSDA(p);
   I2CDELAY(I2CSPEED/2);
   /* Clock stretching - wait for slave to raise it*/
   while (READSCL(p) == 0);
   /* SCL is high, set SDA from 0 to 1 */
   if (READSDA(p) == 0) {
      ARBITRATION_LOST();
   }
   I2CDELAY(I2CSPEED/2);
   p->start = 0;
}

uint8_t i2c_write(i2c_port_type *p, uint8_t byte)
{
   uint8_t bit;
   
   for (bit = 0; bit < 8; bit++) 
   {
      write_bit(p, byte & 0x80);
      byte <<= 1;
   }  
   return read_bit(p);
}

uint8_t i2c_read(i2c_port_type *p, uint8_t nak)
{
   uint8_t byte = 0;
   uint8_t bit;

   for(bit = 0; bit < 8; bit++) 
   {
      byte <<= 1;      
      byte |= read_bit(p);
   }
   write_bit(p, nak);
   return byte;  
}

//==== Byte-level protocol
//Translated from Wikipedia pseudocode - http://en.wikipedia.org/wiki/I2C

uint8_t i2c_tx(i2c_port_type *p, uint8_t send_start, uint8_t send_stop, uint8_t byte)
{
   uint8_t bit;
   uint8_t nack;
   if (send_start) 
   {
      i2c_start(p);
   }
   for (bit = 0; bit < 8; bit++) 
   {
      write_bit(p, byte & 0x80);
      byte <<= 1;
   }
   nack = read_bit(p);
   if (send_stop) 
   {
      i2c_stop(p);
   }
   return nack;
}

uint8_t i2c_rx(i2c_port_type *p, uint8_t nak, uint8_t send_stop)
{
   uint8_t byte = 0;
   uint8_t bit;

   for(bit = 0; bit < 8; bit++) 
   {
      byte <<= 1;      
      byte |= read_bit(p);
   }
   write_bit(p, nak);
   if (send_stop) 
   {
      i2c_stop(p);
   }
   return byte;
}
