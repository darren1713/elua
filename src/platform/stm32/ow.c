/*
	1-wire bus enumeration and access code.
	
	Portions of code as supplied by Maxim-IC.
	
	License: Released under the Creative Commons Attribution Share-Alike 3.0 License. 
			 http://creativecommons.org/licenses/by-sa/3.0	
			 
	Distribution: Must preserve this header and the list of authors who have contributed to this module
			 
	Author:	 Omnima Limited modified and extended for use on STM32Expander.
	Adapted for use on almost any platform that implements i2c bitbanging (i2c-bb.c)
	Target:  STM32 family and GCC compiler
*/

#include "ow.h"
#include "ds2482.h"
#include <string.h>

//#define DEBUG
#include "trace.h"

#ifndef false
#define false 0
#endif

#ifndef true
#define true 1
#endif

// method declarations
int  OWVerify();
void OWTargetSetup(unsigned char family_code);
void OWFamilySkipSetup();
unsigned char docrc8(unsigned char value);

// global search state
unsigned char ROM_NO[8];
int LastDiscrepancy;
int LastFamilyDiscrepancy;
int LastDeviceFlag;
unsigned char crc8;

i2c_port_type g_p;

void OWInit(i2c_port_type *p)
{
    	memcpy(&g_p, p, sizeof(i2c_port_type));
}

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire bus
// Return true  : device found, ROM number in ROM_NO buffer
//        false : no device present
//
int OWFirst()
{  
   // reset the search state
   LastDiscrepancy = 0;
   LastDeviceFlag = false;
   LastFamilyDiscrepancy = 0;

   return OWSearch();
}

//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire bus
// Return true  : device found, ROM number in ROM_NO buffer
//        false : device not found, end of search
//
int OWNext()
{
   // leave the search state alone
   return OWSearch();
}

//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return true  : device found, ROM number in ROM_NO buffer
//        false : device not found, end of search
//
int OWSearch()
{
   int id_bit_number;
   int last_zero, rom_byte_number, search_result;
   int id_bit, cmp_id_bit;
   unsigned char rom_byte_mask, search_direction;
   
   TRACE("OWSearch\n");

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = 0;
   crc8 = 0;

   // if the last call was not the last one
   if (!LastDeviceFlag)
   {
      // 1-Wire reset
      if (!OWReset())
      {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
	 TRACE("OWReset failed\n");
         return false;
      }

      // issue the search command 
      OWWriteByte(0xF0);  

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = OWReadBit();
         cmp_id_bit = OWReadBit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1))
            break;
         else
         {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit)
               search_direction = id_bit;  // bit write value for search
            else
            {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy)
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               else
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);

               // if 0 was picked then record its position in LastZero
               if (search_direction == 0)
               {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            OWWriteBit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0)
            {
                docrc8(ROM_NO[rom_byte_number]);  // accumulate the CRC
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!((id_bit_number < 65) || (crc8 != 0)))
      {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0)
            LastDeviceFlag = true;
         
         search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0])
   {
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   }

   return search_result;
}

/*
//
//Alternative search algorithm
//
int path,next,pos;                          // decision markers
int count;                                  // bit count
int bit,chk;                                // bit values

path=0;                                     // initial path to follow
do{                                         // each ROM search pass
(initialize the bus with a reset pulse)
(issue the 'ROM search' command)
    next=0;                                 // next path to follow
    pos=1;                                  // path bit pointer
    count=0;                                // count the bits
    do{                                     // each bit of the ROM value (read two bits, 'bit' and 'chk', from the 1-wire bus)
        if(!bit && !chk){                   // collision, both are zero
            if(pos&path) bit=1;             // if we've been here before
            else next=(path&(pos-1))|pos;   // else, new branch for next
            pos<<=1;
         }
(write 'bit' to the 1-wire bus)
(save this bit as part of the current ROM value)
         count++;
    }while(count<64);
(output the just-completed ROM value)
    path=next;
}while(path);
*/

//--------------------------------------------------------------------------
// Verify the device with the ROM number in ROM_NO buffer is present.
// Return true  : device verified present
//        false : device not present
//
int OWVerify()
{
   unsigned char rom_backup[8];
   int i,rslt,ld_backup,ldf_backup,lfd_backup;

   // keep a backup copy of the current state
   for (i = 0; i < 8; i++)
      rom_backup[i] = ROM_NO[i];
   ld_backup = LastDiscrepancy;
   ldf_backup = LastDeviceFlag;
   lfd_backup = LastFamilyDiscrepancy;

   // set search to find the same device
   LastDiscrepancy = 64;
   LastDeviceFlag = false;

   if (OWSearch())
   {
      // check if same device found
      rslt = true;
      for (i = 0; i < 8; i++)
      {
         if (rom_backup[i] != ROM_NO[i])
         {
            rslt = false;
            break;
         }
      }
   }
   else
     rslt = false;

   // restore the search state 
   for (i = 0; i < 8; i++)
      ROM_NO[i] = rom_backup[i];
   LastDiscrepancy = ld_backup;
   LastDeviceFlag = ldf_backup;
   LastFamilyDiscrepancy = lfd_backup;

   // return the result of the verify
   return rslt;
}

//--------------------------------------------------------------------------
// Setup the search to find the device type 'family_code' on the next call
// to OWNext() if it is present.
//
void OWTargetSetup(unsigned char family_code)
{
   int i;

   // set the search state to find SearchFamily type devices
   ROM_NO[0] = family_code;
   for (i = 1; i < 8; i++)
      ROM_NO[i] = 0;
   LastDiscrepancy = 64;
   LastFamilyDiscrepancy = 0;
   LastDeviceFlag = false;
}

//--------------------------------------------------------------------------
// Setup the search to skip the current device type on the next call
// to OWNext().
//
void OWFamilySkipSetup()
{
   // set the Last discrepancy to last family discrepancy
   LastDiscrepancy = LastFamilyDiscrepancy;
   LastFamilyDiscrepancy = 0;

   // check for end of list
   if (LastDiscrepancy == 0)
      LastDeviceFlag = true;
}

//--------------------------------------------------------------------------
// 1-Wire Functions to be implemented for a particular platform
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Reset the 1-Wire bus and return the presence of any device
// Return true  : device present
//        false : no device present
//
int OWReset()
{
   return (oneWire_reset(&g_p) == 1);
}

//--------------------------------------------------------------------------
// Send 8 bits of data to the 1-Wire bus
//
void OWWriteByte(unsigned char byte_value)
{
   oneWire_WriteByte(&g_p,byte_value);
}

void OWWriteByteSPU(unsigned char byte_value)
//Write byte with strong pull-up (for the devices that use parasitic power to run)
{
   ds2482_write_config(&g_p, DS2482_CFG_SPU);
   oneWire_WriteByte(&g_p,byte_value);
}

//--------------------------------------------------------------------------
// Send 1 bit of data to teh 1-Wire bus
//
void OWWriteBit(unsigned char bit_value)
{
   oneWire_WriteBit(&g_p, bit_value);
}

//--------------------------------------------------------------------------
// Read 1 bit of data from the 1-Wire bus 
// Return 1 : bit read is 1
//        0 : bit read is 0
//
unsigned char OWReadBit()
{
   return (unsigned char)oneWire_ReadBit(&g_p);

}

unsigned char OWReadByte()
{
  return oneWire_ReadByte(&g_p);
}

// TEST BUILD
static unsigned char dscrc_table[] = {
        0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
      157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
       35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
      190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
       70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
      219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
      101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
      248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
      140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
       17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
      175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
       50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
      202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
       87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
      233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
      116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

//--------------------------------------------------------------------------
// Calculate the CRC8 of the byte value provided with the current 
// global 'crc8' value. 
// Returns current global crc8 value
//
unsigned char getcrc8(unsigned char *pcrc, unsigned char value)
{
    return dscrc_table[*pcrc ^ value];
}

unsigned char docrc8(unsigned char value)
{
   // See Application Note 27
   crc8 = getcrc8(&crc8, value);
   return crc8;
}

void OWSelectDevice(const unsigned char *addr)
{
    int i;
    
    if (!OWReset()) return;
    OWWriteByte(0x55);
    //OWWriteByte(0x69);
    for(i=0;i<8;i++) OWWriteByte(addr[i]);
}

unsigned int ds18b20_temp(const unsigned char *addr)
{
    unsigned char readBuf[9];
    unsigned char crc=0;
    uint16_t temp;
    int i;
    
    OWReset();
    OWSelectDevice(addr);
    
    OWWriteByte(0x4E); //Set resolution i.e. write to scratchpad
    OWWriteByte(0x4B); //TH reg
    OWWriteByte(0x46); //TL reg
    OWWriteByte(0x3F); //10-bit resolution
    OWReset();
    OWSelectDevice(addr);
    
    OWWriteByteSPU(0x44);
    delay_ms(300); //200mS for 10bit, 800mS for 12bit resolution
    OWReset();
    OWSelectDevice(addr);
    OWWriteByte(0xBE);
    
    //TRACE("Read:");
    for(i=0;i<9;i++)
    {
	readBuf[i]=OWReadByte();
	if (i<8) crc=getcrc8(&crc, readBuf[i]);
	//TRACE1("%02x", readBuf[i]);
    }
    //TRACE("\n");
    
    if (crc!=readBuf[8])
    {
	TRACE2("temp crc nok: %x!=%x\n", crc, readBuf[9]);
	return 0xFFFF;
    }
    
    temp=(uint16_t)((float)((readBuf[1]<<8)+readBuf[0])*6.25); //10bit=0.125°C=12.5, and 12bit=0.0625°C=6.25,
    return temp;
}

unsigned int ds2408_read_pio(const unsigned char *addr, unsigned char reg)
{
    unsigned char val; //,crc,ccrc=0,valff
    
    OWReset();
    OWSelectDevice(addr);
    //OWWriteByte(0xCC);
    OWWriteByte(0xF0);
    OWWriteByte(reg);
    OWWriteByte(0x00);
    val=OWReadByte();
    
    //TODO: verify checksum
    
    //crc=OWReadByte();
    //ccrc=getcrc8(&ccrc, OWReadByte());
    //valff=OWReadByte();
    //if (crc!=ccrc || valff!=0xff)
    //{
    //	TRACE4("ds2408_read_pio crcerr val: %x, crc: %x, ccrc: %x, valff: %x", val, crc, ccrc, valff);
    //	val=0xFFFF;
    //}
    
    OWReset();
    return val;
}

unsigned char ds2408_set_pio(const unsigned char *addr, unsigned char byte)
{
    unsigned char recv;
    
    OWReset();
    OWSelectDevice(addr);
    OWWriteByte(0x5A);  
    //TRACE1("ds2408 write %x\n", byte);
    OWWriteByte(byte);
    //TRACE1("ds2408 write inverted %x\n", (~byte)&0xff);
    OWWriteByte((~byte)&0xff);
    if ((recv=OWReadByte())!=0xAA)
    {
      TRACE2("ds2408 sent %x but received ack %x\n", byte, recv);
      return 0; //Must receive AA as confirmation of the byte being received
    }
    if ((recv=OWReadByte())!=byte)
    {
      TRACE2("ds2408 sent %x but read pio %x\n", byte, recv);
      return 0;
    }
    //TRACE2("ds2408 %s set to %x\n", addr, byte);
    OWReset();
    return 1;
}

#define FID_DS18B20 0x28
#define FID_DS2408  0x29

unsigned int OWGetDeviceValue(const unsigned char *addr)
{
    unsigned int val=0;
    
    switch (addr[0])
    {
      case FID_DS18B20:
	    val=ds18b20_temp(addr);
	    TRACE1(" val: %d\n", val);
	break;
      case FID_DS2408:
	    val=ds2408_read_pio(addr, 0x88);
	    TRACE1(" val: %d\n", val);
	break;
    }
    
    return val;
}

unsigned char OWSetDeviceValue(const unsigned char *addr, unsigned int val)
{
    unsigned char res=0;
        
    switch (addr[0])
    {
      case FID_DS18B20:
	    TRACE1("ds18b20 can't set value: %x\n", val);
	break;
      case FID_DS2408:
	    res=ds2408_set_pio(addr, val&0xff);
	    TRACE2("ds2408 set to: %02x res %d\n", val, res);
	break;
    }
    return res;
}
