/*
	DS2482-800 1-Wire Driver for accessing Maxim 1-wire devices

	Author:	 Charith Fernanado http://www.inmojo.com charith@inmojo.com 
	License: Released under the Creative Commons Attribution Share-Alike 3.0 License. 
			 http://creativecommons.org/licenses/by-sa/3.0
	Target:  Microchip PIC, dsPIC with CCS C Compiler
	
	Author:	 Omnima Limited modified for use on STM32Expander.
	Adapted for use on almost any platform that implements i2c bitbanging (i2c-bb.c)
	Target:  STM32 family and GCC compiler
*/

#define DEBUG
#include "trace.h"

#include "ds2482.h"

#ifndef false
#define false 0
#endif

#ifndef true
#define true 1
#endif

uint8_t ds2482_reset(i2c_port_type *p)
{
	uint8_t status; 
	i2c_start(p);
	i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_WRITE);
	i2c_write(p, DS2482_CMD_DRST);
	i2c_start(p);
	i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_READ);
	status = i2c_read(p, I2C_READ_NACK);
	i2c_stop(p);
	
	// check for failure due to incorrect read back of status
#ifdef DEBUG
	if ((status & 0xf7) != 0x10) TRACE("DS2482 Reset failed\n");
#endif
	return ((status & 0xf7) == 0x10);
}	

uint8_t ds2482_detect(i2c_port_type *p)
{	
	if(!ds2482_reset(p))
	{
	    TRACE("DS2482 failed 1\n");
	    return false;
	}
		
	if(!ds2482_write_config(p, DS2482_CFG_APU))
	{
	    TRACE("DS2482 failed 2\n");
	    return false;
	}
		
	return true;
}

uint8_t ds2482_write_config(i2c_port_type *p, uint8_t config)
{
    uint8_t  read_config; 
    
    //TRACE1("--> DS2482 write config: %x\n", config);

    i2c_start(p);
    i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_WRITE);
    i2c_write(p, DS2482_CMD_WCFG);
    i2c_write(p, config | (~config << 4));
    i2c_start(p);
    i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_READ);
    read_config = i2c_read(p, I2C_READ_NACK);
    i2c_stop(p);

    // check for failure due to incorrect read back
    if (config != read_config)
    {
	TRACE("DS2482 write config failed\n");
	ds2482_reset(p);
	return false;
    }

    //TRACE("DS2482 write config ok\n");
    return true;
}

uint8_t ds2482_channel_select(i2c_port_type *p, uint8_t channel)
{
    uint8_t ch, ch_read, read_channel; 

    switch (channel){
	    default: case 0: ch = DS2482_CH_IO0; ch_read = DS2482_RCH_IO0; break;
	    case 1: ch = DS2482_CH_IO1; ch_read = DS2482_RCH_IO1; break;
	    case 2: ch = DS2482_CH_IO2; ch_read = DS2482_RCH_IO2; break;
	    case 3: ch = DS2482_CH_IO3; ch_read = DS2482_RCH_IO3; break;
	    case 4: ch = DS2482_CH_IO4; ch_read = DS2482_RCH_IO4; break;
	    case 5: ch = DS2482_CH_IO5; ch_read = DS2482_RCH_IO5; break;
	    case 6: ch = DS2482_CH_IO6; ch_read = DS2482_RCH_IO6; break;
	    case 7: ch = DS2482_CH_IO7; ch_read = DS2482_RCH_IO7; break;
    }

    i2c_start(p);
    i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_WRITE);
    i2c_write(p, DS2482_CMD_CHSL);
    i2c_write(p, ch);
    i2c_start(p);
    i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_READ);
    read_channel = i2c_read(p, I2C_READ_NACK);
    i2c_stop(p);

    return (read_channel == ch_read);
}

uint8_t oneWire_reset(i2c_port_type *p)
{
    uint8_t  status = 0; 
    uint8_t  poll_count = 0;
    
    i2c_start(p);
    i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_WRITE);
    i2c_write(p, DS2482_CMD_1WRS);
    delay_us(1);
    i2c_start(p);
    i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_READ);
    status = i2c_read(p, I2C_READ_ACK);

    do{
	    status = i2c_read(p, status & DS2482_STATUS_1WB);
    } while((status & DS2482_STATUS_1WB) && (poll_count++ < POLL_LIMIT));
    
    //write final NACK i2c command for confirmation 
    i2c_read(p, I2C_READ_NACK);

    i2c_stop(p);
    
    if(poll_count >= POLL_LIMIT){
	    TRACE("oneWire_reset ex limit\n");
	    ds2482_reset(p);
	    return false;
    } else
    {
	//TRACE1("oneWire_reset poll count was: %d\n", poll_count);
    }

    // check for presence detect
    if(status & DS2482_STATUS_PPD)
    {
	    return true;
    }
    else
    {
	    TRACE("oneWire_reset no ppd\n");
	    return false;
    }
}

void oneWire_WriteBit(i2c_port_type *p, uint8_t sendbit)
{
    oneWire_TouchBit(p, sendbit);
}

uint8_t oneWire_ReadBit(i2c_port_type *p)
{
    return oneWire_TouchBit(p, 0x01);
}

uint8_t oneWire_TouchBit(i2c_port_type *p, uint8_t sendbit)
{
    uint8_t status;
    int poll_count = 0;

    i2c_start(p);
    i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_WRITE);
    i2c_write(p, DS2482_CMD_1WSB);
    i2c_write(p, sendbit ? 0x80:0x00);
    i2c_start(p);
    i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_READ);
    status = i2c_read(p, I2C_READ_ACK);

    do
    {
	status = i2c_read(p, status & DS2482_STATUS_1WB);
    } while ((status & DS2482_STATUS_1WB) && (poll_count++ < POLL_LIMIT));

    i2c_read(p, I2C_READ_NACK);

    i2c_stop(p);

    if(poll_count >= POLL_LIMIT)
    {  
        TRACE("oneWire_TouchBit ex limit\n");
	ds2482_reset(p);
	return false;
    }

    // check for single bit result
    if(status & DS2482_STATUS_SBR)
	    return true;
    else
    {
	    //TRACE("oneWire_TouchBit DS2482_STATUS_SBR\n");
	    return false;
    }
}

void oneWire_WriteByte(i2c_port_type *p, uint8_t sendbyte)
{
    uint8_t status;
    int poll_count = 0;

    i2c_start(p);
    i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_WRITE);
    i2c_write(p, DS2482_CMD_1WWB);
    i2c_write(p, sendbyte);
    i2c_start(p);
    i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_READ);         // Read ScratchpadG_READ);
    status = i2c_read(p, I2C_READ_ACK);

    do
    {
	    status = i2c_read(p, status & DS2482_STATUS_1WB);
    } while ((status & DS2482_STATUS_1WB) && (poll_count++ < POLL_LIMIT));

    //write final NACK i2c command for confirmation 
    i2c_read(p, I2C_READ_NACK);
	    
    i2c_stop(p);

    if(poll_count >= POLL_LIMIT)
    {
            //TRACE("oneWire_WriteByte ex limit\n");
	    //ds2482_reset(p);
    }
}

uint8_t oneWire_ReadByte(i2c_port_type *p)
{
	uint8_t data, status;
	uint8_t poll_count = 0;
	
	i2c_start(p);
	i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_WRITE);
	i2c_write(p, DS2482_CMD_1WRB);
	i2c_start(p);
	i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_READ);
	status = i2c_read(p, I2C_READ_ACK);

	do{
		status = i2c_read(p, status & DS2482_STATUS_1WB);
	} while ((status & DS2482_STATUS_1WB) && (poll_count++ < POLL_LIMIT));
	
	//write final NACK i2c command for confirmation 
	i2c_read(p, I2C_READ_NACK);

	i2c_stop(p);
	
	if(poll_count >= POLL_LIMIT)
	{
	    TRACE("oneWire_ReadByte ex limit\n");
	    ds2482_reset(p);
	    return false; 
	}
	
	i2c_start(p);
	i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_WRITE);
	i2c_write(p, DS2482_CMD_SRP);
	i2c_write(p, DS2482_READPTR_RDR);
	i2c_start(p);
	i2c_write(p, DS2482_I2C_ADDR | I2C_FLAG_READ);
	data = i2c_read(p, I2C_READ_NACK);
	i2c_stop(p);
	
	//TRACE1("oneWire_ReadByte: %x", data);
	return data;
}

void oneWire_BlockTransfer(i2c_port_type *p, uint8_t *transfer_buffer, uint16_t length)
{
    uint16_t i;
    for(i = 0; i < length; i++)
	    transfer_buffer[i] = oneWire_TouchByte(p, transfer_buffer[i]);
}

uint8_t oneWire_TouchByte(i2c_port_type *p, uint8_t sendbyte)
{
	if(sendbyte == 0xff)
		return oneWire_ReadByte(p);
	else
	{
		oneWire_WriteByte(p, sendbyte);
		return sendbyte;
	}
}
