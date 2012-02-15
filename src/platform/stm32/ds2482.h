#ifndef DS2482HEADER
#define DS2482HEADER

#include "i2c-bb.h"

// constants/macros/typdefs
#define DS2482_I2C_ADDR		0x32	//< Base I2C address of DS2482 devices
#define POLL_LIMIT		0xFF	// 0x30 is the minimum poll limit

//1-wire eeprom and silicon serial number commands
#define READ_DEVICE_ROM		0x33
#define SKIP_ROM			0xCC
#define WRITE_SCRATCHPAD	0x0F
#define READ_MEMORY			0xF0
#define COPY_SCRATCHPAD		0x55

// DS2482 command defines
#define DS2482_CMD_DRST		0xF0	//< DS2482 Device Reset
#define DS2482_CMD_SRP		0xE1	//< DS2482 Set Read Pointer
#define DS2482_CMD_WCFG		0xD2	//< DS2482 Write Configuration
#define DS2482_CMD_CHSL		0xC3	//< DS2482 Channel Select
#define DS2482_CMD_1WRS		0xB4	//< DS2482 1-Wire Reset
#define DS2482_CMD_1WWB		0xA5	//< DS2482 1-Wire Write Byte
#define DS2482_CMD_1WRB		0x96	//< DS2482 1-Wire Read Byte
#define DS2482_CMD_1WSB		0x87	//< DS2482 1-Wire Single Bit
#define DS2482_CMD_1WT		0x78	//< DS2482 1-Wire Triplet

// DS2482 status register bit defines
#define DS2482_STATUS_1WB	0x01	//< DS2482 Status 1-Wire Busy
#define DS2482_STATUS_PPD	0x02	//< DS2482 Status Presence Pulse Detect
#define DS2482_STATUS_SD	0x04	//< DS2482 Status Short Detected
#define DS2482_STATUS_LL	0x08	//< DS2482 Status 1-Wire Logic Level
#define DS2482_STATUS_RST	0x10	//< DS2482 Status Device Reset
#define DS2482_STATUS_SBR	0x20	//< DS2482 Status Single Bit Result
#define DS2482_STATUS_TSB	0x40	//< DS2482 Status Triplet Second Bit
#define DS2482_STATUS_DIR	0x80	//< DS2482 Status Branch Direction Taken

// DS2482 configuration register bit defines
#define DS2482_CFG_APU		0x01	//< DS2482 Config Active Pull-Up
#define DS2482_CFG_PPM		0x02	//< DS2482 Config Presence Pulse Masking
#define DS2482_CFG_SPU		0x04	//< DS2482 Config Strong Pull-Up
#define DS2482_CFG_1WS		0x08	//< DS2482 Config 1-Wire Speed

// DS2482 channel selection code for defines
#define DS2482_CH_IO0		0xF0	//< DS2482 Select Channel IO0
#define DS2482_CH_IO1		0xE1	//< DS2482 Select Channel IO1
#define DS2482_CH_IO2		0xD2	//< DS2482 Select Channel IO2
#define DS2482_CH_IO3		0xC3	//< DS2482 Select Channel IO3
#define DS2482_CH_IO4		0xB4	//< DS2482 Select Channel IO4
#define DS2482_CH_IO5		0xA5	//< DS2482 Select Channel IO5
#define DS2482_CH_IO6		0x96	//< DS2482 Select Channel IO6
#define DS2482_CH_IO7		0x87	//< DS2482 Select Channel IO7

// DS2482 channel selection read back code for defines
#define DS2482_RCH_IO0		0xB8	//< DS2482 Select Channel IO0
#define DS2482_RCH_IO1		0xB1	//< DS2482 Select Channel IO1
#define DS2482_RCH_IO2		0xAA	//< DS2482 Select Channel IO2
#define DS2482_RCH_IO3		0xA3	//< DS2482 Select Channel IO3
#define DS2482_RCH_IO4		0x9C	//< DS2482 Select Channel IO4
#define DS2482_RCH_IO5		0x95	//< DS2482 Select Channel IO5
#define DS2482_RCH_IO6		0x8E	//< DS2482 Select Channel IO6
#define DS2482_RCH_IO7		0x87	//< DS2482 Select Channel IO7

// DS2482 read pointer code defines
#define DS2482_READPTR_SR	0xF0	//< DS2482 Status Register
#define DS2482_READPTR_RDR	0xE1	//< DS2482 Read Data Register
#define DS2482_READPTR_CSR	0xD2	//< DS2482 Channel Selection Register
#define DS2482_READPTR_CR	0xC3	//< DS2482 Configuration Register

// DS2482 Funtion definition
uint8_t ds2482_reset(i2c_port_type *p);
uint8_t ds2482_detect(i2c_port_type *p);
uint8_t ds2482_write_config(i2c_port_type *p, uint8_t config);
uint8_t ds2482_channel_select(i2c_port_type *p, uint8_t channel); 
uint8_t oneWire_reset(i2c_port_type *p);
void oneWire_WriteBit(i2c_port_type *p, uint8_t sendbit);
uint8_t oneWire_ReadBit(i2c_port_type *p);
uint8_t oneWire_TouchBit(i2c_port_type *p, uint8_t sendbit);
void oneWire_WriteByte(i2c_port_type *p, uint8_t sendbyte);
uint8_t oneWire_ReadByte(i2c_port_type *p);
void oneWire_BlockTransfer(i2c_port_type *p, uint8_t *transfer_buffer, uint16_t length);
uint8_t oneWire_TouchByte(i2c_port_type *p, uint8_t sendbyte);

extern unsigned char ROM_NO[8];

#endif