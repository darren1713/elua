// Function definitions for STM32L4 I2C features based on interrupts 
// I2C3 (Master); I2C1 (Slave)
// In order to work on Nucleo it requires SB46 and SB52 to be shorted so both I2C interfaces
// are looped back  
// The master can initiate Write and read transaction. A single data byte is transmitted.
// On read I2C transaction the Slave returnes the last byte it has received from 
// Master (last write transaction)   

#ifndef __BLE_H__
#define __BLE_H__

//Platform functions ----------------------------------------------------------------

#define UART_BLE                0
#define PACKET_LEN(data_len)    ((data_len)+7)

#define IBEACON_DATA    39   //We have 38 bytes per iBeacon and 0 for termination
#define MAX_IBEACONS    52   //In our buffer we will have place for maximum that many iBeacons
#define IBEACON_BUFFER_SIZE	(IBEACON_DATA*MAX_IBEACONS)
#define BLE_TIMEOUT 	10000
#define BLE_CDC_PACKET_SIZE   50

enum states {WAITING_FOR_START, WAITING_FOR_LENGTH,  WAITING_FOR_DATACOMPLETE, CHECK_SUM};
int BLE_CDC_ENABLED;		//If set CDC thru the BLE module is enabled

void create_packet(unsigned char PacketID, unsigned char *Data, unsigned short DataLen);
int parse_packet(unsigned char *Packet, unsigned short PacketLen, unsigned char **PacketID, unsigned char **Data, unsigned short *DataLen);
#endif // __BLE_H__
