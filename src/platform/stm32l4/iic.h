// Function definitions for STM32L4 I2C features based on interrupts 
// I2C3 (Master); I2C1 (Slave)
// In order to work on Nucleo it requires SB46 and SB52 to be shorted so both I2C interfaces
// are looped back  
// The master can initiate Write and read transaction. A single data byte is transmitted.
// On read I2C transaction the Slave returnes the last byte it has received from 
// Master (last write transaction)   

#ifndef __I2C_H__
#define __I2C_H__

//Platform functions ----------------------------------------------------------------
volatile unsigned char master_data_to_send; //byte for transmission I2C3 -> I2C1 
volatile unsigned char slave_received_data; //byte received at I2C1 from I2C3  
volatile unsigned char master_received_data; //byte I2C3 read from I2C1
volatile int I2C_Master_read_write = 0; //1 master read transfer, 0 master write data to Slave
volatile int I2C_Master_read_complete=0; //Master -> Slave I2C transaction complete
#define SLAVE_OWN_ADDRESS                       0x5A
/* Timing register value is computed with the STM32CubeMX Tool,
  * Fast Mode @400kHz with I2CCLK = 80 MHz,
  * rise time = 100ns, fall time = 10ns
  * Timing Value = (uint32_t)0x00F02B86
  */
#define I2C_TIMING               0x00F02B86

void stm32l4_Configure_I2C_Master(void);
void stm32l4_Configure_I2C_Slave(void);
void stm32l4_i2c_read(void);
void stm32l4_i2c_write(unsigned char);
unsigned char stm32l4_i2c_read_result(void);

#endif
