#ifndef I2C_BB_HEADER
#define I2C_BB_HEADER

#include "platform.h"

#define I2C_FLAG_WRITE 0
#define I2C_FLAG_READ  1
#define I2C_READ_NACK  1
#define I2C_READ_ACK   0

typedef struct 
{
    uint8_t init;
    uint8_t start;

    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
} i2c_port_type;

void init_I2C(i2c_port_type *p);
uint8_t READSDA(i2c_port_type *p);
void CLRSDA(i2c_port_type *p);
uint8_t READSCL(i2c_port_type *p);
void CLRSCL(i2c_port_type *p);
void I2CDELAY(volatile uint16_t clock_period);
void ARBITRATION_LOST(void);
uint8_t read_bit(i2c_port_type *p);
void write_bit(i2c_port_type *p, uint8_t bit);
void i2c_start(i2c_port_type *p);
void i2c_stop(i2c_port_type *p);
uint8_t i2c_write(i2c_port_type *p, uint8_t byte);
uint8_t i2c_read(i2c_port_type *p, uint8_t nak);
uint8_t i2c_tx(i2c_port_type *p, uint8_t send_start, uint8_t send_stop, uint8_t byte);
uint8_t i2c_rx(i2c_port_type *p, uint8_t nak, uint8_t send_stop);

#endif