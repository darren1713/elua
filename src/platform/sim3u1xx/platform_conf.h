// eLua platform configuration

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#include "auxmods.h"
#include "stacks.h"
#include "type.h"

// *****************************************************************************
// Define here what components you want for this platform

#define BUILD_XMODEM
#define BUILD_SHELL
#define BUILD_ROMFS
#define BUILD_TERM
#define BUILD_CON_GENERIC
//#define BUILD_ADC
#define BUILD_RPC

#define BUILD_C_INT_HANDLERS
#define BUILD_LUA_INT_HANDLERS

#define ENABLE_PMU

#define PLATFORM_HAS_SYSTIMER

// *****************************************************************************
// UART/Timer IDs configuration data (used in main.c)

#if defined( ELUA_BOARD_SIM3U1XXBDK )
#define CON_UART_ID           2
#else
#define CON_UART_ID           0
#endif
#define CON_UART_SPEED        115200
#define TERM_LINES            25
#define TERM_COLS             80

// *****************************************************************************
// Auxiliary libraries that will be compiled for this platform

#ifdef BUILD_ADC
#define ADCLINE _ROM( AUXLIB_ADC, luaopen_adc, adc_map )
#else
#define ADCLINE
#endif

// RPC
#if defined( BUILD_RPC ) 
#define RPCLINE _ROM( AUXLIB_RPC, luaopen_rpc, rpc_map )
#else
#define RPCLINE
#endif

// The name of the platform specific libs table
#ifdef ENABLE_PMU
#define PS_LIB_TABLE_NAME "sim3"
#endif


#ifdef PS_LIB_TABLE_NAME
#define PLATLINE _ROM( PS_LIB_TABLE_NAME, luaopen_platform, platform_map )
#else
#define PLATLINE
#endif

#define LUA_PLATFORM_LIBS_ROM\
  _ROM( AUXLIB_PIO, luaopen_pio, pio_map )\
  _ROM( AUXLIB_UART, luaopen_uart, uart_map )\
  _ROM( AUXLIB_PD, luaopen_pd, pd_map )\
  _ROM( AUXLIB_TMR, luaopen_tmr, tmr_map )\
  ADCLINE\
  _ROM( AUXLIB_TERM, luaopen_term, term_map )\
  _ROM( AUXLIB_PACK, luaopen_pack, pack_map )\
  _ROM( AUXLIB_BIT, luaopen_bit, bit_map )\
  _ROM( AUXLIB_CPU, luaopen_cpu, cpu_map )\
  RPCLINE\
  _ROM( LUA_MATHLIBNAME, luaopen_math, math_map )\
  _ROM( AUXLIB_ELUA, luaopen_elua, elua_map )\
  PLATLINE

// *****************************************************************************
// Configuration data

#define EGC_INITIAL_MODE      1

// Virtual timers (0 if not used)
#define VTMR_NUM_TIMERS       0
#define VTMR_FREQ_HZ          4

// Number of resources (0 if not available/not implemented)
#define NUM_PIO               4
#define NUM_SPI               0
#define NUM_UART              4
#define NUM_PWM               0
#define NUM_ADC               0
#define NUM_CAN               0
#define NUM_TIMER             1


// Enable RX buffering on UART
#define BUF_ENABLE_UART
#define CON_BUF_SIZE          BUF_SIZE_64

// ADC Configuration Params
// #define ADC_BIT_RESOLUTION    12
// #define BUF_ENABLE_ADC
// #define ADC_BUF_SIZE          BUF_SIZE_2

// These should be adjusted to support multiple ADC devices
// #define ADC_TIMER_FIRST_ID    0
// #define ADC_NUM_TIMERS        4

// RPC  
#define RPC_UART_ID           CON_UART_ID

// CPU frequency (needed by the CPU module, 0 if not used)
u32 cmsis_get_cpu_frequency();
#define CPU_FREQUENCY         cmsis_get_cpu_frequency()

// PIO prefix ('0' for P0, P1, ... or 'A' for PA, PB, ...)
#define PIO_PREFIX            '0'
// Pins per port configuration:
// #define PIO_PINS_PER_PORT (n) if each port has the same number of pins, or
// #define PIO_PIN_ARRAY { n1, n2, ... } to define pins per port in an array
// Use #define PIO_PINS_PER_PORT 0 if this isn't needed

// SiM3U1x7 - Crossbar Ports (PB4 left off for now)
#define PIO_PIN_ARRAY     { 16, 16, 15, 12 }

// Allocator data: define your free memory zones here in two arrays
// (start address and end address)
#define SRAM_ORIGIN           0x20000000
#define SRAM_SIZE             0x8000
//#define SRAM2_ORIGIN          0x20000000
//#define SRAM2_SIZE            0x1000
#define MEM_START_ADDRESS     { ( void* )end }
#define MEM_END_ADDRESS       { ( void* )( SRAM_ORIGIN + SRAM_SIZE - STACK_SIZE_TOTAL - 1 ) }
 
// Interrupt queue size
#define PLATFORM_INT_QUEUE_LOG_SIZE 5

// Interrupt list
#define INT_UART_RX      ELUA_INT_FIRST_ID
#define INT_ELUA_LAST         INT_UART_RX

#define PLATFORM_CPU_CONSTANTS\
  _C( INT_UART_RX )

#endif // #ifndef __PLATFORM_CONF_H__
