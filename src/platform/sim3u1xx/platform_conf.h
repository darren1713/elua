// eLua platform configuration

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#include "auxmods.h"
#include "stacks.h"
#include "type.h"
#include "extra_libs.h"

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
#define BUILD_USB_CDC

#define ENABLE_PMU

#define ENABLE_INTERACTIVE_AUTORUN

#define PLATFORM_HAS_SYSTIMER

// *****************************************************************************
// UART/Timer IDs configuration data (used in main.c)


extern unsigned platform_get_console_uart( void );

#define EXTERNAL_CONSOLE

#if defined( BUILD_USB_CDC )
#if defined( EXTERNAL_CONSOLE )
#define CON_VIRTUAL_ID 255
#define CON_UART_ID CON_VIRTUAL_ID
#else
#define CON_UART_ID         ( platform_get_console_uart() )
#define CON_UART_ID_HW_UART  0
#endif
#else
#if defined( ELUA_BOARD_SIM3U1XXBDK )
#define CON_UART_ID           2
#else
#define CON_UART_ID           0
#endif
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
// #ifdef ENABLE_PMU
// #define PS_LIB_TABLE_NAME "sim3"
// #endif


#ifdef PS_LIB_TABLE_NAME
#define PLATLINE _ROM( PS_LIB_TABLE_NAME, luaopen_platform, platform_map )
#else
#define PLATLINE
#endif

#define LUA_PLATFORM_LIBS_ROM\
  _ROM( AUXLIB_PIO, luaopen_pio, pio_map )\
  _ROM( AUXLIB_UART, luaopen_uart, uart_map )\
  _ROM( AUXLIB_PD, luaopen_pd, pd_map )\
  _ROM( AUXLIB_I2C, luaopen_i2c, i2c_map )\
  _ROM( AUXLIB_TMR, luaopen_tmr, tmr_map )\
  ADCLINE\
  _ROM( AUXLIB_TERM, luaopen_term, term_map )\
  _ROM( AUXLIB_PACK, luaopen_pack, pack_map )\
  _ROM( AUXLIB_BIT, luaopen_bit, bit_map )\
  _ROM( AUXLIB_CPU, luaopen_cpu, cpu_map )\
  RPCLINE\
  _ROM( LUA_MATHLIBNAME, luaopen_math, math_map )\
  _ROM( AUXLIB_ELUA, luaopen_elua, elua_map )\
  _ROM( "pmu", luaopen_pmu, pmu_map )\
  SIM3_EXTRA_LIBS_ROM\
  PLATLINE

LUALIB_API int ( luaopen_pmu )( lua_State *L );


// *****************************************************************************
// Configuration data

#define EGC_INITIAL_MODE      1

// SysTick Frequency
#define SYSTICKHZ             100

// Virtual timers (0 if not used)
#define VTMR_NUM_TIMERS       0
#define VTMR_FREQ_HZ          4

// Number of resources (0 if not available/not implemented)
#define NUM_PIO               5
#define NUM_SPI               0
#define NUM_UART              4
#define NUM_PWM               0
#define NUM_I2C               2
#define NUM_ADC               0
#define NUM_CAN               0
#define NUM_TIMER             1


// Enable RX buffering on UART
#define BUF_ENABLE_UART
#define CON_BUF_SIZE          BUF_SIZE_128

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
#define PIO_PIN_ARRAY     { 16, 16, 15, 12, 6 }

// Allocator data: define your free memory zones here in two arrays
// (start address and end address)
#define SRAM_ORIGIN           0x20000000
#define SRAM_SIZE             0x8000
//#define SRAM2_ORIGIN          0x20000000
//#define SRAM2_SIZE            0x1000
#define MEM_START_ADDRESS     { ( void* )end }
#define MEM_END_ADDRESS       { ( void* )( SRAM_ORIGIN + SRAM_SIZE - STACK_SIZE_TOTAL - 1 ) }

// Flash data (only for STM32F103RE for now)
#ifdef ELUA_CPU_SIM3U167
#define INTERNAL_FLASH_SIZE             ( 256 * 1024 )
#define INTERNAL_FLASH_SECTOR_SIZE      1024
#define INTERNAL_FLASH_WRITE_UNIT_SIZE  4

#if defined( USE_BOOTLOADER )
#define INTERNAL_FLASH_START_ADDRESS    0x00003000
#else
#define INTERNAL_FLASH_START_ADDRESS    0x00000000
#endif

#define BUILD_WOFS
#endif // #ifdef ELUA_CPU_SIM3U167

// Interrupt queue size
#define PLATFORM_INT_QUEUE_LOG_SIZE 5

// Interrupt list
#define INT_UART_RX          ELUA_INT_FIRST_ID
#define INT_UART_BUF_FULL    ( ELUA_INT_FIRST_ID + 1 )
#define INT_UART_BUF_MATCH   ( ELUA_INT_FIRST_ID + 2 )
#define INT_SYSTICK          ( ELUA_INT_FIRST_ID + 3 )
#define INT_IRIDIUM_SIGNAL   ( ELUA_INT_FIRST_ID + 4 )
#define INT_IRIDIUM_TX_OK    ( ELUA_INT_FIRST_ID + 5 )
#define INT_IRIDIUM_TX_FAIL  ( ELUA_INT_FIRST_ID + 6 )
#define INT_IRIDIUM_TIMEOUT  ( ELUA_INT_FIRST_ID + 7 )
#define INT_GPS_VALID        ( ELUA_INT_FIRST_ID + 8 )
#define INT_GPS_TIMEOUT      ( ELUA_INT_FIRST_ID + 9 )
#define INT_SYSINIT          ( ELUA_INT_FIRST_ID + 10 )
#define INT_ELUA_LAST        INT_SYSINIT

#define PLATFORM_CPU_CONSTANTS\
    _C( INT_UART_RX ),        \
    _C( INT_UART_BUF_FULL ),  \
    _C( INT_UART_BUF_MATCH ), \
    _C( INT_SYSTICK ), \
    _C( INT_IRIDIUM_SIGNAL ), \
    _C( INT_IRIDIUM_TX_OK ), \
    _C( INT_IRIDIUM_TX_FAIL ), \
    _C( INT_IRIDIUM_TIMEOUT ), \
    _C( INT_GPS_VALID ), \
    _C( INT_GPS_TIMEOUT ), \
    _C( INT_SYSINIT )

#define RRAM_SIZE 8

#define RRAM_INT_SLEEPTIME 0
#define RRAM_BIT_SOS 40
  #define SOS_MODE_ACTIVE 1
  #define SOS_MODE_DISABLED 0
#define RRAM_BIT_CHECKIN 41
  #define CHECKIN_MODE_ACTIVE 1
  #define CHECKIN_MODE_DISABLED 0
#define RRAM_BIT_POWEROFF 42
  #define POWEROFF_MODE_ACTIVE 1
  #define POWEROFF_MODE_DISABLED 0
#define RRAM_BIT_STORAGE_MODE 43
  #define STORAGE_MODE_ACTIVE 1
  #define STORAGE_MODE_DISABLED 0
#define RRAM_BIT_SLEEP_WHEN_POWERED 44
  #define SLEEP_WHEN_POWERED_ACTIVE 1
  #define SLEEP_WHEN_POWERED_DISABLED 0

#define RRAM_INT_X_Z 4
#define RRAM_INT_Y_Z 5
#define RRAM_INT_TIME 6
#define RRAM_INT_Z_DRIFT 7

// Sleep Persistent SRAM Storage
extern int rram_reg[RRAM_SIZE] __attribute__((section(".sret")));
extern int rram_read_int(int byte_number);
extern void rram_write_int(int byte_number, int value);
extern int rram_read_byte(int byte_number);
extern void rram_write_byte(int byte_number, int value);
extern int rram_read_bit(int bit_number);
extern void rram_write_bit(int bit_number, int value);
extern void button_down(int port, int pin);
extern void button_up(int port, int pin);
#define TRICK_TO_REBOOT_WITHOUT_DFU_MODE 0xFFFFFFFF
#define SLEEP_FOREVER 0x7FFFFFFF
void sim3_pmu_pm9( unsigned seconds );

typedef enum {
    WAKE_UNKNOWN = 0x00,
    WAKE_POWERUP = 0x01,
    WAKE_RESETPIN = 0x02,
    WAKE_WAKEPIN = 0x03,
    WAKE_RTC = 0x04,
    WAKE_POWERCONNECTED = 0x05

} wake_type;

extern int wake_reason;

extern unsigned console_cdc_active;

//define CHARGER_MOSFETS_PRESENT
//define BLUETOOTH_POWEREDWHILESLEEPING

#undef SHELL_WELCOMEMSG
#define SHELL_WELCOMEMSG "\nGSatMicro %s\n"

#endif // #ifndef __PLATFORM_CONF_H__

