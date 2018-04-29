// CPU definition file for STM32F407VG

#ifndef __CPU_SIM3U167_H__
#define __CPU_SIM3U167_H__

#include "type.h"
#include "stacks.h"
#include "platform_ints.h"
#include "SI32_PBSTD_A_Type.h"
#include "sim3u1xx.h"

extern unsigned platform_get_console_uart( void );

#define TIMER1_HZ 1800 //High priority timer for uarts = 115200/8/1800 = 4 bytes per tick for ~57600 bps (BLE speed)

SI32_PBSTD_A_Type* const port_std[] = { SI32_PBSTD_0, SI32_PBSTD_1, SI32_PBSTD_2, SI32_PBSTD_3 };

#define CON_VIRTUAL_ID 255

// #if defined( BUILD_USB_CDC )
//   #if defined( EXTERNAL_CONSOLE )
//     #define CON_VIRTUAL_ID 255
//     #define CON_UART_ID CON_VIRTUAL_ID
//   #else
//     #define CON_UART_ID         ( platform_get_console_uart() )
//     #define CON_UART_ID_HW_UART  0
//   #endif
// #else
//   #if defined( EXTERNAL_CONSOLE )
//     #define CON_VIRTUAL_ID 255
//     #define CON_UART_ID CON_VIRTUAL_ID
//   #else
//     #if defined( ELUA_BOARD_SIM3U1XXBDK )
//       #define CON_UART_ID           2
//     #else
//       #define CON_UART_ID           0
//     #endif
//   #endif
// #endif

// Number of resources (0 if not available/not implemented)
#define NUM_PIO               5
#define NUM_SPI               0
#define NUM_UART              4
#define NUM_TIMER             1
#define NUM_PHYS_TIMER        1
#define NUM_PWM               0
#define NUM_ADC               3
#define NUM_CAN               0
#define NUM_I2C               2

#define ADC_BIT_RESOLUTION    12

u32 cmsis_get_cpu_frequency();
#define CPU_FREQUENCY         cmsis_get_cpu_frequency()

// PIO prefix ('0' for P0, P1, ... or 'A' for PA, PB, ...)
#define PIO_PREFIX            '0'
// Pins per port configuration:
// #define PIO_PINS_PER_PORT (n) if each port has the same number of pins, or
// #define PIO_PIN_ARRAY { n1, n2, ... } to define pins per port in an array
// Use #define PIO_PINS_PER_PORT 0 if this isn't needed
#define PIO_PIN_ARRAY     { 16, 16, 15, 12, 6 }

// Internal memory data
#define SRAM_ORIGIN                      0x20000000
#define SRAM_SIZE                        0x8000
#define INTERNAL_RAM1_FIRST_FREE         end
#define INTERNAL_RAM1_LAST_FREE          ( void* )( SRAM_ORIGIN + SRAM_SIZE - STACK_SIZE_TOTAL - 1 )

#define INTERNAL_FLASH_SIZE             ( 256 * 1024 )
#define INTERNAL_FLASH_SECTOR_SIZE      1024
#define INTERNAL_FLASH_WRITE_UNIT_SIZE  4

#define BOOTLOADER_SIZE                 0x3000

#define INTERNAL_FLASH_START_ADDRESS    0x00000000

// Interrupt list for this CPU
#define PLATFORM_CPU_CONSTANTS_INTS\
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
    _C( INT_BOOT ), \
    _C( INT_CONTENTION ), \
    _C( INT_TICKSECOND ), \
    _C( INT_GSM_SIGNAL ), \
    _C( INT_GSM_TX_OK ), \
    _C( INT_GSM_TX_FAIL ), \
    _C( INT_GSM_TIMEOUT ),

#define RRAM_SIZE 8

#define RRAM_INT_SLEEPTIME 0
#define RRAM_BIT_ALERT 40
  #define ALERT_MODE_ACTIVE 1
  #define ALERT_MODE_DISABLED 0
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
#define RRAM_BIT_WAKE_ON_INPUT1 45
  #define WAKE_ON_INPUT1_DISABLED 0
  #define WAKE_ON_INPUT1_ACTIVE 1
#define RRAM_BIT_WAKE_ON_INPUT1_POLARITY 46
  #define WAKE_ON_INPUT1_POLARITY_POSITIVE 0
  #define WAKE_ON_INPUT1_POLARITY_NEGATIVE 1
#define RRAM_BIT_WAKE_ON_INPUT2 47
  #define WAKE_ON_INPUT2_DISABLED 0
  #define WAKE_ON_INPUT2_ACTIVE 1
#define RRAM_BIT_WAKE_ON_INPUT2_POLARITY 48
  #define WAKE_ON_INPUT2_POLARITY_POSITIVE 0
  #define WAKE_ON_INPUT2_POLARITY_NEGATIVE 1
#define RRAM_BIT_ALERT_SINGLE 49
#define ALERT_MODE_SINGLE_ACTIVE 1
#define ALERT_MODE_SINGLE_DISABLED 0
#define RRAM_BIT_BLUETOOTH_WAKE 50
#define BLUETOOTH_WAKE_ACTIVE 1
#define BLUETOOTH_WAKE_DISABLED 0
#define RRAM_BIT_TEMP_STORAGE_MODE 51
#define TEMP_STORAGE_MODE_ACTIVE 1
#define TEMP_STORAGE_MODE_DISABLED 0
#define RRAM_BIT_BLUETOOTH_PAIRABLE 52
#define BLUETOOTH_PAIRABLE_ACTIVE 1
#define BLUETOOTH_PAIRABLE_DISABLED 0
#define RRAM_BIT_GPS_HIBERNATE_WHILE_SLEEPING 53
#define GPS_HIBERNATE_WHILE_SLEEPING_ACTIVE 1
#define GPS_HIBERNATE_WHILE_SLEEPING_DISABLED 0
#define RRAM_BIT_INPUT1_LAST_STATE 54
#define INPUT1_LAST_STATE_LOW 0
#define INPUT1_LAST_STATE_HIGH 1
#define RRAM_BIT_INPUT2_LAST_STATE 55
#define INPUT2_LAST_STATE_LOW 0
#define INPUT2_LAST_STATE_HIGH 1
#define RRAM_BIT_INPUT1_TRIGGERED 56
#define INPUT1_UNTRIGGERED 0
#define INPUT1_TRIGGERED 1
#define RRAM_BIT_INPUT2_TRIGGERED 57
#define INPUT2_UNTRIGGERED 0
#define INPUT2_TRIGGERED 1
#define RRAM_BIT_SLEEP_WITH_BATTERY 58
#define SLEEP_WITH_BATTERY_ACTIVE 1
#define SLEEP_WITH_BATTERY_DISABLED 0
#define RRAM_BIT_MOVING_MODE 59
#define MOVING_MODE_ACTIVE 1
#define MOVING_MODE_DISABLED 0
#define RRAM_BIT_BLE_OFF 59
#define BLE_OFF_ACTIVE 1
#define BLE_OFF_DISABLED 0

#define RRAM_INT_CLK_CORR 3
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
typedef enum {
  OKTOSLEEP = 0,
  WAITTOSLEEP = 1
} ok_to_sleep_enum;
extern int ok_to_sleep();
extern void sim3_hard_fault_cleanup();
void reset_seconds_awake( void );
u8 extras_op_pending( void );

u8 extras_minimum_sleep_power( void );
typedef enum {
  VERY_LIGHT_LOAD_UNDER_5MA = 0,
  LIGHT_LOAD_BETWEEN_20MA_TO_5MA = 1,
  FULL_POWER_ABOVE_20MA = 2
} minimum_sleep_power;

int bat_abovethresh( void );
#define TRICK_TO_REBOOT_WITHOUT_DFU_MODE -1
#define SLEEP_FOREVER 0x7FFFFFFF
void sim3_pmu_pm9( int seconds );
void watchdog_counter_set( u16 value );

// Support for Compiling with & without rotables
#ifdef LUA_OPTIMIZE_MEMORY
#define LUA_ISCALLABLE( state, idx ) ( lua_isfunction( state, idx ) || lua_islightfunction( state, idx ) )
#else
#define LUA_ISCALLABLE( state, idx ) lua_isfunction( state, idx )
#endif
/*int load_lua_string (const char *s);
int load_lua_file (char *filename);
int load_lua_function (char *func);*/

typedef enum {
    WAKE_UNKNOWN = 0x00,
    WAKE_POWERUP = 0x01,
    WAKE_RESETPIN = 0x02,
    WAKE_WAKEPIN = 0x03,
    WAKE_RTC = 0x04,
    WAKE_POWERCONNECTED = 0x05,
    WAKE_IO = 0x06,
    WAKE_WATCHDOG = 0x07,
    WAKE_BLUETOOTH = 0x08,
    WAKE_PENDINGOP = 0x09
} wake_type;

extern int wake_reason;

extern unsigned console_cdc_active;


//#define PCB_V7 !!! this is defined in conf.lua now
//#define PCB_V7_CHARGER_NPN

#define BLUETOOTH_ENABLE_TDI_DTR
#define BLUETOOTH_ENABLE_TDO_DSR
#define BLUETOOTH_POWEREDWHILESLEEPING
//#define REBOOT_AT_END_OF_SLEEP
//define DEBUG_I2C
//define USE_EXTERNAL_MOSFETS

//#define LOW_SYSTEM_CLOCK

/*extern const u8 CLED_FADEUP[];
extern const u8 CLED_FADEDOWN[];
extern const u8 CLED_OFF[];
extern const u8 CLED_ON[];
extern const u8 CLED_FASTFLASH[];
extern const u8 CLED_MEDIUMFLASH[];
extern const u8 CLED_SLOWFLASH[];*/

enum {
  LED_FADEUP,
  LED_FADEDOWN,
  LED_OFF,
  LED_ON,
  LED_FASTFLASH,
  LED_MEDIUMFLASH,
  LED_SLOWFLASH,
  LED_FLASH1,
  LED_FLASH2,
  LED_FLASH3,
  LED_FLASH4,
  LED_FLASH5
} enum_led_state;

#if defined ( MEMBRANE_V1 )
enum {
  LED_COLOR_GPS = 0, // was sat
  LED_COLOR_MSG = 1, // was pwr
  LED_COLOR_PWR = 2, // was alrm
  LED_COLOR_SAT = 3, // was gps
  LED_COLOR_ALRM = 4, // was msg
  LED_COLOR_INTERNAL = 5
};
#else
enum {
  LED_COLOR_SAT = 0,
  LED_COLOR_PWR = 1,
  LED_COLOR_ALRM = 2,
  LED_COLOR_GPS = 3,
  LED_COLOR_MSG = 4
};
#endif

void led_set_mode(int led, int mode, int cycles);
int led_get_mode(int led);
void led_set_mask( u8 mask );
void led_cache_mode(int led, int mode );
void led_set_background(int led, u8 bkgnd);
int external_power( void );
s32 adc_get_single_sample( int adc_id );
int32_t gps_get_rtc_correction( void );

#define SHELL_WELCOMEMSG_EXTRA "\nGSatMicro http://gsat.us/"

#undef SHELL_PROMPT
#define SHELL_PROMPT "GSatMicro# "

#undef SHELL_HELP_VER_STRING
#define SHELL_HELP_VER_STRING "\nThis displays the git revision of the tree used to build or an official version number if applicable.\n"

#undef SHELL_HELP_SUMMARY_STRING
#define SHELL_HELP_SUMMARY_STRING "show version information"

#undef SHELL_HELP_LINE1_STRING
#define SHELL_HELP_LINE1_STRING "GSatMicro version %s\n" //, ELUA_STR_VERSION

#undef SHELL_HELP_LINE2_STRING
#define SHELL_HELP_LINE2_STRING "For more information visit www.gsat.us\n"

#define ROMFS_SECURE_FILENAMES_WITH_CHAR "~" //Enable security for files with a ~ char

#endif // #ifndef __CPU_SIM3U167_H__
