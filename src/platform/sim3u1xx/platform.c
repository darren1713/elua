// Platform-dependent functions

#include "platform.h"
#include "type.h"
#include "devman.h"
#include "genstd.h"
#include "stacks.h"
#include <reent.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "utils.h"
#include "common.h"
#include "elua_adc.h"
#include "platform_conf.h"
#include "lrotable.h"
#include "buf.h"
#include "sermux.h"

// Platform includes
#include "sim3u1xx.h"
#include "sim3u1xx_Types.h"

// FreakUSB
#include "cdc.h"
#include "freakusb.h"
#include "hw.h"

// Generated code includes
#include <SI32_PBCFG_A_Type.h>
#include <SI32_PBSTD_A_Type.h>
#include <si32_device.h>
#include <SI32_RTC_A_Type.h>
#include <SI32_RSTSRC_A_Type.h>
#include <SI32_VREG_A_Type.h>
#include <SI32_PMU_A_Type.h>
#include <SI32_SARADC_A_Type.h>
#include <SI32_LDO_A_Type.h>
#include "myPB.h"
#include <gVMON0.h>
#include <gLDO0.h>
#include <gVREG0.h>
#include <gRSTSRC0.h>

// ****************************************************************************
// Platform initialization

#define PIN_CHECK_INTERVAL 10
int wake_reason = WAKE_UNKNOWN;

// Watchdog timer
#define PLATFORM_EARLY_WARNING_DELAY_MS        500   // Will result in approx a 1 s
                                             // periodic early warning interrupt

#define PLATFORM_RESET_DELAY_MS                3000  // Will result in approx a 2 s
                                            // reset delay (if early warning isn't captured)

#define PLATFORM_EARLY_WARNING_THRESHOLD       (uint32_t)((16400*PLATFORM_EARLY_WARNING_DELAY_MS)/1000)
#define PLATFORM_RESET_THRESHOLD               (uint32_t)((16400*PLATFORM_RESET_DELAY_MS)/1000)

#ifdef EXTRA_SLEEP_HOOK
extern void extras_sleep_hook( int seconds );
#endif

//I2C
#define I2C_TIMEOUT_SYSTICKS 3
static volatile int i2c_timeout_timer = I2C_TIMEOUT_SYSTICKS;

#if defined( PCB_V7 ) || defined( PCB_V8 )
#define PIN_HV_BANK SI32_PBSTD_3
#define PIN_HV_PIN ( 1 << 8 )
#else
#define PIN_HV_BANK SI32_PBSTD_3
#define PIN_HV_PIN ( 1 << 7 )
#endif

int rram_reg[RRAM_SIZE] __attribute__((section(".sret")));
static int rtc_remaining = 0;
static u8 sleep_delay = 0;

void sim3_pmu_reboot( void );
void sim3_pmu_reboot_nodfu( void );

extern int load_lua_function (char *func);

// forward dcls
static void pios_init();
static void clk_init();
static void rtc_init();
static void adcs_init();
static void gTIMER0_enter_auto_reload_config(void);
static void gTIMER1_enter_auto_reload_config(void);

static SI32_PBSTD_A_Type* const port_std[] = { SI32_PBSTD_0, SI32_PBSTD_1, SI32_PBSTD_2, SI32_PBSTD_3 };


// Reference: http://stackoverflow.com/q/2422712/105950
int div_round_closest( const int numerator, const int denominator )
{
  if( ( ( numerator >= 0 ) && ( denominator >= 0 ) ) ||
      ( ( numerator <  0 ) && ( denominator <  0 ) ) )
 {
    // Both are positive or negative, positive result
    if( denominator == 0 )
      return INT_MAX;
    else
      return ( numerator + denominator / 2 ) / denominator;
  }
  else
  {
    // One is postive the other is negative, negative result
    if( denominator == 0 )
      return INT_MIN;
    else
      return ( numerator - denominator / 2 ) / denominator;
  }
}

void hard_fault_handler_c(unsigned int * hardfault_args)
{
  FILE *fp;

  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

  printf ("[Hard fault handler]\n");
  printf ("R0 = %x\n", stacked_r0);
  printf ("R1 = %x\n", stacked_r1);
  printf ("R2 = %x\n", stacked_r2);
  printf ("R3 = %x\n", stacked_r3);
  printf ("R12 = %x\n", stacked_r12);
  printf ("LR = %x\n", stacked_lr);
  printf ("PC = %x\n", stacked_pc);
  printf ("PSR = %x\n", stacked_psr);
  printf ("BFAR = %x\n", (*((volatile unsigned  *)(0xE000ED38))));
  printf ("CFSR = %x\n", (*((volatile unsigned  *)(0xE000ED28))));
  printf ("HFSR = %x\n", (*((volatile unsigned  *)(0xE000ED2C))));
  printf ("DFSR = %x\n", (*((volatile unsigned  *)(0xE000ED30))));
  printf ("AFSR = %x\n", (*((volatile unsigned  *)(0xE000ED3C))));
  printf ("\n");
  printf ("\n");
  printf ("\n");

  // Write out hard fault data to file
  fp = fopen("/wo/._hardfault", "w");
  if( fp != NULL )
  {
    fprintf (fp, "[Last hard fault]\n");
    fprintf (fp, "R0 = %x\n", stacked_r0);
    fprintf (fp, "R1 = %x\n", stacked_r1);
    fprintf (fp, "R2 = %x\n", stacked_r2);
    fprintf (fp, "R3 = %x\n", stacked_r3);
    fprintf (fp, "R12 = %x\n", stacked_r12);
    fprintf (fp, "LR = %x\n", stacked_lr);
    fprintf (fp, "PC = %x\n", stacked_pc);
    fprintf (fp, "PSR = %x\n", stacked_psr);
    fprintf (fp, "BFAR = %x\n", (*((volatile unsigned  *)(0xE000ED38))));
    fprintf (fp, "CFSR = %x\n", (*((volatile unsigned  *)(0xE000ED28))));
    fprintf (fp, "HFSR = %x\n", (*((volatile unsigned  *)(0xE000ED2C))));
    fprintf (fp, "DFSR = %x\n", (*((volatile unsigned  *)(0xE000ED30))));
    fprintf (fp, "AFSR = %x\n", (*((volatile unsigned  *)(0xE000ED3C))));
    printf ("\n");
    fclose( fp );
  }

  sim3_pmu_reboot_nodfu();
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  asm(
  "tst lr, #4\n\t"
  "ite eq\n\t"
  "mrseq r0, msp\n\t"
  "mrsne r0, psp\n\t"
  "b hard_fault_handler_c"
  );
}

void WDTIMER0_IRQHandler(void)
{
    if ((SI32_WDTIMER_A_is_early_warning_interrupt_pending(SI32_WDTIMER_0) &
        SI32_WDTIMER_A_is_early_warning_interrupt_enabled(SI32_WDTIMER_0)))
    {
      SI32_WDTIMER_A_reset_counter(SI32_WDTIMER_0);
      SI32_WDTIMER_A_clear_early_warning_interrupt(SI32_WDTIMER_0);
    }
}

#if defined( BUILD_USB_CDC )
unsigned console_cdc_active = 0;
#endif



// SiM3 SystemInit calls this function, disable watchdog timer
void mySystemInit(void)
{
  // Setup Watchdog Timer
  SI32_WDTIMER_A_stop_counter(SI32_WDTIMER_0);

  // enable APB clock to the Port Bank module
  SI32_CLKCTRL_A_enable_apb_to_modules_0 (SI32_CLKCTRL_0, SI32_CLKCTRL_A_APBCLKG0_PB0CEN_MASK);
  // make the SWO pin (PB1.3) push-pull to enable SWV printf
  //SI32_PBSTD_A_set_pins_push_pull_output (SI32_PBSTD_1, (1<<3));

}

#if defined( ELUA_BOARD_GSATMICRO )

int usb_power()
{
  return ( int )SI32_VREG_A_is_vbus_valid( SI32_VREG_0 );
}

int external_power()
{
  //check USB DC 3.9 or HVDC 3.8
  if( ( SI32_PBSTD_A_read_pins( PIN_HV_BANK ) & ( PIN_HV_PIN ) ) ||
      ( usb_power() ) )
    return 1;
  else
    return 0;
}

int external_buttons()
{
#if defined( PCB_V7 ) || defined( PCB_V8 )
  //check inputs 1 and 2
  if( ( SI32_PBSTD_A_read_pins( SI32_PBSTD_2 ) & ( 1 << 2 ) ) ||
      ( SI32_PBSTD_A_read_pins( SI32_PBSTD_2 ) & ( 1 << 3 ) ) ||
      ( SI32_PBSTD_A_read_pins( SI32_PBSTD_2 ) & ( 1 << 4 ) ) )
    return 1;
  else
    return 0;
#else
  //check inputs 1 and 2
  if( /*( SI32_PBSTD_A_read_pins( SI32_PBSTD_3 ) & ( 1 << 6 ) ) ||*/
      ( SI32_PBSTD_A_read_pins( SI32_PBSTD_0 ) & ( 1 << 1 ) ) )
    return 1;
  else
    return 0;
#endif
}
int external_io()
{
#if defined( PCB_V7 ) || defined( PCB_V8 )
  if(rram_read_bit(RRAM_BIT_WAKE_ON_INPUT1) == WAKE_ON_INPUT1_ACTIVE)
  {
    int val = SI32_PBSTD_A_read_pins( SI32_PBSTD_1 ) & ( 1 << 14 );
    if(rram_read_bit(RRAM_BIT_WAKE_ON_INPUT1_POLARITY) == WAKE_ON_INPUT1_POLARITY_POSITIVE)
    {
      if(val) return 1;
    } else {
      if(!val) return 1;
    }
  }
  if(rram_read_bit(RRAM_BIT_WAKE_ON_INPUT2) == WAKE_ON_INPUT2_ACTIVE)
  {
    int val = SI32_PBSTD_A_read_pins( SI32_PBSTD_1 ) & ( 1 << 15 );
    if(rram_read_bit(RRAM_BIT_WAKE_ON_INPUT2_POLARITY) == WAKE_ON_INPUT2_POLARITY_POSITIVE)
    {
      if(val) return 1;
    } else {
      if(!val) return 1;
    }
  }
#if defined( BLUETOOTH_ENABLE_TDI_DTR )
  if( platform_pio_op( 1, 1 << 4, PLATFORM_IO_PIN_GET ) )
    return 1;
#endif
#endif
  return 0;
}
int bluetooth_connected()
{
#if defined( BLUETOOTH_ENABLE_TDI_DTR )
  if( platform_pio_op( 1, 1 << 4, PLATFORM_IO_PIN_GET ) )
    return 1;
#endif
  return 0;
}


#endif
static int pmu_wake_status = -1;
static int pmu_status = -1;
static int reset_status = -1;
void reset_parameters()
{
  int i;
  for(i=0;i<8;i++)
  {
    rram_write_int(i, 0);
  }
}

void wake_init( void )
{
#if defined( ELUA_BOARD_GSATMICRO )

  //Determine if we had a power failure, voltage dropout, or reset button pressed
  //The pre-generated code for SI32_RSTSRC_A_get_last_reset_source is incorrect and does
  //not take into account PORRF. Also, these registers MUST be read in order PORRF, VMONRF, PINRF
  //as the remainder are invalid if the previous one is set. See table 6.2.

  // Wake from PM9
  if((pmu_status & SI32_PMU_A_STATUS_PM9EF_MASK) == SI32_PMU_A_STATUS_PM9EF_SET_U32)
  {
    //Check for reset pin while in PM9
    if((pmu_wake_status & SI32_PMU_A_WAKESTATUS_RSTWF_MASK) == SI32_PMU_A_WAKESTATUS_RSTWF_SET_U32)
    {
      reset_parameters();
      wake_reason = WAKE_RESETPIN;
    }
    else if((pmu_status & SI32_PMU_A_STATUS_PWAKEF_MASK) == (0 << SI32_PMU_A_STATUS_PWAKEF_SHIFT)) //Check for pin wake NOTE: SiLabs headers are wrong, this bit is backwards per the manual...
    {
      //Wakeup from WAKE pins, just reset reg[0] so we stay awake
      wake_reason = WAKE_WAKEPIN;

      //Put the remaining sleep time back into rram_reg[0]
      rram_write_int(RRAM_INT_SLEEPTIME, rram_read_int(RRAM_INT_SLEEPTIME) + rtc_remaining);

#ifdef EXTRA_SLEEP_HOOK
      //pass negative time to notify early wakeup
      //extras_sleep_hook(rtc_remaining * -1);
#endif

      //Don't auto-sleep for some period of seconds
      sleep_delay = 5;
    }
    else
    {
      if(rram_read_bit(RRAM_BIT_POWEROFF) == POWEROFF_MODE_ACTIVE)
      {
        rram_write_int(RRAM_INT_SLEEPTIME, SLEEP_FOREVER); //will wakeup in 68 years
      }
      else if(rram_read_bit(RRAM_BIT_STORAGE_MODE) == STORAGE_MODE_ACTIVE)
      {
        //Sleep forever, in storage mode. Power button will wakeup device
        rram_write_int(RRAM_INT_SLEEPTIME, SLEEP_FOREVER); //will wakeup in 68 years
      }

      if(external_buttons() || external_power() )
      {
        //Continue on as normal. The timer will count down rram_reg and execute
        //the appropriate script when it reaches zero
        wake_reason = WAKE_POWERCONNECTED;
      }
      if( external_io() )
      {
        wake_reason = WAKE_IO;
        //Don't auto-sleep for some period of seconds
        sleep_delay = 5;
      }
      if( bluetooth_connected() )
      {
        wake_reason = WAKE_BLUETOOTH;
        //Don't auto-sleep for some period of seconds
        sleep_delay = 2;
      }
      else if( rram_read_int(RRAM_INT_SLEEPTIME) > 0 )  //Go back to sleep if we woke from a PMU wakeup
      {
        sim3_pmu_pm9( rram_read_int(RRAM_INT_SLEEPTIME) );
        wake_reason = WAKE_RTC;
      }
    }
  }
  // Wake sources other than PM9
  else if (((reset_status & SI32_RSTSRC_A_RESETFLAG_PORRF_MASK) == SI32_RSTSRC_A_RESETFLAG_PORRF_SET_U32)
    ||  ((reset_status & SI32_RSTSRC_A_RESETFLAG_VMONRF_MASK) == SI32_RSTSRC_A_RESETFLAG_VMONRF_SET_U32)
    ||  ((reset_status & SI32_RSTSRC_A_RESETFLAG_PINRF_MASK) == SI32_RSTSRC_A_RESETFLAG_PINRF_SET_U32)
    ||  ((pmu_status & SI32_PMU_A_STATUS_PORF_MASK) == SI32_PMU_A_STATUS_PORF_SET_U32)
    ||  ((pmu_wake_status & SI32_PMU_A_WAKESTATUS_RSTWF_MASK) == SI32_PMU_A_WAKESTATUS_RSTWF_SET_U32)
    ||  ((reset_status & SI32_RSTSRC_A_RESETFLAG_WDTRF_MASK) == SI32_RSTSRC_A_RESETFLAG_WDTRF_SET_U32) )
  {
    if((pmu_wake_status & SI32_PMU_A_WAKESTATUS_RSTWF_MASK) == SI32_PMU_A_WAKESTATUS_RSTWF_SET_U32)
      wake_reason = WAKE_RESETPIN;
    if((reset_status & SI32_RSTSRC_A_RESETFLAG_WDTRF_MASK) == SI32_RSTSRC_A_RESETFLAG_WDTRF_SET_U32)
      wake_reason = WAKE_WATCHDOG;
    else if((pmu_status & SI32_PMU_A_STATUS_PWAKEF_MASK) == 0) //Check for pin wake NOTE: SiLabs headers are wrong, this bit is backwards per the manual...
      wake_reason = WAKE_WAKEPIN;
    else
      wake_reason = WAKE_POWERUP;

    reset_parameters();
  }

#endif
}


int platform_init()
{
  int i;
  SystemInit();

  // Configure the NVIC Preemption Priority Bits:
  // two (2) bits of preemption priority, six (6) bits of sub-priority.
  // Since the Number of Bits used for Priority Levels is five (5), so the
  // actual bit number of sub-priority is three (3)
  //NVIC_SetPriorityGrouping(0x05);

  // Setup peripherals
  // platform_setup_timers();
  SI32_VREG_A_enable_band_gap(SI32_VREG_0);
  SI32_VREG_A_exit_suspend_mode(SI32_VREG_0);

  // Peripheral Clocking setup
  clk_init();

  //Set flash read speed to slow
#if defined( LOW_SYSTEM_CLOCK )
  SI32_FLASHCTRL_A_select_flash_read_time_slow(SI32_FLASHCTRL_0);
#endif

  SI32_PMU_A_clear_pmu_level_shifter_hold(SI32_PMU_0);
  pmu_status = SI32_PMU_0->STATUS.U32;
  pmu_wake_status = SI32_PMU_0->WAKESTATUS.U32;
  reset_status = SI32_RSTSRC_0->RESETFLAG.U32;
  SI32_PMU_A_clear_pin_level_shifter_hold(SI32_PMU_0);
  SI32_PMU_A_clear_wakeup_flags(SI32_PMU_0);
  SI32_PMU_A_clear_por_flag(SI32_PMU_0);

  // GPIO setup
  pios_init();

  // System timer setup
  cmn_systimer_set_base_freq( cmsis_get_cpu_frequency() );
  cmn_systimer_set_interrupt_freq( SYSTICKHZ );

  // Enable SysTick
  SysTick_Config( cmsis_get_cpu_frequency() / SYSTICKHZ );

  // RTC Configuration
  SI32_RTC_A_start_timer_capture(SI32_RTC_0);
  while(SI32_RTC_A_is_timer_capture_in_progress(SI32_RTC_0));

  rtc_remaining = (SI32_RTC_A_read_alarm0(SI32_RTC_0)-SI32_RTC_A_read_setcap(SI32_RTC_0))/16384;
  rtc_init();

#ifdef BUILD_ADC
  // Setup ADCs
  adcs_init();
#endif

#if defined( BUILD_USB_CDC )

  usb_init();
  hw_init();

  // init the class driver here
  cdc_init();

  // register the rx handler function with the cdc
  cdc_reg_rx_handler(NULL);
#endif //defined( BUILD_USB_CDC )

  wake_init();

  // Common platform initialization code
  cmn_platform_init();

#if defined( INT_SYSINIT )
    cmn_int_handler( INT_SYSINIT, 0 );
#endif

  // Set the Systick as the highest priority, then uarts, then the rest
  NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
  for(i=WDTIMER0_IRQn;i<=VREG0LOW_IRQn;i++)
  {
    switch(i)
    {
      case UART0_IRQn:
      case UART1_IRQn:
      case USB0_IRQn:
      case USART0_IRQn:
      case USART1_IRQn:
        NVIC_SetPriority(i, (1 << __NVIC_PRIO_BITS) - 4);
        break;
      case TIMER1L_IRQn:
      case TIMER1H_IRQn:
        NVIC_SetPriority(i, (1 << __NVIC_PRIO_BITS) - 3);
        break;
      case TIMER0L_IRQn:
      case TIMER0H_IRQn:
        NVIC_SetPriority(i, (1 << __NVIC_PRIO_BITS) - 2);
        break;
      default:
        NVIC_SetPriority(i, (1 << __NVIC_PRIO_BITS) - 1);
        break;
    }
  }

  // Enable Timer 0
  gTIMER0_enter_auto_reload_config();
  // Enable Timer 1
  gTIMER1_enter_auto_reload_config();

  // Setup Watchdog Timer
  SI32_WDTIMER_A_stop_counter(SI32_WDTIMER_0);
  SI32_WDTIMER_A_reset_counter (SI32_WDTIMER_0);
  while(SI32_WDTIMER_A_is_threshold_update_pending(SI32_WDTIMER_0));
  SI32_WDTIMER_A_set_early_warning_threshold (SI32_WDTIMER_0, PLATFORM_EARLY_WARNING_THRESHOLD);
  while(SI32_WDTIMER_A_is_threshold_update_pending(SI32_WDTIMER_0));
  SI32_WDTIMER_A_set_reset_threshold (SI32_WDTIMER_0, PLATFORM_RESET_THRESHOLD);

#if 1 //Option to disable Watchdog Timer
  // Enable Watchdog Timer
  SI32_WDTIMER_A_start_counter(SI32_WDTIMER_0);

  /// Enable Watchdog Interrupt
  NVIC_ClearPendingIRQ(WDTIMER0_IRQn);
  SI32_WDTIMER_A_enable_early_warning_interrupt(SI32_WDTIMER_0);
  SI32_RSTSRC_A_enable_watchdog_timer_reset_source(SI32_RSTSRC_0);
  NVIC_EnableIRQ(WDTIMER0_IRQn);
#endif

  return PLATFORM_OK;
}

void clk_init( void )
{
#if defined( LOW_SYSTEM_CLOCK )
  while( SI32_CLKCTRL_A_are_system_clocks_busy( SI32_CLKCTRL_0 ) );
  SI32_CLKCTRL_A_select_ahb_divider(SI32_CLKCTRL_0, SI32_CLKCTRL_A_CONTROL_AHBDIV_DIV4_VALUE);

  // Set system clock to AHB divider frequency
  SystemCoreClock = 5000000;
#endif
#if defined( ELUA_BOARD_GSATMICRO )
  SI32_CLKCTRL_A_enable_apb_to_modules_0(SI32_CLKCTRL_0,
                                         SI32_CLKCTRL_A_APBCLKG0_PB0 |
                                         SI32_CLKCTRL_A_APBCLKG0_USART0 |
                                         SI32_CLKCTRL_A_APBCLKG0_USART1 |
                                         SI32_CLKCTRL_A_APBCLKG0_UART0 |
                                         SI32_CLKCTRL_A_APBCLKG0_UART1 |
                                         SI32_CLKCTRL_A_APBCLKG0_SPI0 |
                                         SI32_CLKCTRL_A_APBCLKG0_I2C0 |
                                         SI32_CLKCTRL_A_APBCLKG0_SARADC1 |
                                         SI32_CLKCTRL_A_APBCLKG0_AES0 |
                                         SI32_CLKCTRL_A_APBCLKG0_CRC0 |
                                         SI32_CLKCTRL_A_APBCLKG0_LPTIMER0 |
                                         SI32_CLKCTRL_A_APBCLKG0_USB0 |
                                         SI32_CLKCTRL_A_APBCLKG0_FLASHCTRL0 |
                                         SI32_CLKCTRL_A_APBCLKG0_CMP0
                                         );
  SI32_CLKCTRL_A_enable_apb_to_modules_1(SI32_CLKCTRL_0,
                                         SI32_CLKCTRL_A_APBCLKG1_MISC1 |
                                         SI32_CLKCTRL_A_APBCLKG1_MISC0);
  //SI32_CLKCTRL_A_enable_ahb_to_emif(SI32_CLKCTRL_0);

#else
  SI32_CLKCTRL_A_enable_apb_to_modules_0(SI32_CLKCTRL_0,
                                         SI32_CLKCTRL_A_APBCLKG0_PB0 |
                                         SI32_CLKCTRL_A_APBCLKG0_USART0 |
                                         SI32_CLKCTRL_A_APBCLKG0_USART1 |
                                         SI32_CLKCTRL_A_APBCLKG0_UART0 |
                                         SI32_CLKCTRL_A_APBCLKG0_UART1 |
                                         SI32_CLKCTRL_A_APBCLKG0_CMP0);
  SI32_CLKCTRL_A_enable_apb_to_modules_1(SI32_CLKCTRL_0,
                                         SI32_CLKCTRL_A_APBCLKG1_MISC1 |
                                         SI32_CLKCTRL_A_APBCLKG1_MISC0);
#endif
}
void RTC0ALRM_IRQHandler()
{
  if (SI32_RTC_A_is_alarm0_interrupt_pending(SI32_RTC_0))
  {
    SI32_RTC_A_clear_alarm0_interrupt(SI32_RTC_0);
    //printf("Alarm\n");
  }
}

void rtc_init( void )
{
  SI32_RTC_A_enable_high_speed_mode(SI32_RTC_0);
  // Low Frequency Oscillator Mode
  SI32_RTC_A_enable_low_frequency_oscillator(SI32_RTC_0);
  SI32_RTC_A_set_clock_source_lfo(SI32_RTC_0);
  SI32_RTC_A_disable_crystal_oscillator(SI32_RTC_0);

  SI32_RTC_A_enable_module(SI32_RTC_0);

  SI32_RTC_A_start_timer(SI32_RTC_0);
  SI32_RTC_A_enable_alarm0_auto_reset(SI32_RTC_0);
  SI32_RTC_A_write_alarm0(SI32_RTC_0, 0xF000);
  SI32_RTC_A_clear_alarm0_interrupt(SI32_RTC_0);

  SI32_RTC_A_enable_alarm0_interrupt(SI32_RTC_0);

  NVIC_ClearPendingIRQ(RTC0ALRM_IRQn);
  NVIC_EnableIRQ(RTC0ALRM_IRQn);
}

extern u32 SystemCoreClock;
u32 cmsis_get_cpu_frequency()
{
  return SystemCoreClock;
}
static u8 firstSecond = 1;
static u8 tickSeconds = 0;

void SecondsTick_Handler()
{
  //Check if we are supposed to be sleeping
  if(rram_read_int(RRAM_INT_SLEEPTIME) > 0)
  {
    //Don't count down timer if buttons are depressed
    if(rram_read_int(RRAM_INT_SLEEPTIME) != SLEEP_FOREVER)
    {
      if(rram_read_int(RRAM_INT_SLEEPTIME) != 1 || ( !external_buttons() && ok_to_sleep() == OKTOSLEEP ) )
        rram_write_int(RRAM_INT_SLEEPTIME, rram_read_int(RRAM_INT_SLEEPTIME)-1);
    }

    if(rram_read_int(RRAM_INT_SLEEPTIME) == 0)
    {
      //Our timer has expired and we are still powered, start TX script
      //Do a software reboot UNTIL we get the memory leaks sorted out...
      //sim3_pmu_reboot();
#ifdef REBOOT_AT_END_OF_SLEEP
      sim3_pmu_reboot_nodfu();
#endif
      //Normally we would run the startup script, but fix memory leaks first...
      //printf("startup %i\n", load_lua_function("autorun"));
      //printf("wakeup\n");
      cmn_int_handler( INT_BOOT, 0 );
      //printf("wakeup %i\n", load_lua_string("wakeup();\n"));
    }
    if( ( ( !external_power() && !external_buttons() && !external_io()) && !bluetooth_connected() ) ||
        ( ( rram_read_bit(RRAM_BIT_SLEEP_WHEN_POWERED) == SLEEP_WHEN_POWERED_ACTIVE ) && !usb_power() ) )
    {
      printf("no power %i\n", rram_read_int(RRAM_INT_SLEEPTIME));
      if(sleep_delay > 0)
        sleep_delay--;
      else
        sim3_pmu_pm9( rram_read_int(RRAM_INT_SLEEPTIME) );
    }
    //else if(rram_read_int(RRAM_INT_SLEEPTIME) != SLEEP_FOREVER &&
    //  ( ( ( rram_read_int(RRAM_INT_SLEEPTIME) % 300 ) == 0 ) || rram_read_int(RRAM_INT_SLEEPTIME) < 5 ) )
    //  printf("powered %i\n", rram_read_int(RRAM_INT_SLEEPTIME));
  }
  if(firstSecond)
  {
    printf("PWS 0x%x PS 0x%x RS 0x%x - %i rtc %i\n",
         pmu_wake_status,
         pmu_status,
         reset_status,
         wake_reason,
         rtc_remaining  );
    firstSecond = 0;
  }
}

static u8 seconds_tick_pending = 0;
void TIMER0H_IRQHandler(void)
{
#if defined( BUILD_USB_CDC )
  usb_pcb_t *pcb = usb_pcb_get();
  //Check if USB is powered. It will not actually TX/RX unless enumerated though
  if( SI32_VREG_A_is_vbus_valid( SI32_VREG_0 ) )
  {
    console_cdc_active = 1;
    pcb->connected = true;
  }
  else
  {
    console_cdc_active = 0;
    pcb->connected = false;
  }

  if( console_cdc_active )
    usb_poll();
#endif

#if defined( INT_SYSTICK )
  cmn_int_handler( INT_SYSTICK, 0 );
#endif

  if(seconds_tick_pending)
  {
    SecondsTick_Handler();
    seconds_tick_pending = 0;
  }

  // Clear the interrupt flag
  SI32_TIMER_A_clear_high_overflow_interrupt(SI32_TIMER_0);
}

//NOTE! These must be sized by a factor of 2 to calculate properly
//First byte is size of the array
#define LED_COUNT 5
#define LED_MAX_ARRAY 32

static u8 const CLED_FADEUP[] = { LED_MAX_ARRAY/2, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 };
static u8 const CLED_FADEDOWN[] = { LED_MAX_ARRAY/2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
static u8 const CLED_OFF[] = { 1, 15 };
static u8 const CLED_ON[] = { 1, 0 };
static u8 const CLED_FASTFLASH[] = { LED_MAX_ARRAY/4, 0, 15, 15, 15, 15, 15, 15, 15 };
static u8 const CLED_MEDIUMFLASH[] = { LED_MAX_ARRAY/2, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15 };
static u8 const CLED_SLOWFLASH[] = { LED_MAX_ARRAY, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
static u8 const CLED_FLASH1[] = { LED_MAX_ARRAY, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
static u8 const CLED_FLASH2[] = { LED_MAX_ARRAY, 0, 15, 15, 15, 15, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
static u8 const CLED_FLASH3[] = { LED_MAX_ARRAY, 0, 15, 15, 15, 15, 0, 15, 15, 15, 15, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
static u8 const CLED_FLASH4[] = { LED_MAX_ARRAY, 0, 15, 15, 15, 15, 0, 15, 15, 15, 15, 0, 15, 15, 15, 15, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
static u8 const CLED_FLASH5[] = { LED_MAX_ARRAY, 0, 15, 15, 15, 15, 0, 15, 15, 15, 15, 0, 15, 15, 15, 15, 0, 15, 15, 15, 15, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};

static u8 const * led_cled_ptr[] = {
  CLED_FADEUP,
  CLED_FADEDOWN,
  CLED_OFF,
  CLED_ON,
  CLED_FASTFLASH,
  CLED_MEDIUMFLASH,
  CLED_SLOWFLASH,
  CLED_FLASH1,
  CLED_FLASH2,
  CLED_FLASH3,
  CLED_FLASH4,
  CLED_FLASH5
};

#define LEDTICKHZ 800
u8 led_pointer_tick = 0;
u8 led_tick = 0;
u8 led_repeat_tick = 0;

u8 led_ticks = 0;

u8 led_ticks_ptr[LED_COUNT] = { 0, 0, 0, 0, 0 };
u8 led_background[LED_COUNT] = { 0, 0, 0, 0, 0 };

#define LED_REPEATS_FOREVER 255

//u8 * led_mode_ptr[] = { CLED_FADEDOWN, CLED_FADEUP, CLED_FASTFLASH, CLED_SLOWFLASH, CLED_MEDIUMFLASH };
//u8 led_repeats_ptr[LED_COUNT] = { 10, 10, 10, 10, 10 };

u8 * led_mode_ptr[] = { (u8 *)CLED_OFF, (u8 *)CLED_OFF, (u8 *)CLED_OFF, (u8 *)CLED_OFF, (u8 *)CLED_OFF };
u8 led_repeats_ptr[LED_COUNT] = { 10, 10, 10, 10, 10 };

//Pending variables here...wait for new "frame"
u8 * led_pending_mode_ptr[LED_COUNT] = { NULL, NULL, NULL, NULL, NULL };
u8 led_pending_repeats_ptr[LED_COUNT] = { 0, 0, 0, 0, 0 };

u8 led_mask = 0x00;

#if defined( PCB_V8 )
#define LED_PORT 0
#else
#define LED_PORT 2
#endif

void TIMER1H_IRQHandler(void)
{
  led_ticks=((led_ticks+1) & 0x0F);
  if(!led_ticks)
  {
    if(led_pointer_tick == 3)
    {
      led_pointer_tick = 0;

      //load next values here
      if(led_repeat_tick == 0)
      {
        led_repeat_tick = 0;

        int lednum;
        for(lednum=0;lednum<LED_COUNT;lednum++)
        {
          if(led_pending_mode_ptr[lednum] != NULL)
          {
            led_mode_ptr[lednum] = led_pending_mode_ptr[lednum];
            led_repeats_ptr[lednum] = led_pending_repeats_ptr[lednum];
            led_pending_mode_ptr[lednum] = NULL;
            led_pending_repeats_ptr[lednum] = 0;
          }
        }
      }

      int lednum;
      for(lednum=0;lednum<LED_COUNT;lednum++)
      {
        if(led_repeats_ptr[lednum] > 0)
        {
          if((led_repeats_ptr[lednum] != LED_REPEATS_FOREVER) && ((led_tick % led_mode_ptr[lednum][0]) == 0))
            led_repeats_ptr[lednum]--;
          if(led_repeats_ptr[lednum] > 0)
            led_ticks_ptr[lednum] = led_mode_ptr[lednum][(led_tick % led_mode_ptr[lednum][0])+1];
        }
      }
      led_tick++;
      led_repeat_tick = (led_repeat_tick+1) % LED_MAX_ARRAY;
    }
    led_pointer_tick++;
  }
  else
  {
    if(led_ticks + led_background[0] > led_ticks_ptr[0] && (led_mask & 1 ) )
      SI32_PBSTD_A_write_pins_high( port_std[ LED_PORT ], ( ( u32 ) 1 << 5 ) );
    else
      SI32_PBSTD_A_write_pins_low( port_std[ LED_PORT ], ( ( u32 ) 1 << 5 ) );
    if(led_ticks + led_background[1] > led_ticks_ptr[1] && (led_mask & 1<<1 ) )
    {
      SI32_PBSTD_A_write_pins_high( port_std[ LED_PORT ], ( ( u32 ) 1 << 6 ) );
    #if !defined( MEMBRANE_V1)
    #if defined( PCB_V8 )
      SI32_PBSTD_A_write_pins_high( port_std[ LED_PORT ], ( ( u32 ) 1 << 4 ) );
    #else
      SI32_PBHD_A_write_pins_high( SI32_PBHD_4, ( ( u32 ) 1 << 3 ) );
    #endif
    #endif
    }
    else
    {
      SI32_PBSTD_A_write_pins_low( port_std[ LED_PORT ], ( ( u32 ) 1 << 6 ) );
    #if !defined( MEMBRANE_V1)
    #if defined( PCB_V8 )
      SI32_PBSTD_A_write_pins_low( port_std[ LED_PORT ], ( ( u32 ) 1 << 4 ) );
    #else
      SI32_PBHD_A_write_pins_low( SI32_PBHD_4, ( ( u32 ) 1 << 3 ) );
    #endif
    #endif
    }
    if(led_ticks + led_background[2] > led_ticks_ptr[2] && (led_mask & 1<<2 ) )
    {
    #if defined( MEMBRANE_V1)
    #if defined( PCB_V8 )
      SI32_PBSTD_A_write_pins_high( port_std[ LED_PORT ], ( ( u32 ) 1 << 4 ) );
    #else
      SI32_PBHD_A_write_pins_high( SI32_PBHD_4, ( ( u32 ) 1 << 3 ) );
    #endif
    #endif
      SI32_PBSTD_A_write_pins_high( port_std[ LED_PORT ], ( ( u32 ) 1 << 7 ) );
    }
    else
    {
    #if defined( MEMBRANE_V1)
    #if defined( PCB_V8 )
      SI32_PBSTD_A_write_pins_low( port_std[ LED_PORT ], ( ( u32 ) 1 << 4 ) );
    #else
      SI32_PBHD_A_write_pins_low( SI32_PBHD_4, ( ( u32 ) 1 << 3 ) );
    #endif
    #endif
      SI32_PBSTD_A_write_pins_low( port_std[ LED_PORT ], ( ( u32 ) 1 << 7 ) );
    }
    if(led_ticks + led_background[3] > led_ticks_ptr[3] && (led_mask & 1<<3 ) )
      SI32_PBSTD_A_write_pins_high( port_std[ LED_PORT ], ( ( u32 ) 1 << 8 ) );
    else
      SI32_PBSTD_A_write_pins_low( port_std[ LED_PORT ], ( ( u32 ) 1 << 8 ) );
    if(led_ticks + led_background[4] > led_ticks_ptr[4] && (led_mask & 1<<4 ) )
      SI32_PBSTD_A_write_pins_high( port_std[ LED_PORT ], ( ( u32 ) 1 << 9 ) );
    else
      SI32_PBSTD_A_write_pins_low( port_std[ LED_PORT ], ( ( u32 ) 1 << 9 ) );
  }
  // Clear the interrupt flag
  SI32_TIMER_A_clear_high_overflow_interrupt(SI32_TIMER_1);
}

void led_cache_mode(int led, int mode )
{
  u8 rram_led_byte = rram_read_byte(4);
  if( led == LED_COLOR_GPS )
  {
    rram_led_byte &= ~0xF;
    rram_led_byte |= (mode & 0xF);
  }
  if( led == LED_COLOR_SAT )
  {
    rram_led_byte &= ~0xF0;
    rram_led_byte |= ( ( mode << 4 ) & 0xF0);
  }
  rram_write_byte(4, rram_led_byte);
  //printf("CACHING_LED: 0x%x, %d, %d\n", rram_led_byte, led, mode);
}

void led_set_mask( u8 mask )
{
  led_mask = mask;
}

void led_set_background(int led, u8 bkgnd)
{
  if(led > LED_COUNT)
    return;
  if(bkgnd > 15)
    bkgnd = 15;

  led_background[led]  = bkgnd;
}

void led_set_mode(int led, int mode, int cycles)
{
  if(led > LED_COUNT)
    return;
  if(cycles > 255)
    cycles = 255;
  led_pending_mode_ptr[led] = (u8 *)led_cled_ptr[mode];
  led_pending_repeats_ptr[led] = cycles;
}

int led_get_mode(int led)
{
  if(led > LED_COUNT)
    return -1;

  int i;
  if(led_pending_mode_ptr[led] != NULL)
  {
    for(i=0;i<( sizeof( led_cled_ptr ) / sizeof( led_cled_ptr[0] ) );i++)
    {
      if(led_pending_mode_ptr[led] == led_cled_ptr[i])
        return i;
    }
  } else {
    for(i=0;i<( sizeof( led_cled_ptr ) / sizeof( led_cled_ptr[0] ) );i++)
    {
      if(led_mode_ptr[led] == led_cled_ptr[i])
        return i;
    }
  }
  return -1;
}

// SysTick interrupt handler
void SysTick_Handler()
{
  // Handle virtual timers
  cmn_virtual_timer_cb();

  // Handle system timer call
  cmn_systimer_periodic();

  tickSeconds++;
  if(tickSeconds == SYSTICKHZ)
  {
    seconds_tick_pending = 1;
    tickSeconds = 0;
  }
  if(i2c_timeout_timer)
    i2c_timeout_timer--;
}

static void gTIMER0_enter_auto_reload_config(void)
{
  // ENABLE TIMER0 CLOCK
  SI32_CLKCTRL_A_enable_apb_to_modules_0(SI32_CLKCTRL_0,
    SI32_CLKCTRL_A_APBCLKG0_TIMER0CEN_ENABLED_U32);

  // INITIALIZE TIMER
  SI32_TIMER_A_initialize (SI32_TIMER_0, 0x00, 0x00, 0x00, 0x00);
  SI32_TIMER_A_select_single_timer_mode (SI32_TIMER_0);
  SI32_TIMER_A_select_high_clock_source_apb_clock (SI32_TIMER_0);
  SI32_TIMER_A_select_high_auto_reload_mode (SI32_TIMER_0);

  // Set overflow frequency to SYSTICKHZ
  SI32_TIMER_A_write_capture (SI32_TIMER_0, (unsigned) -(cmsis_get_cpu_frequency()/SYSTICKHZ));
  SI32_TIMER_A_write_count (SI32_TIMER_0, (unsigned) -(cmsis_get_cpu_frequency()/SYSTICKHZ));

  // Run Timer
  SI32_TIMER_A_start_high_timer(SI32_TIMER_0);

  // ENABLE INTERRUPTS
  NVIC_ClearPendingIRQ(TIMER0H_IRQn);
  NVIC_EnableIRQ(TIMER0H_IRQn);
  SI32_TIMER_A_enable_high_overflow_interrupt(SI32_TIMER_0);
}

static void gTIMER1_enter_auto_reload_config(void)
{
  // ENABLE TIMER1 CLOCK
  SI32_CLKCTRL_A_enable_apb_to_modules_0(SI32_CLKCTRL_0,
    SI32_CLKCTRL_A_APBCLKG0_TIMER1CEN_ENABLED_U32);

  // INITIALIZE TIMER
  SI32_TIMER_A_initialize (SI32_TIMER_1, 0x00, 0x00, 0x00, 0x00);
  SI32_TIMER_A_select_single_timer_mode (SI32_TIMER_1);
  SI32_TIMER_A_select_high_clock_source_apb_clock (SI32_TIMER_1);
  SI32_TIMER_A_select_high_auto_reload_mode (SI32_TIMER_1);

  // Set overflow frequency to SYSTICKHZ
  SI32_TIMER_A_write_capture (SI32_TIMER_1, (unsigned) -(cmsis_get_cpu_frequency()/LEDTICKHZ));
  SI32_TIMER_A_write_count (SI32_TIMER_1, (unsigned) -(cmsis_get_cpu_frequency()/LEDTICKHZ));

  // Run Timer
  SI32_TIMER_A_start_high_timer(SI32_TIMER_1);

  // ENABLE INTERRUPTS
  NVIC_ClearPendingIRQ(TIMER1H_IRQn);
  NVIC_EnableIRQ(TIMER1H_IRQn);
  SI32_TIMER_A_enable_high_overflow_interrupt(SI32_TIMER_1);
}

// ****************************************************************************
// PIO section


void pios_init( void )
{
#if defined( ELUA_BOARD_GSATMICRO )
  // SI32_PBCFG_A_unlock_ports(SI32_PBCFG_0);

  // // PB0 Setup
  // SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_0, 0x0603);
  // SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_0, 0x1114);
  // SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_0, 0x37FB);

  // // PB1 Setup
  // SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_1, 0x00A1);
  // SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_1, 0xFC1C);

  // // Enable Crossbar0 signals & set properties
  // SI32_PBCFG_A_enable_xbar0h_peripherals(SI32_PBCFG_0,
  //                                        SI32_PBCFG_A_XBAR0H_UART0EN |
  //                                        SI32_PBCFG_A_XBAR0H_UART1EN);
  // SI32_PBCFG_A_enable_xbar0l_peripherals(SI32_PBCFG_0,
  //                                        SI32_PBCFG_A_XBAR0L_USART0EN |
  //                                        SI32_PBCFG_A_XBAR0L_I2C0EN);
  // SI32_PBCFG_A_enable_crossbar_0(SI32_PBCFG_0);

  // // PB2 Setup
  // SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_2, 0x7FFF);

  // // PB3 Setup
  // SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_3, 0x00FF);

  // // Enable Crossbar1 signals & set properties
  // SI32_PBCFG_A_enable_crossbar_1(SI32_PBCFG_0);


  SI32_PBCFG_A_unlock_ports(SI32_PBCFG_0);

  // PB0 Setup
  SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_0, 0x0603);
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_0, 0x1514);
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_0, 0x03F3);

  // PB1 Setup
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_1, 0x03A1);
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_1, 0xFC1C);

#if defined( BLUETOOTH_ENABLE_TDI_DTR )
  SI32_PBSTD_A_set_pins_digital_input( SI32_PBSTD_1, 1 << 4 );
#endif

#if defined( BLUETOOTH_ENABLE_TDO_DSR )
  SI32_PBCFG_A_disable_jtag(SI32_PBCFG_0);
  SI32_PBSTD_A_set_pins_push_pull_output( SI32_PBSTD_1, 1 << 3 );
  SI32_PBSTD_A_write_pins_high(SI32_PBSTD_1, 1 << 3 );
#endif

  // Analog Pins (1.14 & 1.15)
  SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_1, 0xC000);
  SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_1, 0xC000);

#if !defined( PCB_V7 ) && !defined( PCB_V8 )
  SI32_PBSTD_A_write_pins_low(SI32_PBSTD_1, 0x0200 ); //Set 5V regulator off
#endif
  SI32_PBSTD_A_write_pins_high(SI32_PBSTD_1, 0x0100 ); //Set USB high power mode

#if defined( PCB_V7 )
  SI32_PBSTD_A_write_pins_low(SI32_PBSTD_2, 0x03E0 ); //Set external LEDS 0-4 off
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_2, 0x03E0); //Set external LEDS 0-4 as outputs
#endif

#if defined( PCB_V8 )
  SI32_PBSTD_A_write_pins_high(SI32_PBSTD_0, 0x3F0 ); //Set external LEDS 0-4 off
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_0, 0x3F0); //Set external LEDS 0-4 as outputs
#endif

  // Enable Crossbar0 signals & set properties
  SI32_PBCFG_A_enable_xbar0l_peripherals(SI32_PBCFG_0,
                                         SI32_PBCFG_A_XBAR0L_USART0EN |
                                         SI32_PBCFG_A_XBAR0L_USART1EN |
                                         SI32_PBCFG_A_XBAR0L_USART1FCEN |
                                         SI32_PBCFG_A_XBAR0L_I2C0EN);
  SI32_PBCFG_A_enable_xbar0h_peripherals(SI32_PBCFG_0,
                                         SI32_PBCFG_A_XBAR0H_UART0EN |
                                         SI32_PBCFG_A_XBAR0H_UART1EN);
  SI32_PBCFG_A_enable_crossbar_0(SI32_PBCFG_0);

  // PB2 Setup
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_2, 0xFFFF);

  SI32_PBSTD_A_disable_pullup_resistors( SI32_PBSTD_2 );

  // PB3 Setup
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_3, 0xFFFF);

  SI32_PBSTD_A_disable_pullup_resistors( SI32_PBSTD_3 );


  SI32_PBSTD_A_disable_pullup_resistors( SI32_PBSTD_1 );

#if defined( PCB_V7 ) || defined( PCB_V8 )
  //PB3.8 is high voltage dc detection
  //PB3.9 is usb voltage detection
  SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_3, 0x00000300);
  //PB3.11 5V on/off

  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_3, ( u32 ) 1 << 11);
#if defined( PCB_V8 )
  SI32_PBSTD_A_write_pins_low(SI32_PBSTD_3, ( u32 ) 1 << 11 ); //Set 5V regulator off
#else
  SI32_PBSTD_A_write_pins_high(SI32_PBSTD_3, ( u32 ) 1 << 11 ); //Set 5V regulator off
#endif
  // PB2.1 is wakeup
  SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_2, 0x00000002);
#else
  //PB3.6 is external input 1
  //PB3.7 is high voltage dc detection
  //PB3.8 is usb voltage detection
  SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_3, 0x000001C0);
  //PB3.11 is RS232 Force Off
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_3, ( u32 ) 1 << 11);
  SI32_PBSTD_A_write_pins_high(SI32_PBSTD_3, ( u32 ) 1 << 11 );
  //PB0.10 is RS232 VCC
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_0, ( u32 ) 1 << 10);
  SI32_PBSTD_A_write_pins_high(SI32_PBSTD_0, ( u32 ) 1 << 10 );
#endif

  //PB0.1 is external input 2
  //PB0.0 is accel INT1
  SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_0, 0x00000003);
  SI32_PBSTD_A_disable_pullup_resistors( SI32_PBSTD_0 );

  // Enable Crossbar1 signals & set properties
  SI32_PBCFG_A_enable_crossbar_1(SI32_PBCFG_0);


  // Setup PBHD4
  SI32_PBCFG_A_unlock_ports(SI32_PBCFG_0);
  SI32_PBHD_A_write_pblock(SI32_PBHD_4, 0x00);

  SI32_PBHD_A_select_pin0_safe_state(SI32_PBHD_4, 0x0);
  SI32_PBHD_A_select_pin1_safe_state(SI32_PBHD_4, 0x0);
  SI32_PBHD_A_select_pin2_safe_state(SI32_PBHD_4, 0x0);
  SI32_PBHD_A_select_pin3_safe_state(SI32_PBHD_4, 0x0);
  SI32_PBHD_A_select_pin4_safe_state(SI32_PBHD_4, 0x0);
  SI32_PBHD_A_select_pin5_safe_state(SI32_PBHD_4, 0x0);

  SI32_PBHD_A_enable_bias(SI32_PBHD_4);
  SI32_PBHD_A_select_normal_power_port_mode(SI32_PBHD_4);
  SI32_PBHD_A_enable_drivers(SI32_PBHD_4);

  //Setup PB4.0/4.1 to outputs, no pullups, and low for MOSFET outputs
  //Setup PB4.3 LED0/1
  //Setup PB4.4/4.5 GPS
  //Setup PB4.2 to HIGH to turn on mosfets for bat charger!
  SI32_PBHD_A_set_pins_push_pull_output( SI32_PBHD_4, 0x003F );
  SI32_PBHD_A_disable_pullup_resistors( SI32_PBHD_4 );
  SI32_PBHD_A_write_pins_low( SI32_PBHD_4, 0x3B );

  SI32_PBHD_A_set_pins_low_drive_strength(SI32_PBHD_4, 0x3F);

  //SI32_PBHD_A_select_nchannel_current_limit(SI32_PBHD_4, 0xA);
  SI32_PBHD_A_select_pchannel_current_limit(SI32_PBHD_4, 0xF);
  SI32_PBHD_A_enable_pin_current_limit( SI32_PBHD_4, 0x3F );

  SI32_PBHD_A_select_slew_rate(SI32_PBHD_4, SI32_PBHD_A_SLEW_FASTEST);

#if defined( PCB_V7 ) || defined( PCB_V8 )
#if defined( PCB_V7_CHARGER_NPN ) || defined( PCB_V8 )
  //PB4.2 charger disconnect needs to be set high to enable NPN
  SI32_PBHD_A_write_pins_high( SI32_PBHD_4, 0x04 );
#else
  //PB4.2 charger disconnect needs to be set low to enable MOSFET
  SI32_PBHD_A_write_pins_low( SI32_PBHD_4, 0x04 );
#endif
#endif

#if defined( PCB_V7 ) || defined( PCB_V8 )
  // PB4.4 high to power RS232 and I2C
  SI32_PBHD_A_write_pins_high( SI32_PBHD_4, 0x10 );
  // PB4.4 high to power RS232 and I2C
  SI32_PBHD_A_write_pins_high( SI32_PBHD_4, 0x10 );
#endif

  // SI32_PBHD_A_select_pchannel_current_limit(SI32_PBHD_4, 0);
  // SI32_PBHD_A_enable_pin_current_limit( SI32_PBHD_4, 0x3F );

  // SI32_PBHD_A_write_pblock(SI32_PBHD_4, 0x0000);
  // SI32_PBHD_A_enable_drivers(SI32_PBHD_4);
  // SI32_PBHD_A_select_pchannel_current_limit(SI32_PBHD_4, 15);
  // SI32_PBHD_A_select_nchannel_current_limit(SI32_PBHD_4, 15);
  // SI32_PBHD_A_set_pins_push_pull_output(SI32_PBHD_4, 0x003F);
  // SI32_PBHD_A_set_pins_high_drive_strength(SI32_PBHD_4, 0x003F);
  // SI32_PBHD_A_enable_n_channel_drivers(SI32_PBHD_4, 0x003F);
  // SI32_PBHD_A_enable_p_channel_drivers(SI32_PBHD_4, 0x003F);
  // SI32_PBHD_A_enable_pin_current_limit(SI32_PBHD_4, 0x003F);
  //Enable blue LED's if we are on or just in a PM9 temporary sleep...
  if(rram_read_bit(RRAM_BIT_POWEROFF) == POWEROFF_MODE_ACTIVE)
  {
#if defined(  PCB_V7 ) || defined( PCB_V8 )
  #if defined( PCB_V8 )
    SI32_PBSTD_A_write_pins_low( SI32_PBSTD_0, 0x10 );
  #else
    SI32_PBHD_A_write_pins_low( SI32_PBHD_4, 0x08 );
  #endif
#else
  #ifndef USE_EXTERNAL_MOSFETS
      SI32_PBHD_A_write_pins_low( SI32_PBHD_4, 0x02 );
  #endif
      SI32_PBHD_A_write_pins_low( SI32_PBHD_4, 0x08 );
#endif
  }
  else
  {
#if defined( PCB_V7 ) || defined( PCB_V8 )
  #if defined( PCB_V8 )
    SI32_PBSTD_A_write_pins_high( SI32_PBSTD_0, 0x10 );
  #else
    SI32_PBHD_A_write_pins_high( SI32_PBHD_4, 0x08 );
  #endif
#else
  #ifndef USE_EXTERNAL_MOSFETS
      SI32_PBHD_A_write_pins_high( SI32_PBHD_4, 0x02 );
  #endif
      SI32_PBHD_A_write_pins_high( SI32_PBHD_4, 0x08 );
#endif
  }

#else //#if defined( ELUA_BOARD_GSATMICRO )
  // Set up prinf pin
  //SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_1, 0x00000008);

  SI32_PBCFG_A_enable_crossbar_1(SI32_PBCFG_0);
  SI32_PBCFG_A_enable_crossbar_0(SI32_PBCFG_0);

  // ENABLE LED DRIVERS (P2.11, P2.10)
  //SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_2, 0x00000C00);

  // Enable switch sensing (P2.10, P2.11)
  //SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_2, 0x00000300);

  // UART PINS TO PROPER CONFIG (TX = PB1.12, RX = PB1.13)
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_1, 0x0001000);
  SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_1, 0x00002000);
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_0, 0x0000FFFF);
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_1, 0x00000FFF);

  // BRING OUT UART
  SI32_PBCFG_A_enable_xbar0h_peripherals(SI32_PBCFG_0, SI32_PBCFG_A_XBAR0H_UART0EN);
#endif
}


// The platform I/O functions
pio_type platform_pio_op( unsigned port, pio_type pinmask, int op )
{
  pio_type retval = 1;

  switch( op )
  {
    case PLATFORM_IO_PORT_SET_VALUE:
      if( port < 4)
        SI32_PBSTD_A_write_pins_masked( port_std[ port ], 0xFFFF, pinmask);
      else
        SI32_PBHD_A_write_pins_masked( SI32_PBHD_4, 0xFFFF, pinmask);
      break;

    case PLATFORM_IO_PIN_SET:
      if( port < 4)
        SI32_PBSTD_A_write_pins_high( port_std[ port ], pinmask );
      else
        SI32_PBHD_A_write_pins_high( SI32_PBHD_4, pinmask );
      break;

    case PLATFORM_IO_PIN_CLEAR:
      if( port < 4)
        SI32_PBSTD_A_write_pins_low( port_std[ port ], pinmask );
      else
        SI32_PBHD_A_write_pins_low( SI32_PBHD_4, pinmask );
      break;

    case PLATFORM_IO_PORT_DIR_OUTPUT:
      if( port < 4 )
        SI32_PBSTD_A_set_pins_push_pull_output( port_std[ port ], 0xFFFF );
      else
        SI32_PBHD_A_set_pins_push_pull_output( SI32_PBHD_4, 0xFFFF );
        //SI32_PBHD_A_enable_p_channel_drivers( SI32_PBHD_4, 0xFFFF );
      break;

    case PLATFORM_IO_PIN_DIR_OUTPUT:
      if( port < 4 )
        SI32_PBSTD_A_set_pins_push_pull_output( port_std[ port ], pinmask );
      else
        SI32_PBHD_A_set_pins_push_pull_output( SI32_PBHD_4, pinmask );
        //SI32_PBHD_A_enable_p_channel_drivers( SI32_PBHD_4, pinmask );
      break;

    case PLATFORM_IO_PORT_DIR_INPUT:
      if( port < 4 )
        SI32_PBSTD_A_set_pins_digital_input( port_std[ port ], 0xFFFF );
      else
        SI32_PBHD_A_set_pins_digital_input( SI32_PBHD_4, 0xFFFF );
      break;

    case PLATFORM_IO_PIN_DIR_INPUT:
      if( port < 4 )
        SI32_PBSTD_A_set_pins_digital_input( port_std[ port ], pinmask );
      else
        SI32_PBHD_A_set_pins_digital_input( SI32_PBHD_4, pinmask );
      break;

    case PLATFORM_IO_PORT_GET_VALUE:
      if( port < 4 )
        retval = SI32_PBSTD_A_read_pins( port_std[ port ] );
      else
        retval = SI32_PBHD_A_read_pins( SI32_PBHD_4 );
      break;

    case PLATFORM_IO_PIN_GET:
      if( port < 4 )
        retval = ( SI32_PBSTD_A_read_pins(port_std[ port ]) & pinmask ) ? 1 : 0;
      else
        retval = ( SI32_PBHD_A_read_pins( SI32_PBHD_4 ) & pinmask ) ? 1 : 0;
      break;
    case PLATFORM_IO_PIN_PULLDOWN:
      SI32_PBSTD_A_set_pins_analog(port_std[ port ], pinmask);
      break;
    case PLATFORM_IO_PIN_PULLUP:
      SI32_PBSTD_A_enable_pullup_resistors( port_std[ port ] );
      break;
    case PLATFORM_IO_PIN_NOPULL:
      SI32_PBSTD_A_disable_pullup_resistors( port_std[ port ] );
      break;

    default:
      retval = 0;
      break;
  }
  return retval;
}


// ****************************************************************************
// UART section

#ifdef BUILD_USB_CDC
static void sim3_usb_cdc_send( u8 data );
static int sim3_usb_cdc_recv( s32 timeout );
#endif

static SI32_UART_A_Type* const uart[] = { SI32_UART_0, SI32_UART_1 };
static SI32_USART_A_Type* const usart[] = { SI32_USART_0, SI32_USART_1 };

//For DK board, interface is On:
// PB1.12 RX
// PB1.13 TX
// PB1.14 CTS
// PB1.15 RTS

u32 platform_uart_setup( unsigned id, u32 baud, int databits, int parity, int stopbits )
{
  if( id == CDC_UART_ID )
    return 0;

  if( id < 2 )
  {
    SI32_USART_A_enter_full_duplex_mode( uart[ id ] );

    // Set Baud Rate
    // rate = F_APB / ( N * (R/TBAUD + 1 ) )
    SI32_USART_A_set_rx_baudrate( usart[ id ], div_round_closest(cmsis_get_cpu_frequency(), (2 * baud)) - 1);
    SI32_USART_A_set_tx_baudrate( usart[ id ], div_round_closest(cmsis_get_cpu_frequency(), (2 * baud)) - 1);

    // Use Asynchronous Mode
    SI32_USART_A_select_tx_asynchronous_mode ( usart[ id ] );
    SI32_USART_A_select_rx_asynchronous_mode ( usart[ id ] );

    // Set Data Bits
    SI32_USART_A_select_tx_data_length( usart[ id ], databits );
    SI32_USART_A_select_rx_data_length( usart[ id ], databits );


    SI32_USART_A_enable_tx_start_bit( usart[ id ] );
    SI32_USART_A_enable_tx_stop_bit( usart[ id ] );
    SI32_USART_A_enable_rx_start_bit( usart[ id ] );
    SI32_USART_A_enable_rx_stop_bit( usart[ id ] );

    if( stopbits == PLATFORM_UART_STOPBITS_2 )
    {
      SI32_USART_A_select_tx_stop_bits( usart[ id ], SI32_USART_A_STOP_BITS_2_BITS );
      SI32_USART_A_select_rx_stop_bits( usart[ id ], SI32_USART_A_STOP_BITS_2_BITS );
    }
    else
    {
      SI32_USART_A_select_tx_stop_bits( usart[ id ], SI32_USART_A_STOP_BITS_1_BIT );
      SI32_USART_A_select_rx_stop_bits( usart[ id ], SI32_USART_A_STOP_BITS_1_BIT );
    }

    // Set Parity
    switch( parity )
    {
      case PLATFORM_UART_PARITY_NONE:
        SI32_USART_A_disable_tx_parity_bit( usart[ id ] );
        SI32_USART_A_disable_rx_parity_bit( usart[ id ] );
        break;

      case PLATFORM_UART_PARITY_ODD:
        SI32_USART_A_enable_tx_parity_bit( usart[ id ] );
        SI32_USART_A_select_tx_parity( usart[ id ], 0 );
        SI32_USART_A_enable_rx_parity_bit( usart[ id ] );
        SI32_USART_A_select_rx_parity( usart[ id ], 0 );
        break;

      case PLATFORM_UART_PARITY_EVEN:
        SI32_USART_A_enable_tx_parity_bit( usart[ id ] );
        SI32_USART_A_select_tx_parity( usart[ id ], 1 );
        SI32_USART_A_enable_rx_parity_bit( usart[ id ] );
        SI32_USART_A_select_rx_parity( usart[ id ], 1 );
        break;
    }

    SI32_USART_A_disable_tx_signal_inversion( usart[ id ] );
    SI32_USART_A_disable_rx_signal_inversion( usart[ id ] );

    //SI32_USART_A_select_rx_fifo_threshold_2( usart[ id ] );

    // Enable RX & TX
    SI32_USART_A_enable_tx( usart[ id ] );
    SI32_USART_A_enable_rx( usart[ id ] );
  }
  else
  {
    id = id - 2;

    SI32_UART_A_enter_full_duplex_mode( uart[ id ] );

    // Set Baud Rate
    // rate = F_APB / ( N * (R/TBAUD + 1 ) )
    SI32_UART_A_set_rx_baudrate( uart[ id ], div_round_closest(cmsis_get_cpu_frequency(), (2 * baud)) - 1);
    SI32_UART_A_set_tx_baudrate( uart[ id ], div_round_closest(cmsis_get_cpu_frequency(), (2 * baud)) - 1);

    // Use Asynchronous Mode
    //SI32_UART_A_select_tx_asynchronous_mode ( uart[ id ] );
    //SI32_UART_A_select_rx_asynchronous_mode ( uart[ id ] );

    // Set Data Bits
    SI32_UART_A_select_tx_data_length( uart[ id ], databits );
    SI32_UART_A_select_rx_data_length( uart[ id ], databits );


    SI32_UART_A_enable_tx_start_bit( uart[ id ] );
    SI32_UART_A_enable_tx_stop_bit( uart[ id ] );
    SI32_UART_A_enable_rx_start_bit( uart[ id ] );
    SI32_UART_A_enable_rx_stop_bit( uart[ id ] );

    if( stopbits == PLATFORM_UART_STOPBITS_2 )
    {
      SI32_UART_A_select_tx_stop_bits( uart[ id ], SI32_UART_A_STOP_BITS_2_BITS );
      SI32_UART_A_select_rx_stop_bits( uart[ id ], SI32_UART_A_STOP_BITS_2_BITS );
    }
    else
    {
      SI32_UART_A_select_tx_stop_bits( uart[ id ], SI32_UART_A_STOP_BITS_1_BIT );
      SI32_UART_A_select_rx_stop_bits( uart[ id ], SI32_UART_A_STOP_BITS_1_BIT );
    }

    // Set Parity
    switch( parity )
    {
      case PLATFORM_UART_PARITY_NONE:
        SI32_UART_A_disable_tx_parity_bit( uart[ id ] );
        SI32_UART_A_disable_rx_parity_bit( uart[ id ] );
        break;

      case PLATFORM_UART_PARITY_ODD:
        SI32_UART_A_enable_tx_parity_bit( uart[ id ] );
        SI32_UART_A_select_tx_parity( uart[ id ], 0 );
        SI32_UART_A_enable_rx_parity_bit( uart[ id ] );
        SI32_UART_A_select_rx_parity( uart[ id ], 0 );
        break;

      case PLATFORM_UART_PARITY_EVEN:
        SI32_UART_A_enable_tx_parity_bit( uart[ id ] );
        SI32_UART_A_select_tx_parity( uart[ id ], 1 );
        SI32_UART_A_enable_rx_parity_bit( uart[ id ] );
        SI32_UART_A_select_rx_parity( uart[ id ], 1 );
        break;
    }

    SI32_UART_A_disable_tx_signal_inversion( uart[ id ] );
    SI32_UART_A_disable_rx_signal_inversion( uart[ id ] );

    //SI32_UART_A_select_rx_fifo_threshold_2( uart[ id ] );

    // Enable RX & TX
    SI32_UART_A_enable_tx( uart[ id ] );
    SI32_UART_A_enable_rx( uart[ id ] );
  }


  return baud; // FIXME: find a way to actually get baud
}

void platform_s_uart_send( unsigned id, u8 data )
{
#ifdef BUILD_USB_CDC
  if( id == CDC_UART_ID )
    sim3_usb_cdc_send( data );
  else
#endif
  if( id < 2 )
  {
    // Block if the output buffer is full
    while (SI32_USART_A_read_tx_fifo_count(usart[ id ]) >= 4);

    // Write character to the output buffer
    SI32_USART_A_write_data_u8(usart[ id ], data);
  }
  else
  {
    id = id - 2;
    // Block if the output buffer is full
    while (SI32_UART_A_read_tx_fifo_count(uart[ id ]) >= 4);

    // Write character to the output buffer
    SI32_UART_A_write_data_u8(uart[ id ], data);
  }
}

int platform_s_uart_recv( unsigned id, timer_data_type timeout )
{
#ifdef BUILD_USB_CDC
  if( id == CDC_UART_ID )
    return sim3_usb_cdc_recv( timeout );
  else
#endif
  if( id < 2 )
  {
    if( timeout == 0 )
    {
      if ( SI32_USART_A_read_rx_fifo_count( usart[ id ] ) == 0 )
        return -1;
      else
        return ( int )SI32_USART_A_read_data_u8( usart[ id ] );
    }

    // Block if input buffer is empty
    while (SI32_USART_A_read_rx_fifo_count( usart[ id ] ) == 0);

    // Read character from the input buffer
    return ( int )SI32_USART_A_read_data_u8( usart[ id ] );
  }
  else
  {
    id = id - 2;
    if( timeout == 0 )
    {
      if ( SI32_UART_A_read_rx_fifo_count( uart[ id ] ) == 0 )
        return -1;
      else
        return ( int )SI32_UART_A_read_data_u8( uart[ id ] );
    }

    // Block if input buffer is empty
    while (SI32_UART_A_read_rx_fifo_count( uart[ id ] ) == 0);

    // Read character from the input buffer
    return ( int )SI32_UART_A_read_data_u8( uart[ id ] );
  }
}

int platform_s_uart_set_flow_control( unsigned id, int type )
{
  if( id < 2 )
  {
    if( type == PLATFORM_UART_FLOW_NONE )
    {
      SI32_USART_A_disable_cts( usart[ id ] );
      SI32_USART_A_disable_rts( usart[ id ] );
    }
    if( type & PLATFORM_UART_FLOW_CTS )
    {
      SI32_USART_A_enable_cts( usart[ id ] );
    }
    if( type & PLATFORM_UART_FLOW_RTS )
    {
      SI32_USART_A_enable_rts( usart[ id ] );
    }
  }
  else
  {
    id = id - 2;
    if( type == PLATFORM_UART_FLOW_NONE )
    {
      SI32_UART_A_disable_cts( uart[ id ] );
      SI32_UART_A_disable_rts( uart[ id ] );
    }
    if( type & PLATFORM_UART_FLOW_CTS )
    {
      SI32_UART_A_enable_cts( uart[ id ] );
    }
    if( type & PLATFORM_UART_FLOW_RTS )
    {
      SI32_UART_A_enable_rts( uart[ id ] );
    }
  }

  return PLATFORM_OK;
}

// ****************************************************************************
// Timer section


/*
// Helper function: get timer clock
static u32 platform_timer_get_clock( unsigned id )
{
  return 0;
}

// Helper function: set timer clock
static u32 platform_timer_set_clock( unsigned id, u32 clock )
{
  return clock;
}

// Helper function: setup timers
static void platform_setup_timers()
{

}*/

int platform_s_timer_set_match_int( unsigned id, timer_data_type period_us, int type )
{
    return PLATFORM_TIMER_INT_OK;
}

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{

}

timer_data_type platform_s_timer_op( unsigned id, int op, timer_data_type data )
{
  u32 res = 0;

  switch( op )
  {
    case PLATFORM_TIMER_OP_START:
      break;

    case PLATFORM_TIMER_OP_READ:
      break;

    case PLATFORM_TIMER_OP_SET_CLOCK:
      break;

    case PLATFORM_TIMER_OP_GET_CLOCK:
      break;

    case PLATFORM_TIMER_OP_GET_MAX_CNT:
      break;
  }
  return res;
}

u64 platform_timer_sys_raw_read()
{
  return SysTick->LOAD - SysTick->VAL;
}

void platform_timer_sys_disable_int()
{
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

void platform_timer_sys_enable_int()
{
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

timer_data_type platform_timer_read_sys()
{
  return cmn_systimer_get();
}

// *****************************************************************************
// ADC specific functions and variables

#ifdef BUILD_ADC
// PB1.14: ADC1.4
// PB1.15: ADC1.3
// VREGIN / 4: ADC1.17

#define SI32_ADC      SI32_SARADC_1
#define SI32_ADC_IRQ  SARADC1_IRQn

const static u32 adc_ctls[] = { 4, 3, 17 };

int platform_adc_check_timer_id( unsigned id, unsigned timer_id )
{
  //return ( ( timer_id >= ADC_TIMER_FIRST_ID ) && ( timer_id < ( ADC_TIMER_FIRST_ID + ADC_NUM_TIMERS ) ) );
  return 0;
}

void platform_adc_stop( unsigned id )
{
  elua_adc_ch_state *s = adc_get_ch_state( id );
  elua_adc_dev_state *d = adc_get_dev_state( 0 );

  s->op_pending = 0;
  INACTIVATE_CHANNEL(d, id);

  // If there are no more active channels, stop the sequencer
  if( d->ch_active == 0 )
  {
    //SI32_SARADC_A_disable_autoscan( SI32_ADC );
    SI32_SARADC_A_disable_module(SI32_ADC);
    //printf("Stoppit!\n");
    d->running = 0;
  }
}

// Handle ADC interrupts
void SARADC1_IRQHandler( void )
{
  //u32 tmpbuff[ NUM_ADC ];
  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  elua_adc_ch_state *s;

  if ( SI32_SARADC_A_is_scan_done_interrupt_pending( SI32_ADC ) )
  {
    SI32_SARADC_A_clear_single_conversion_complete_interrupt( SI32_ADC );
    SI32_SARADC_A_clear_scan_done_interrupt( SI32_ADC );
    SI32_SARADC_A_disable_autoscan( SI32_ADC );
    SI32_SARADC_A_disable_accumulator( SI32_ADC );

    d->seq_ctr = 0;

    // Update smoothing and/or write to buffer if needed
    while( d->seq_ctr < d->seq_len )
    {
      //printf( "Ctr: %d ", d->seq_ctr );
      s = d->ch_state[ d->seq_ctr ];
      d->sample_buf[ s->id ] = ( u16 )( SI32_SARADC_A_read_data( SI32_ADC ) / 16 );
      //printf("Value: %d\n", d->sample_buf[ d->seq_ctr ] );
      s->value_fresh = 1; // Mark sample as fresh

      // Fill in smoothing buffer until warmed up
      if ( s->logsmoothlen > 0 && s->smooth_ready == 0)
        adc_smooth_data( s->id );

#if defined( BUF_ENABLE_ADC )
      else if ( s->reqsamples > 1 )
      {
        buf_write( BUF_ID_ADC, s->id, ( t_buf_data* )s->value_ptr );
        s->value_fresh = 0;
      }
#endif

      // If we have the number of requested samples, stop sampling
      if ( adc_samples_available( s->id ) >= s->reqsamples && s->freerunning == 0 )
        platform_adc_stop( s->id );

      d->seq_ctr++;
    }
    d->seq_ctr = 0;

    // Only attempt to refresh sequence order if still running
    // This allows us to "cache" an old sequence if all channels
    // finish at the same time
    if ( d->running == 1 )
      adc_update_dev_sequence( 0 );

    if ( d->clocked == 0 && d->running == 1 )
    {
      SI32_SARADC_A_enable_burst_mode(SI32_ADC);
      SI32_SARADC_A_enable_autoscan(SI32_ADC);

      // a 1-to-0 transition on ACCMD bit will enable the accumulator for the next conversion
      SI32_SARADC_A_enable_accumulator(SI32_ADC);
      SI32_SARADC_A_clear_accumulator(SI32_ADC);
      SI32_SARADC_A_start_conversion( SI32_ADC );
    }
  }
  else if ( SI32_SARADC_A_is_single_conversion_complete_interrupt_pending( SI32_ADC ) )
  {
    SI32_SARADC_A_clear_single_conversion_complete_interrupt( SI32_ADC );
    SI32_SARADC_A_enable_burst_mode( SI32_ADC );
    SI32_SARADC_A_start_conversion( SI32_ADC );
    //printf("Single\n");
  }
}

#define ADC_CLK 1000000

static void adcs_init()
{
  unsigned id;
  //elua_adc_dev_state *d = adc_get_dev_state( 0 );
  // Fclk_SAR = (2 x Fapb)/(sar_clk_div +1).
  SI32_SARADC_A_select_sar_clock_divider( SI32_ADC, ( div_round_closest( 2 * cmsis_get_cpu_frequency(), ADC_CLK )  - 1 ) );

  SI32_SARADC_A_select_output_packing_mode_lower_halfword_only( SI32_ADC );

  SI32_SARADC_A_select_start_of_conversion_source( SI32_ADC, SI32_SARADC_A_CONTROL_SCSEL_ADCNT0_VALUE );

  SI32_SARADC_A_select_burst_mode_clock_apb_clock( SI32_ADC );

  // adc will run through one scan of all enabled time sequences once
  SI32_SARADC_A_select_autoscan_mode_once( SI32_ADC );

  SI32_SARADC_A_enable_common_mode_buffer( SI32_ADC );

  SI32_SARADC_A_select_vref_internal( SI32_ADC );

  // Set up characteristic group 0
  SI32_SARADC_A_enter_12bit_mode(SI32_ADC, 0);

  SI32_SARADC_A_select_delayed_track_mode( SI32_ADC );

  SI32_SARADC_A_select_burst_mode_repeat_count(SI32_ADC, 0,
                                              SI32_SARADC_A_BURST_MODE_REPEAT_COUNT_SAMPLE_16_TIMES);

  SI32_SARADC_A_select_channel_character_group_gain_half( SI32_ADC, 0 );
  //SI32_SARADC_A_select_channel_character_group_gain_one( SI32_ADC, 0 );


  for( id = 0; id < NUM_ADC; id ++ )
    adc_init_ch_state( id );

  // Perform sequencer setup
  platform_adc_set_clock( 0, 0 );

  NVIC_ClearPendingIRQ(SI32_ADC_IRQ);
  NVIC_EnableIRQ(SI32_ADC_IRQ);

  SI32_SARADC_A_enable_single_conversion_complete_interrupt(SI32_ADC);
  SI32_SARADC_A_enable_scan_done_interrupt(SI32_ADC);

  // Enable VREGIN ADC source
  SI32_VREG_A_enable_vreg_sense(SI32_VREG_0);
}

u32 platform_adc_set_clock( unsigned id, u32 frequency )
{
  elua_adc_dev_state *d = adc_get_dev_state( 0 );

  // if ( frequency > 0 )
  // {
  //   //d->clocked = 1;
  //   // not yet implemented
  // }
  // else
  // {
    d->clocked = 0;
    // Conversion will run back-to-back until required samples are acquired
    SI32_SARADC_A_select_start_of_conversion_source( SI32_ADC, SI32_SARADC_A_CONTROL_SCSEL_ADCNT0_VALUE );
  // }

  return frequency;
}


int platform_adc_update_sequence( )
{
  elua_adc_dev_state *d = adc_get_dev_state( 0 );

  SI32_SARADC_A_disable_module(SI32_ADC);

  // NOTE: seq ctr should have an incrementer that will wrap appropriately..
  d->seq_ctr = 0;
  while( d->seq_ctr < d->seq_len )
  {

    SI32_SARADC_A_select_timeslot_channel( SI32_ADC, d->seq_ctr,
                                           adc_ctls[ d->ch_state[ d->seq_ctr ]->id ] );

    SI32_SARADC_A_select_timeslot_channel_character_group( SI32_ADC, d->seq_ctr, 0 );
    d->seq_ctr++;
  }
  // Set sequence end
  SI32_SARADC_A_select_timeslot_channel( SI32_ADC, d->seq_ctr, 31 );
  d->seq_ctr = 0;


  // // ENABLE MODULE
  SI32_SARADC_A_enable_module(SI32_ADC);

  return PLATFORM_OK;
}


int platform_adc_start_sequence()
{
  elua_adc_dev_state *d = adc_get_dev_state( 0 );

  if( d->running != 1 )
  {
    adc_update_dev_sequence( 0 );

    SI32_SARADC_A_enable_burst_mode(SI32_ADC);

    // a 0-to-1 transition on the SCANEN bit is required to arm the scan.
    // the scan will not start until a conversion start occurs.
    SI32_SARADC_A_enable_autoscan(SI32_ADC);

    // a 1-to-0 transition on ACCMD bit will enable the accumulator for the next conversion
    SI32_SARADC_A_enable_accumulator(SI32_ADC);
    SI32_SARADC_A_clear_accumulator(SI32_ADC);

    d->running = 1;

    if( d->clocked == 1 )
    {
      // not yet implemented
    }
    else
    {
      SI32_SARADC_A_start_conversion( SI32_ADC );
    }
  }

  return PLATFORM_OK;
}

#endif // ifdef BUILD_ADC

// ****************************************************************************
// I2C support

#if NUM_I2C > 0

//#define DEBUG_I2C

static SI32_I2C_A_Type* const i2cs[] = { SI32_I2C_0, SI32_I2C_1 };

#define  I2C_WRITE          0x00           // I2C WRITE command
#define  I2C_READ           0x01           // I2C READ command

u32 platform_i2c_setup( unsigned id, u32 speed )
{
  SI32_I2C_A_set_scaler_value( i2cs[ id ], div_round_closest( cmsis_get_cpu_frequency(), speed ) );

  // set SETUP time to non-zero value for repeated starts to function correctly
  SI32_I2C_A_set_extended_data_setup_time(SI32_I2C_0, 0x01);

  // ENABLE MODULE
  SI32_I2C_A_enable_module( i2cs[ id ] );

  // Return actual speed
  return cmsis_get_cpu_frequency() / SI32_I2C_A_get_scaler_value( i2cs[ id ] );
}

void platform_i2c_send_start( unsigned id )
{
  // The master write operation starts with firmware setting the STA
  // bit to generate a start condition.

  SI32_I2C_A_set_start( i2cs[ id ] );

  if( SI32_I2C_A_is_tx_interrupt_pending( i2cs[ id ] ) )
    SI32_I2C_A_clear_tx_interrupt ( i2cs[ id ] );

  if( SI32_I2C_A_is_rx_interrupt_pending( i2cs[ id ] ) )
    SI32_I2C_A_clear_rx_interrupt ( i2cs[ id ] );

  i2c_timeout_timer = I2C_TIMEOUT_SYSTICKS;

  while( SI32_I2C_A_is_start_interrupt_pending( i2cs[ id ] ) == 0 &&
         i2c_timeout_timer );

  if( i2c_timeout_timer == 0 )
  {
#if defined( DEBUG_I2C )
    printf("BEG TIMEOUT\n");
#endif
    return;
  }
#if defined( DEBUG_I2C )
  printf("BEG CONTROL = %lx\n",  i2cs[ id ]->CONTROL.U32 );
#endif
}

void platform_i2c_send_stop( unsigned id )
{
  if ( SI32_I2C_A_is_busy( i2cs[ id ] ) )
  {
#if defined( DEBUG_I2C )
    printf("END CONTROL = %lx\n",  i2cs[ id ]->CONTROL.U32 );
#endif

    if( SI32_I2C_A_is_tx_interrupt_pending( i2cs[ id ] ) )
      SI32_I2C_A_clear_tx_interrupt ( i2cs[ id ] );

    if( SI32_I2C_A_is_rx_interrupt_pending( i2cs[ id ] ) )
      SI32_I2C_A_clear_rx_interrupt ( i2cs[ id ] );

    SI32_I2C_A_set_stop( i2cs[ id ] );
#if defined( DEBUG_I2C )
    printf("END CONTROL = %lx\n",  i2cs[ id ]->CONTROL.U32 );
#endif

    i2c_timeout_timer = I2C_TIMEOUT_SYSTICKS;

    while( SI32_I2C_A_is_stop_interrupt_pending( i2cs[ id ] ) == 0 &&
      i2c_timeout_timer );

    if( i2c_timeout_timer == 0 )
      return;

    SI32_I2C_A_clear_stop( i2cs[ id ] );
    SI32_I2C_A_send_nack ( i2cs[ id ] );
    SI32_I2C_A_clear_stop_interrupt( i2cs[ id ] );
#if defined( DEBUG_I2C )
    printf("END CONTROL = %lx\n",  i2cs[ id ]->CONTROL.U32 );
#endif
  }
  else
  {
#if defined( DEBUG_I2C )
    printf("send_stop: not active\n");
#endif
  }
}

int platform_i2c_send_address( unsigned id, u16 address, int direction )
{
  u8 acktmp = 0;
  // The ISR or firmware routine should then clear the start bit (STA),
  // set the targeted slave address and the R/W direction bit in the DATA
  // register, set the byte count, arm the transmission (TXARM = 1), and
  // clear the start interrupt.
  if ( SI32_I2C_A_is_busy( i2cs[ id ] ) )
  {
#if defined( DEBUG_I2C )
    printf("A CONTROL = %lx\n",  i2cs[ id ]->CONTROL.U32 );
#endif

    while( SI32_I2C_A_is_start_interrupt_pending( i2cs[ id ] ) == 0 &&
      i2c_timeout_timer );
    if( i2c_timeout_timer == 0 )
    {
#if defined( DEBUG_I2C )
      printf("I2C TIMEOUT\n");
#endif
      return 0;
    }

    //SI32_I2C_A_set_slave_address_7_bit( i2cs[ id ] );
    SI32_I2C_A_clear_start( i2cs[ id ] );

    SI32_I2C_A_write_data( i2cs[ id ] , ( address << 1 ) | (direction == PLATFORM_I2C_DIRECTION_TRANSMITTER ?  I2C_WRITE : I2C_READ) );
    SI32_I2C_A_set_byte_count( i2cs[ id ] , 1);
    SI32_I2C_A_arm_tx( i2cs[ id ] );

    SI32_I2C_A_clear_tx_interrupt( i2cs[ id ] ); //JEFF TESTING!!
    SI32_I2C_A_clear_ack_interrupt( i2cs[ id ] );
    SI32_I2C_A_clear_start_interrupt( i2cs[ id ] );

    i2c_timeout_timer = I2C_TIMEOUT_SYSTICKS;

    while( SI32_I2C_A_is_tx_interrupt_pending( i2cs[ id ] ) == 0 &&
      i2c_timeout_timer );

    if( i2c_timeout_timer == 0 )
    {
#if defined( DEBUG_I2C )
      printf("I2C TIMEOUT2\n");
#endif
      return 0;
    }

    acktmp = ( u8 )SI32_I2C_A_is_ack_received( i2cs[ id ] );

    SI32_I2C_A_clear_ack_interrupt( i2cs[ id ] );

    if( direction == PLATFORM_I2C_DIRECTION_RECEIVER )
    {
      SI32_I2C_A_clear_tx_interrupt( i2cs[ id ] );
    }
#if defined( DEBUG_I2C )
    printf("A CONTROL = %lx %d\n",  i2cs[ id ]->CONTROL.U32, acktmp );
#endif
  }
  else
  {
    //#if defined( DEBUG_I2C )
    printf("send_addr: not active\n");
    //#endif
  }

  return acktmp;
}

int platform_i2c_send_byte( unsigned id, u8 data )
{
  if ( SI32_I2C_A_is_busy( i2cs[ id ] ) )
  {
#if defined( DEBUG_I2C )
    printf("B CONTROL = %lx\n",  i2cs[ id ]->CONTROL.U32 );
#endif
    u32 tmpdata = ( u32 )data;
    SI32_I2C_A_set_byte_count( i2cs[ id ] , 1 );
    SI32_I2C_A_write_data( i2cs[ id ], tmpdata );
    SI32_I2C_A_arm_tx( i2cs[ id ] );
    SI32_I2C_A_clear_tx_interrupt( i2cs[ id ] );

    i2c_timeout_timer = I2C_TIMEOUT_SYSTICKS;

    while( SI32_I2C_A_is_tx_interrupt_pending( i2cs[ id ] ) == 0 &&
      i2c_timeout_timer );

    if( i2c_timeout_timer == 0 )
      return 0;

    SI32_I2C_A_clear_ack_interrupt( i2cs[ id ] );

#if defined( DEBUG_I2C )
    printf("B CONTROL = %lx\n",  i2cs[ id ]->CONTROL.U32 );
#endif
    if( SI32_I2C_A_is_ack_received(SI32_I2C_0) )
      return 1;
    else
      return 0;
  }
  else
  {
#if defined( DEBUG_I2C )
    printf("send_byte: not active\n");
#endif
    return 0;
  }
}

int platform_i2c_recv_byte( unsigned id, int ack )
{
  u32 tmpdata;
  if ( SI32_I2C_A_is_busy( i2cs[ id ] ) )
  {
#if defined( DEBUG_I2C )
    printf("R CONTROL = %lx\n",  i2cs[ id ]->CONTROL.U32 );
#endif

    SI32_I2C_A_set_byte_count( i2cs[ id ] , 1);
    SI32_I2C_A_arm_rx( i2cs[ id ] );

    if( SI32_I2C_A_is_rx_interrupt_pending( i2cs[ id ] ) )
      SI32_I2C_A_clear_rx_interrupt ( i2cs[ id ] );
    else if( SI32_I2C_A_is_tx_interrupt_pending( i2cs[ id ] ) )
      SI32_I2C_A_clear_tx_interrupt ( i2cs[ id ] );

    i2c_timeout_timer = I2C_TIMEOUT_SYSTICKS;

    while( SI32_I2C_A_is_ack_interrupt_pending( i2cs[ id ] ) == 0 &&
      i2c_timeout_timer );

    if( i2c_timeout_timer == 0 )
      return 0;

    if( ack )
      SI32_I2C_A_send_ack( i2cs[ id ] );
    else
      SI32_I2C_A_send_nack( i2cs[ id ] );

    SI32_I2C_A_clear_ack_interrupt( i2cs[ id ] );

    i2c_timeout_timer = I2C_TIMEOUT_SYSTICKS;

    while( SI32_I2C_A_is_rx_interrupt_pending( i2cs[ id ] ) == 0 &&
      i2c_timeout_timer );

    if( i2c_timeout_timer == 0 )
      return 0;

    tmpdata = SI32_I2C_A_read_data( i2cs[ id ] );
#if defined( DEBUG_I2C )
    printf("Got: %u\n",(u8)tmpdata);
    if( 0xFFFFFF00 & tmpdata )
      printf("GOT MORE THAN ONE BYTE!\n");

    printf("R CONTROL = %lx\n",  i2cs[ id ]->CONTROL.U32 );
#endif

      return ( u8 )tmpdata;
  }
  else
  {
#if defined( DEBUG_I2C )
    printf("recv_byte: not active\n");
#endif
    return 0;
  }
}

#endif // NUM_I2C > 0

// ****************************************************************************
// PMU functions

void sim3_pmu_sleep( unsigned seconds )
{
  #ifdef EXTRA_SLEEP_HOOK
    extras_sleep_hook(seconds);
  #endif

  // GET CURRENT TIMER VALUE INTO SETCAP
  SI32_RTC_A_start_timer_capture(SI32_RTC_0);
  while(SI32_RTC_A_is_timer_capture_in_progress(SI32_RTC_0));

  // SET ALARM FOR now+s
  // RTC running at 16.384Khz so there are 16384 cycles/sec)
  SI32_RTC_A_write_alarm0(SI32_RTC_0, SI32_RTC_A_read_setcap(SI32_RTC_0) + (16384 * seconds));
  SI32_RTC_A_clear_alarm0_interrupt(SI32_RTC_0);

  // Enable RTC alarm interrupt
  SI32_RTC_A_enable_alarm0_interrupt(SI32_RTC_0);

  // Disable crossbar peripheral connections
  //SI32_PBCFG_A_write_xbar1(SI32_PBCFG_0,0x00000000);
  //SI32_PBCFG_A_write_xbar0h(SI32_PBCFG_0,0x00000000);
  //SI32_PBCFG_A_write_xbar0l(SI32_PBCFG_0,0x00000000);

  // Mask low priority interrupts from waking us
  __set_BASEPRI(0x40);

  SI32_DMACTRL_A_disable_module( SI32_DMACTRL_0 );
  SI32_CLKCTRL_A_exit_fast_wake_mode( SI32_CLKCTRL_0 );
  SI32_RSTSRC_A_disable_power_mode_9( SI32_RSTSRC_0 );

  // Switch VREG to low power mode
  SI32_VREG_A_disable_band_gap( SI32_VREG_0 );
  SI32_VREG_A_enter_suspend_mode( SI32_VREG_0 );
  //SI32_VREG_A_enable_vbus_invalid_interrupt( SI32_VREG_0 );

  // Disable VDD Monitor
  SI32_VMON_A_disable_vdd_supply_monitor(SI32_VMON_0);

  // Switch AHB source to LFO oscillator
  SI32_CLKCTRL_A_select_ahb_source_low_frequency_oscillator( SI32_CLKCTRL_0 );

  // Turn off all peripheral clocks
  SI32_CLKCTRL_A_disable_apb_to_all_modules( SI32_CLKCTRL_0 );

  __WFI();

  SI32_CLKCTRL_A_select_ahb_source_low_power_oscillator(SI32_CLKCTRL_0);

  // Allow all interrupts
  __set_BASEPRI(0x00);

  // Re-enable clocks used at startup
  clk_init();
  pios_init();

  SI32_VREG_A_enable_band_gap( SI32_VREG_0 );
  SI32_EXTVREG_A_enable_module( SI32_EXTVREG_0 );
}

void sim3_pmu_reboot( void )
{
  // Stop USB
  SI32_USB_A_reset_module( SI32_USB_0 );
  NVIC_DisableIRQ(USB0_IRQn);
  SI32_USB_A_disable_module(SI32_USB_0);
  SI32_USB_A_disable_internal_pull_up( SI32_USB_0 );
  reset_parameters();

  SI32_RSTSRC_A_generate_software_reset( SI32_RSTSRC_0 );
}

void sim3_pmu_reboot_nodfu( void )
{
  sim3_pmu_pm9(TRICK_TO_REBOOT_WITHOUT_DFU_MODE);
}

void memory_error( void )
{
  sim3_pmu_reboot_nodfu();
}

void myPMU_enter_sleep(void)
{
  // Configure for Sleep
  SI32_PMU_A_clear_wakeup_flags(SI32_PMU_0);
  SI32_RSTSRC_A_enable_power_mode_9(SI32_RSTSRC_0);
  SCB->SCR = SCB_SCR_SLEEPDEEP_Msk; // set SLEEPDEEP
  __set_FAULTMASK(1);
  __WFI();
}
void VREG0_vbus_invalid_handler(void)
{
}
void myPB_enter_off_config()
{
   // all ports hi-z (analog)
  SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_0, 0x0000FFFF);
  SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_1, 0x0000FFFF);
  SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_2, 0x0000FFFF);
  SI32_PBSTD_A_set_pins_analog(SI32_PBSTD_3, 0x0000FFFF);

  SI32_PBCFG_A_unlock_ports(SI32_PBCFG_0);
  SI32_PBHD_A_write_pblock(SI32_PBHD_4, 0x00);

  SI32_PBCFG_A_enable_xbar0h_peripherals(SI32_PBCFG_0, 0x0000);
  SI32_PBCFG_A_enable_xbar0l_peripherals(SI32_PBCFG_0, 0x0000);
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_0, 0xFFFF);
  SI32_PBSTD_A_write_pbskipen(SI32_PBSTD_1, 0xFFFF);
  SI32_PBCFG_A_enable_crossbar_0(SI32_PBCFG_0);
  SI32_PBCFG_A_enable_crossbar_1(SI32_PBCFG_0);

  int i;
  for( i=0; i<4; i++)
  {
    SI32_PBSTD_A_set_pins_analog(port_std[ i ], 0x0000);
       // basePointer->PBMDSEL_CLR = pin_mask;
       // basePointer->PB_SET      = pin_mask;
    SI32_PBSTD_A_write_pbskipen(port_std[ i ], 0xFFFF);
       // basePointer->PBSKIPEN.U32 = pbskipen;
    SI32_PBSTD_A_set_pins_digital_input( port_std[ i ], 0xFFFF);
       // basePointer->PBOUTMD_CLR = pin_mask;
       // basePointer->PB_SET      = pin_mask;
       // basePointer->PBMDSEL_SET = pin_mask;
    SI32_PBSTD_A_disable_pullup_resistors( port_std[ i ] );
       // basePointer->PBDRV_CLR = SI32_PBSTD_A_PBDRV_PBPUEN_MASK;
    SI32_PBSTD_A_write_pins_low( port_std[ i ], 0xFFFF );
       // basePointer->PB_CLR = pin_mask
  }

  //JEFF TEST
  //SI32_PBCFG_A_write_xbar1(SI32_PBCFG_0,0x00000000);
  //SI32_PBCFG_A_write_xbar0h(SI32_PBCFG_0,0x00000000);
  //SI32_PBCFG_A_write_xbar0l(SI32_PBCFG_0,0x00000000);

  //Set I2C pins to analog float...
  SI32_PBSTD_A_set_pins_analog( SI32_PBSTD_0, 0x0000C000);
  SI32_PBSTD_A_write_pins_low( SI32_PBSTD_0, 0xC000 );
#ifdef BLUETOOTH_POWEREDWHILESLEEPING
  //Set bluetooth pins to analog high...
  if( rram_read_bit(RRAM_BIT_STORAGE_MODE) == STORAGE_MODE_DISABLED )
  {
    SI32_PBSTD_A_set_pins_push_pull_output( SI32_PBSTD_1, 0x0080);
    SI32_PBSTD_A_write_pins_high( SI32_PBSTD_1, 0x0080 );
    SI32_PBSTD_A_set_pins_push_pull_output( SI32_PBSTD_0, 1 << 12);
    SI32_PBSTD_A_write_pins_high( SI32_PBSTD_0, 1 << 12 );
#if defined( BLUETOOTH_ENABLE_TDO_DSR )
    SI32_PBSTD_A_set_pins_push_pull_output( SI32_PBSTD_1, 1 << 3);
    SI32_PBSTD_A_write_pins_high(SI32_PBSTD_1, 1 << 3 );
#endif
    SI32_PBSTD_A_set_pins_digital_input( SI32_PBSTD_0, ( (1 << 11) | (1 << 13) ));
  }
#endif

  //Set 5V pin to analog high...
  SI32_PBSTD_A_set_pins_push_pull_output( SI32_PBSTD_1, 0x00000200);
  SI32_PBSTD_A_write_pins_low( SI32_PBSTD_1, 0x0200 );

#if defined( PCB_V7 ) || defined( PCB_V8 )
  SI32_PBSTD_A_set_pins_digital_input( SI32_PBSTD_2, 0x0002 );
  SI32_PBSTD_A_set_pins_digital_input( SI32_PBSTD_3, 1 << 9 );
#endif

  SI32_PBHD_A_set_pins_push_pull_output( SI32_PBHD_4, 0x00 );
  SI32_PBHD_A_set_pins_low_drive_strength(SI32_PBHD_4, 0x3F);
  SI32_PBHD_A_disable_bias( SI32_PBHD_4 );
  SI32_PBHD_A_disable_pin_current_limit( SI32_PBHD_4, 0x3F );
  SI32_PBHD_A_set_pins_digital_input( SI32_PBHD_4, 0x3F );
  SI32_PBHD_A_disable_pullup_resistors( SI32_PBHD_4 );
  SI32_PBHD_A_write_pins_low( SI32_PBHD_4, 0x3F );

#if defined( PCB_V7 ) || defined( PCB_V8 )
#if defined( PCB_V7_CHARGER_NPN ) || defined( PCB_V8 )
  //PB4.2 is set low above which disables the NPN so do nothing
#else
  //PB4.2 Set the disconnect mosfets to high!
  SI32_PBHD_A_write_pins_high( SI32_PBHD_4, 0x04 );
  SI32_PBHD_A_set_pins_analog( SI32_PBHD_4, 0x04 );
#endif

  //JEFF TESTING with R21 shorted and 150uA to VCC
  SI32_PBSTD_A_write_pins_high( SI32_PBSTD_3, 0x0800 );
  SI32_PBSTD_A_set_pins_digital_input( SI32_PBSTD_3, 0x00000800);
#endif
}



void sim3_pmu_pm9( unsigned seconds )
{
  //u8 i;
  led_set_mode(LED_COLOR_PWR, LED_FADEDOWN, 10);

  if( ( seconds != TRICK_TO_REBOOT_WITHOUT_DFU_MODE ) && external_power() &&
      ( ( rram_read_bit(RRAM_BIT_SLEEP_WHEN_POWERED) == SLEEP_WHEN_POWERED_DISABLED ) || usb_power() ) )
  {
    //printf("Unit is powered, no PM9\n");
    wake_reason = WAKE_POWERCONNECTED;
    rram_write_int(RRAM_INT_SLEEPTIME, seconds);
    return;
  }
  if(seconds == TRICK_TO_REBOOT_WITHOUT_DFU_MODE)
    seconds = 1;

  // GET CURRENT TIMER VALUE INTO SETCAP
  //SI32_RTC_A_start_timer_capture(SI32_RTC_0);
  //while(SI32_RTC_A_is_timer_capture_in_progress(SI32_RTC_0));

#ifdef EXTRA_SLEEP_HOOK
    extras_sleep_hook(PIN_CHECK_INTERVAL);
#endif

  // SET ALARM FOR now+s
  // RTC running at 16.384Khz so there are 16384 cycles/sec)
  if(rram_read_bit(RRAM_BIT_STORAGE_MODE) == STORAGE_MODE_ACTIVE)
  {
    //Sleep forever, in storage mode. Power button will wakeup device
    rram_write_int(RRAM_INT_SLEEPTIME,  0);
    SI32_RTC_A_write_alarm0(SI32_RTC_0, 0xFFFFFFFF); //will wakeup in 3 days
  }
  else if( seconds > PIN_CHECK_INTERVAL )
  {
    rram_write_int(RRAM_INT_SLEEPTIME, seconds - PIN_CHECK_INTERVAL);
    SI32_RTC_A_write_alarm0(SI32_RTC_0, /*SI32_RTC_A_read_setcap(SI32_RTC_0) +*/ (16384 * PIN_CHECK_INTERVAL));
  }
  else
  {
    rram_write_int(RRAM_INT_SLEEPTIME, 0);
    SI32_RTC_A_write_alarm0(SI32_RTC_0, /*SI32_RTC_A_read_setcap(SI32_RTC_0) +*/ (16384 * seconds));
  }


  // Sleep if buttons pressed
  // Disable the SysTick timer to prevent these interrupts
  // from waking the core.
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;


  // Stop USB
  SI32_USB_A_reset_module( SI32_USB_0 );
  NVIC_DisableIRQ(USB0_IRQn);
  SI32_USB_A_disable_module(SI32_USB_0);
  SI32_USB_A_disable_internal_pull_up( SI32_USB_0 );

  // Enter Sleep Mode
  myPB_enter_off_config();

  //VMON0_enter_power_9_mode_from_normal_power_mode();
  SI32_VMON_A_disable_vdd_supply_monitor(SI32_VMON_0);

  //LDO0_enter_power_9_mode_from_normal_power_mode();
  SI32_LDO_A_disable_bias(SI32_LDO_0);
  SI32_LDO_A_select_low_bias(SI32_LDO_0);

  //VREG0_enter_power_9_mode_from_normal_power_mode();
  SI32_VREG_A_enter_suspend_mode(SI32_VREG_0);
  SI32_VREG_A_disable_band_gap(SI32_VREG_0);
  RSTSRC0_enter_power_9_mode_from_normal_power_mode();

  //Reset RTC Timer and clear any interrupts
  SI32_RTC_A_write_setcap(SI32_RTC_0, 0x0);
  SI32_RTC_A_start_timer_set(SI32_RTC_0);
  SI32_RTC_A_clear_alarm0_interrupt(SI32_RTC_0);
  SI32_RTC_A_clear_oscillator_fail_flag(SI32_RTC_0);
  while(SI32_RTC_A_is_timer_set_in_progress(SI32_RTC_0));

  //Enable reset off of RTC or Reset Pin
  SI32_PMU_A_write_wakeen(SI32_PMU_0, 0x0); //Clear all wake events
  SI32_PMU_A_enable_reset_pin_wake_event(SI32_PMU_0);
  SI32_PMU_A_enable_rtc0_alarm_wake_event(SI32_PMU_0);

  //Enable WAKE setup
#if defined( PCB_V7 ) || defined( PCB_V8 )
  //PB2.1
  SI32_PMU_A_set_pin_wake_events( SI32_PMU_0, (1 << 4), (1 << 4) );
#else
  //PB3.6
  SI32_PMU_A_set_pin_wake_events( SI32_PMU_0, (1 << 10), (1 << 10) );
#endif
  SI32_RSTSRC_0->RESETEN_SET = SI32_RSTSRC_A_RESETEN_WAKEREN_MASK | 0x0000003; //replace incorrect SI32_PMU_A_enable_pin_wake_reset
  SI32_PMU_A_enable_pin_wake_event( SI32_PMU_0 );
  SI32_PMU_A_enable_pin_wake( SI32_PMU_0 );

  SI32_PMU_A_clear_wakeup_flags( SI32_PMU_0 );
  //>>HERE
  SI32_RTC_A_start_timer(SI32_RTC_0); //Start count down to wakeup
  myPMU_enter_sleep();

  // DISABLE all wakeup sources
  SI32_PMU_A_write_wakeen(SI32_PMU_0, 0x0);

  SI32_RSTSRC_A_enable_power_mode_9(SI32_RSTSRC_0);
  SI32_RSTSRC_A_enable_rtc0_reset_source(SI32_RSTSRC_0);
}

// ****************************************************************************
// PBHD functions

//  SI32_PBHD_A_select_low_power_port_mode(SI32_PBHD_4);  //needs to be high power if VDDHD >3.6v
//  SI32_PBHD_A_select_slew_rate(SI32_PBHD_4, SI32_PBHD_A_SLEW_FASTEST);
//  SI32_PBHD_A_set_pins_low_drive_strength(SI32_PBHD_4, 0x3F);
//  SI32_PBHD_A_enable_drivers(SI32_PBHD_4);

void sim3_pbhd_setbias( unsigned state )
{
  //  SI32_PBHD_A_enable_bias(SI32_PBHD_4)
}

void sim3_pbhd_setdrive( unsigned state )
{

}

void sim3_pbhd_setslew( unsigned state )
{

}

void sim3_pbhd_setdrivestrength( unsigned state, int pin )
{

}

// ****************************************************************************
// Flash access functions

volatile u8 flash_key_mask  = 0x00;
volatile u8 armed_flash_key = 0x00;

u8 flash_erase( u32 address, u8 verify)
{
    u32 wc;
    u32* verify_address;
    // Write the address of the Flash page to WRADDR
    SI32_FLASHCTRL_A_write_wraddr( SI32_FLASHCTRL_0, address );
    // Enter Flash Erase Mode
    SI32_FLASHCTRL_A_enter_flash_erase_mode( SI32_FLASHCTRL_0 );

    // Disable interrupts
    //hw_intp_disable();
    __disable_irq();

    // Unlock the flash interface for a single access
    armed_flash_key = flash_key_mask ^ 0xA4;
    SI32_FLASHCTRL_A_write_flash_key(SI32_FLASHCTRL_0, armed_flash_key);
    armed_flash_key = flash_key_mask ^ 0xF0;
    SI32_FLASHCTRL_A_write_flash_key(SI32_FLASHCTRL_0, armed_flash_key);
    armed_flash_key = 0;

    // Write any value to initiate a page erase.
    SI32_FLASHCTRL_A_write_wrdata(SI32_FLASHCTRL_0, 0xA5);

    // Wait for flash operation to complete
    while (SI32_FLASHCTRL_A_is_flash_busy(SI32_FLASHCTRL_0))
    {
      WDTIMER0_IRQHandler();
    }

    if( verify )
    {

        address &= ~(INTERNAL_FLASH_SECTOR_SIZE - 1); // Round down to nearest even page address
        verify_address = (u32*)address;

        for( wc = INTERNAL_FLASH_SECTOR_SIZE/4; wc != 0; wc-- )
        {
            if ( *verify_address != 0xFFFFFFFF )
                return 1;

            verify_address++;
        }
    }

    //hw_intp_enable();
    __enable_irq();

    return 0;
}


u8 flash_write( u32 address, u32* data, u32 count, u8 verify )
{
    u32* tmpdata = data;
    u32* verify_address;
    u32 wc;

    // Write the address of the Flash page to WRADDR
    SI32_FLASHCTRL_A_write_wraddr( SI32_FLASHCTRL_0, address );
    // Enter flash erase mode
    SI32_FLASHCTRL_A_exit_flash_erase_mode(SI32_FLASHCTRL_0);

    // disable interrupts
    //hw_intp_disable();
    __disable_irq();

    // Unlock flash interface for multiple accesses
    armed_flash_key = flash_key_mask ^ 0xA4;
    SI32_FLASHCTRL_A_write_flash_key(SI32_FLASHCTRL_0, armed_flash_key);
    armed_flash_key = flash_key_mask ^ 0xF3;
    SI32_FLASHCTRL_A_write_flash_key(SI32_FLASHCTRL_0, armed_flash_key);
    armed_flash_key = 0;

    // Write word-sized
    for( wc = count; wc != 0; wc-- )
    {
        SI32_FLASHCTRL_A_write_wrdata( SI32_FLASHCTRL_0, *data );
        SI32_FLASHCTRL_A_write_wrdata( SI32_FLASHCTRL_0, *data >> 16 );
        data++;
        WDTIMER0_IRQHandler();
    }

    // Relock flash interface
    SI32_FLASHCTRL_A_write_flash_key( SI32_FLASHCTRL_0, 0x5A );

    // Wait for flash operation to complete
    while( SI32_FLASHCTRL_A_is_flash_busy(SI32_FLASHCTRL_0 ) )
    {
      WDTIMER0_IRQHandler();
    }

    if( verify )
    {
        verify_address = (u32*)address;

        for( wc = count; wc != 0; wc-- )
        {
            if (*verify_address != *tmpdata++)
            {
                return 1;
            }

            verify_address++;
        }
    }

    // re-enable interrupts
    //hw_intp_enable();
    __enable_irq();

    return 0;
}


u32 platform_s_flash_write( const void *from, u32 toaddr, u32 size )
{
  flash_key_mask = 0x01;
  if( 0 != flash_write( toaddr, ( u32 * )from, (size + (INTERNAL_FLASH_WRITE_UNIT_SIZE - 1))/INTERNAL_FLASH_WRITE_UNIT_SIZE, 1 ) ) // round up size to count of 4-byte words needed
    return 0;
  else
    return size;
}

int platform_flash_erase_sector( u32 sector_id )
{
  flash_key_mask = 0x01;
  return flash_erase( sector_id * INTERNAL_FLASH_SECTOR_SIZE + INTERNAL_FLASH_START_ADDRESS, 1) == 0 ? PLATFORM_OK : PLATFORM_ERR;
}

// ****************************************************************************
// USB functions

#if defined( BUILD_USB_CDC )

unsigned platform_get_console_uart( void )
{
  return CON_UART_ID;
}

void sim3_usb_cdc_send( u8 data )
{
  usb_pcb_t *pcb = usb_pcb_get();

  if (!(pcb->flags & (1<<ENUMERATED)))
  {
      return;
  }
  if(usb_buf_space(EP_1) == 0)
      return;

  if(usb_buf_write(EP_1, (U8)data) == 0)
    usb_poll();//ep_write(EP_1);
}

int sim3_usb_cdc_recv( s32 timeout )
{
  usb_pcb_t *pcb = usb_pcb_get();

  if (!(pcb->flags & (1<<ENUMERATED)))
      return -1;

  if(/*usb_buf_bytes(EP_3)*/pcb->fifo[EP_3].len > 0 )
    return usb_buf_read(EP_3);
  else
    return -1;
}

#endif




// ****************************************************************************
// Platform specific modules go here
// #ifdef PS_LIB_TABLE_NAME


// #define MIN_OPT_LEVEL 2
// #include "lrodefs.h"
// #ifdef ENABLE_PMU
// extern const LUA_REG_TYPE pmu_map[];
// #endif

// // #if defined( EXTRA_LIBS_INCLUDE )
// // #include "extra_libs.h"
// // #endif

// // #if defined( SIM3_EXTRA_LIBS_ROM ) && LUA_OPTIMIZE_MEMORY == 2
// // #define _EXTRAROM( name, openf, table ) extern const LUA_REG_TYPE table[];
// // SIM3_EXTRA_LIBS_ROM;
// // #endif

// const LUA_REG_TYPE platform_map[] =
// {
// //#if LUA_OPTIMIZE_MEMORY > 0
//   //{ LSTRKEY( "pmu" ), LROVAL( pmu_map ) },
// // #if defined(SIM3_EXTRA_LIBS_ROM)
// // #undef _EXTRAROM
// // #define _EXTRAROM( name, openf, table ) { LSTRKEY( #name ), LROVAL( table ) },
// //   SIM3_EXTRA_LIBS_ROM
// // #endif
// // #endif
//   { LNILKEY, LNILVAL }
// };

// LUALIB_API int luaopen_platform( lua_State *L )
// {
// #if LUA_OPTIMIZE_MEMORY > 0
//   return 0;
// #else // #if LUA_OPTIMIZE_MEMORY > 0
//   //Turn on aggressive emergency garbage collection
//   legc_set_mode( L, EGC_ALWAYS, 20);

//   luaL_register( L, PS_LIB_TABLE_NAME, platform_map );

//   // Setup the new tables inside platform table
//   // lua_newtable( L );
//   // luaL_register( L, NULL, pmu_map );
//   // lua_setfield( L, -2, "pmu" );

// #if defined( SIM3_EXTRA_LIBS_ROM )
// #undef _EXTRAROM
/* #define _EXTRAROM( name, openf, table ) \
//   lua_newtable( L ); \
//   luaL_register( L, NULL, table ); \
//   lua_setfield( L, -2, #name ); */
// #endif

//   return 1;
// #endif // #if LUA_OPTIMIZE_MEMORY > 0
// }

// #else // #ifdef ENABLE_PMU

// LUALIB_API int luaopen_platform( lua_State *L )
// {
//   return 0;
// }

// #endif // #ifdef ENABLE_PMU
