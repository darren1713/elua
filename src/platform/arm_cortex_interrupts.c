// Generic functions for ARM CPUs

#include "arm_constants.h"
#include "cortex_constants.h"
#include "type.h"
#include "platform.h"

extern void arm_enable_ints( void );
extern void arm_disable_ints( void );
extern u32 arm_get_int_status( void );

static int critical_interrupts_enabled = 0;
static int state_saved = 0;

void platform_s_cpu_enter_critical_section( void )
{
  const int interrupt_state = platform_cpu_get_global_interrupts();
  arm_disable_ints();
  if( state_saved )
  {
    return;
  }
  critical_interrupts_enabled = interrupt_state;
  state_saved = 1;
}

void platform_s_cpu_exit_critical_section( void )
{
  state_saved = 0;

  // Restore the IRQs to their state prior to entering the critical section
  if( critical_interrupts_enabled )
  {
    arm_enable_ints();
  }
}

int platform_cpu_set_global_interrupts( int status )
{
  u32 crt_status = arm_get_int_status();

  if( status == PLATFORM_CPU_ENABLE )
    arm_enable_ints();
  else
    arm_disable_ints();
  return ( crt_status & INTERRUPT_MASK_BIT ) == INTERRUPT_ACTIVE;
}

int platform_cpu_get_global_interrupts( void )
{
  return ( arm_get_int_status() & INTERRUPT_MASK_BIT ) == INTERRUPT_ACTIVE;
}
