// STM32 interrupt support

// Generic headers
#include "platform.h"
#include "platform_conf.h"
#include "elua_int.h"
#include "common.h"

// Platform-specific headers
#include "stm32l4xx_conf.h"

#ifndef VTMR_TIMER_ID
#define VTMR_TIMER_ID         ( -1 )
#endif

// ****************************************************************************
// Interrupt handlers

extern USART_TypeDef *const stm32_usart[];

static void all_usart_irqhandler( int resnum )
{
  //int temp;

  //temp = USART_GetFlagStatus( stm32_usart[ resnum ], USART_FLAG_ORE );
  cmn_int_handler( INT_UART_RX, resnum );
  // if( temp == SET )
  //   for( temp = 0; temp < 10; temp ++ )
  //     platform_s_uart_send( resnum, '@' );
}

void USART1_IRQHandler()
{
  all_usart_irqhandler( 0 );
}

void USART2_IRQHandler()
{
  all_usart_irqhandler( 1 );
}

void USART3_IRQHandler()
{
  all_usart_irqhandler( 2 );
}

void UART4_IRQHandler()
{
  all_usart_irqhandler( 3 );
}

void UART5_IRQHandler()
{
  all_usart_irqhandler( 4 );
}

// ****************************************************************************
// External interrupt handlers

static const u32 exti_line[] = { LL_EXTI_LINE_0, LL_EXTI_LINE_1, LL_EXTI_LINE_2, LL_EXTI_LINE_3, LL_EXTI_LINE_4, LL_EXTI_LINE_5, LL_EXTI_LINE_6, LL_EXTI_LINE_7, LL_EXTI_LINE_8, LL_EXTI_LINE_9, LL_EXTI_LINE_10, LL_EXTI_LINE_11, LL_EXTI_LINE_12, LL_EXTI_LINE_13, LL_EXTI_LINE_14, LL_EXTI_LINE_15 };

static u16 exti_line_to_gpio( u32 line )
{
  return PLATFORM_IO_ENCODE( ( SYSCFG->EXTICR[line >> 0x02] >> (0x04 * ( line & 0x03 ) ) ) & 0x07, line, PLATFORM_IO_ENC_PIN );
}

// Convert a GPIO ID to a EXINT number
static int exint_gpio_to_src( pio_type piodata )
{
  u16 pin = PLATFORM_IO_GET_PIN( piodata );
  return pin;
}

static void all_exti_irqhandler( int line )
{
  u16 v, port, pin;
  
  v = exti_line_to_gpio( line );
  port = PLATFORM_IO_GET_PORT( v );
  pin = PLATFORM_IO_GET_PIN( v );

  if( LL_EXTI_IsEnabledRisingTrig_0_31( exti_line[line] ) && platform_pio_op( port, 1 << pin, PLATFORM_IO_PIN_GET ) )
    cmn_int_handler( INT_GPIO_POSEDGE, v );
  if( LL_EXTI_IsEnabledFallingTrig_0_31( exti_line[line] ) && ( platform_pio_op( port, 1 << pin, PLATFORM_IO_PIN_GET ) == 0 ) )
    cmn_int_handler( INT_GPIO_NEGEDGE, v );

  LL_EXTI_ClearFlag_0_31( exti_line[ line ] );
}

void EXTI0_IRQHandler()
{
  all_exti_irqhandler( 0 );
}

void EXTI1_IRQHandler()
{
  all_exti_irqhandler( 1 );
}

void EXTI2_IRQHandler()
{
  all_exti_irqhandler( 2 );
}

void EXTI3_IRQHandler()
{
  all_exti_irqhandler( 3 );
}

void EXTI4_IRQHandler()
{
  all_exti_irqhandler( 4 );
}

void EXTI9_5_IRQHandler()
{
  int i;
  for( i = 5; i < 10; i++ )
  {
    if( LL_EXTI_IsActiveFlag_0_31( exti_line[ i ] ) )
      all_exti_irqhandler( i );
  }
}

void EXTI15_10_IRQHandler()
{
  int i;
  for( i = 10; i < 16; i++ )
  {
    if( LL_EXTI_IsActiveFlag_0_31( exti_line[ i ] ) )
      all_exti_irqhandler( i );
  }
}

// ----------------------------------------------------------------------------
// Timer interrupt handlers

extern const TIM_TypeDef * const timer[];

extern u8 stm32_timer_int_periodic_flag[ NUM_PHYS_TIMER ];

static void tmr_int_handler( unsigned id )
{
  TIM_TypeDef *base = ( TIM_TypeDef* )timer[ id ];

  if (LL_TIM_IsActiveFlag_CC1( base ) )
  {
    LL_TIM_ClearFlag_CC1( base );

    if( id == VTMR_TIMER_ID )
      cmn_virtual_timer_cb();
    else
      cmn_int_handler( INT_TMR_MATCH, id );

    if( stm32_timer_int_periodic_flag[ id ] != PLATFORM_TIMER_INT_CYCLIC )
      LL_TIM_DisableIT_CC1( base );
  }
}


void TIM1_CC_IRQHandler(void)
{
  tmr_int_handler( 0 );
}

void TIM2_IRQHandler(void)
{
  tmr_int_handler( 1 );
}

void TIM3_IRQHandler(void)
{
  tmr_int_handler( 2 );
}

void TIM4_IRQHandler(void)
{
  tmr_int_handler( 3 );
}

void TIM5_IRQHandler(void)
{
  tmr_int_handler( 4 );
}

void TIM8_CC_IRQHandler(void)
{
  tmr_int_handler( 7 );
}

// ****************************************************************************
// GPIO helper functions

static int gpioh_get_int_status( elua_int_id id, elua_int_resnum resnum )
{
  return LL_EXTI_IsEnabledIT_0_31(exti_line[ exint_gpio_to_src( resnum ) ]);
}

static int gpioh_set_int_status( elua_int_id id, elua_int_resnum resnum, int status )
{
  int prev = gpioh_get_int_status( id, resnum );
//  u32 mask = 1 << exint_gpio_to_src( resnum );
  LL_EXTI_InitTypeDef exti_init_struct;
  
  if( status == PLATFORM_CPU_ENABLE )
  {
    // Configure port for interrupt line
    //GPIO_EXTILineConfig( PLATFORM_IO_GET_PORT( resnum ), PLATFORM_IO_GET_PIN( resnum ) );

    LL_EXTI_StructInit(&exti_init_struct);
    exti_init_struct.Line_0_31 = exti_line[ exint_gpio_to_src( resnum ) ];
    exti_init_struct.Mode = LL_EXTI_MODE_IT;
    if( ( LL_EXTI_IsEnabledRisingTrig_0_31(exti_line[ exint_gpio_to_src( resnum ) ]) && ( id == INT_GPIO_NEGEDGE) ) ||
      ( LL_EXTI_IsEnabledFallingTrig_0_31(exti_line[ exint_gpio_to_src( resnum ) ]) && ( id == INT_GPIO_POSEDGE ) ) )
      exti_init_struct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    else
      exti_init_struct.Trigger = id == INT_GPIO_POSEDGE ? LL_EXTI_TRIGGER_RISING : LL_EXTI_TRIGGER_FALLING;
    exti_init_struct.LineCommand = ENABLE;
    LL_EXTI_Init(&exti_init_struct);

    LL_EXTI_ClearFlag_0_31( exti_line[ exint_gpio_to_src( resnum ) ] );
  }
  else
  {
    //Disable edge
    if( id == INT_GPIO_POSEDGE )
      LL_EXTI_DisableRisingTrig_0_31(exti_line[ exint_gpio_to_src( resnum ) ]);
    else
      LL_EXTI_DisableFallingTrig_0_31(exti_line[ exint_gpio_to_src( resnum ) ]);
    
    //If no edges enabled, disable line interrupt
    if( (LL_EXTI_IsEnabledRisingTrig_0_31(exti_line[ exint_gpio_to_src( resnum ) ]) == 0) &&
        (LL_EXTI_IsEnabledFallingTrig_0_31(exti_line[ exint_gpio_to_src( resnum ) ]) == 0))
      LL_EXTI_DisableIT_0_31(exti_line[ exint_gpio_to_src( resnum ) ]);
  }
  return prev;
}

static int gpioh_get_int_flag( elua_int_id id, elua_int_resnum resnum, int clear )
{
  int flag = 0;
//  u32 mask =  1 << exint_gpio_to_src( resnum );

  if( LL_EXTI_IsActiveFlag_0_31( exti_line[ exint_gpio_to_src( resnum ) ] ) )
  {
    if( id == INT_GPIO_POSEDGE )
      flag = LL_EXTI_IsEnabledRisingTrig_0_31(exti_line[ exint_gpio_to_src( resnum ) ]);
    else
      flag = LL_EXTI_IsEnabledFallingTrig_0_31(exti_line[ exint_gpio_to_src( resnum ) ]);
  }
  if( flag && clear )
    LL_EXTI_ClearFlag_0_31( exti_line[ exint_gpio_to_src( resnum ) ] );
  return flag;
}

// ****************************************************************************
// Interrupt: INT_GPIO_POSEDGE

static int int_gpio_posedge_set_status( elua_int_resnum resnum, int status )
{
  return gpioh_set_int_status( INT_GPIO_POSEDGE, resnum, status );
}

static int int_gpio_posedge_get_status( elua_int_resnum resnum )
{
  return gpioh_get_int_status( INT_GPIO_POSEDGE, resnum );
}

static int int_gpio_posedge_get_flag( elua_int_resnum resnum, int clear )
{
  return gpioh_get_int_flag( INT_GPIO_POSEDGE, resnum, clear );
}

// ****************************************************************************
// Interrupt: INT_GPIO_NEGEDGE

static int int_gpio_negedge_set_status( elua_int_resnum resnum, int status )
{
  return gpioh_set_int_status( INT_GPIO_NEGEDGE, resnum, status );
}

static int int_gpio_negedge_get_status( elua_int_resnum resnum )
{
  return gpioh_get_int_status( INT_GPIO_NEGEDGE, resnum );
}

static int int_gpio_negedge_get_flag( elua_int_resnum resnum, int clear )
{
  return gpioh_get_int_flag( INT_GPIO_NEGEDGE, resnum, clear );
}

// ****************************************************************************
// Interrupt: INT_TMR_MATCH

static int int_tmr_match_get_status( elua_int_resnum resnum )
{
  TIM_TypeDef *base = ( TIM_TypeDef* )timer[ resnum ];

  return LL_TIM_IsEnabledIT_CC1( base );
}

static int int_tmr_match_set_status( elua_int_resnum resnum, int status )
{
  int previous = int_tmr_match_get_status( resnum );
  TIM_TypeDef *base = ( TIM_TypeDef* )timer[ resnum ];
  if(status == PLATFORM_CPU_ENABLE)
    LL_TIM_EnableIT_CC1(base);
  else
    LL_TIM_DisableIT_CC1(base);
  return previous;
}

static int int_tmr_match_get_flag( elua_int_resnum resnum, int clear )
{
  TIM_TypeDef *base = ( TIM_TypeDef* )timer[ resnum ];
  int status = LL_TIM_IsActiveFlag_CC1( base );

  if( clear )
    LL_TIM_ClearFlag_CC1( base );
  return status;
}

// ****************************************************************************
// Interrupt: INT_UART_RX

static int int_uart_rx_get_status( elua_int_resnum resnum )
{
  return LL_USART_IsEnabledIT_RXNE( stm32_usart[ resnum ] ) ? 1 : 0;
}

static int int_uart_rx_set_status( elua_int_resnum resnum, int status )
{
  int prev = int_uart_rx_get_status( resnum );
  if(status == PLATFORM_CPU_ENABLE)
    LL_USART_EnableIT_RXNE(stm32_usart[ resnum ]);
  else
    LL_USART_DisableIT_RXNE(stm32_usart[ resnum ]);
  return prev;
}

static int int_uart_rx_get_flag( elua_int_resnum resnum, int clear )
{
  int status = LL_USART_IsActiveFlag_RXNE( stm32_usart[ resnum ] ) ? 1 : 0;
  if( clear )
    LL_USART_DisableIT_RXNE( stm32_usart[ resnum ] );
  return status;
}

// ****************************************************************************
// Initialize interrupt subsystem

// UART IRQ table
#if defined( STM32F10X_LD )
static const u8 uart_irq_table[] = { USART1_IRQn, USART2_IRQn };
#elif defined( STM32F10X_MD )
static const u8 uart_irq_table[] = { USART1_IRQn, USART2_IRQn, USART3_IRQn };
#else // high density devices
static const u8 uart_irq_table[] = { USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn, UART5_IRQn };
#endif

// EXTI IRQ table
static const u8 exti_irq_table[] = { EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn };

// EXTI IRQ table
#if defined( STM32F10X_LD )
static const u8 timer_irq_table[] = { TIM1_CC_IRQn, TIM2_IRQn, TIM3_IRQn };
#elif defined( STM32F10X_MD )
static const u8 timer_irq_table[] = { TIM1_CC_IRQn, TIM2_IRQn, TIM3_IRQn, TIM4_IRQn };
#else
static const u8 timer_irq_table[] = { TIM1_CC_IRQn, TIM2_IRQn, TIM3_IRQn, TIM4_IRQn, TIM5_IRQn };
#endif

void platform_int_init()
{
  NVIC_InitTypeDef nvic_init_structure;
  unsigned i;
  
  // Enable all USART interrupts in the NVIC
  nvic_init_structure.NVIC_IRQChannelPreemptionPriority = 0;
  nvic_init_structure.NVIC_IRQChannelSubPriority = 0;
  nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;

  for( i = 0; i < sizeof( uart_irq_table ) / sizeof( u8 ); i ++ )
  {
    nvic_init_structure.NVIC_IRQChannel = uart_irq_table[ i ];
    NVIC_Init( &nvic_init_structure );
  }

  // Enable all EXTI interrupts in the NVIC
  for( i = 0; i < sizeof( exti_irq_table ) / sizeof( u8 ); i ++ )
  {
    nvic_init_structure.NVIC_IRQChannel = exti_irq_table[ i ];
    NVIC_Init( &nvic_init_structure );
  }

#ifdef INT_TMR_MATCH
  for( i = 0; i < sizeof( timer_irq_table ) / sizeof( u8 ); i ++ )
  {
    nvic_init_structure.NVIC_IRQChannel = timer_irq_table[ i ];
      nvic_init_structure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init( &nvic_init_structure );
  }
#endif  

}

// ****************************************************************************
// Interrupt table
// Must have a 1-to-1 correspondence with the interrupt enum in platform_ints.h!

const elua_int_descriptor elua_int_table[ INT_ELUA_LAST ] = 
{
  { int_gpio_posedge_set_status, int_gpio_posedge_get_status, int_gpio_posedge_get_flag },
  { int_gpio_negedge_set_status, int_gpio_negedge_get_status, int_gpio_negedge_get_flag },
  { int_tmr_match_set_status, int_tmr_match_get_status, int_tmr_match_get_flag },
  { int_uart_rx_set_status, int_uart_rx_get_status, int_uart_rx_get_flag }  
};
