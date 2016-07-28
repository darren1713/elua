// Platform-dependent functions

#include "platform.h"
#include "type.h"
#include "devman.h"
#include "genstd.h"
#include <reent.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "uip_arp.h"
#include "elua_uip.h"
#include "elua_adc.h"
#include "uip-conf.h"
#include "platform_conf.h"
#include "diskio.h"
#include "common.h"
#include "buf.h"
#include "utils.h"
#include "lua.h"
#include "lauxlib.h"
#include "lrotable.h"

// Platform specific includes
#include "stm32l4xx_conf.h"
#include "pll_config.h"
#ifdef BUILD_USB_CDC
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#endif

#if defined( ELUA_BOARD_INTERNAL_CLOCK_HZ )
#define HCLK        ( (HSI_VALUE / PLL_M) * PLL_N / PLL_P)
#else
#define HCLK        ( (HSE_VALUE / PLL_M) * PLL_N / PLL_P)
#endif

#if defined( FORSTM32F411RE ) || defined( FORSTM32F401RE )
#define PCLK1_DIV   2
#define PCLK2_DIV   1
#else
#define PCLK1_DIV   4
#define PCLK2_DIV   2
#endif

// SysTick Config Data
// NOTE: when using virtual timers, SYSTICKHZ and VTMR_FREQ_HZ should have the
// same value, as they're served by the same timer (the systick)
// Max SysTick preload value is 16777215, for STM32F103RET6 @ 72 MHz, lowest acceptable rate would be about 5 Hz
#define SYSTICKHZ               20
#define SYSTICKMS               (1000 / SYSTICKHZ)

#if ( (HCLK / SYSTICKHZ)  > SysTick_LOAD_RELOAD_Msk)
#error  "Sys tick reload value out of range"
#endif

//#define WATCHDOG_ENABLE
#define WATCH_COUNTER_RESET     127

// ****************************************************************************
// Platform initialization

// forward dcls
static void NVIC_Configuration(void);

#ifdef BUILD_USB_CDC
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;
#endif

static void timers_init();
static void pwms_init();
static void uarts_init();
static void spis_init();
static void pios_init();
#ifdef BUILD_ADC
static void adcs_init();
#endif
#if (NUM_CAN > 0)
static void cans_init();
#endif


int platform_init()
{
  // Setup IRQ's
  NVIC_Configuration();

  // Setup PIO
  pios_init();

  // Setup UARTs
  uarts_init();

  // Setup SPIs
  spis_init();

  // Setup timers
  timers_init();

  // Setup PWMs
  pwms_init();

#ifdef BUILD_ADC
  // Setup ADCs
  adcs_init();
#endif

#if (NUM_CAN > 0)
  // Setup CANs
  cans_init();
#endif

  // Setup system timer
  cmn_systimer_set_base_freq( HCLK );
  cmn_systimer_set_interrupt_freq( SYSTICKHZ );

  // Enable SysTick
  if ( SysTick_Config( HCLK / SYSTICKHZ ) )
  {
    /* Capture error */
    while (1);
  }

#if defined( WATCHDOG_ENABLE )
  // Enable Watchdog
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);
  WWDG_SetPrescaler(WWDG_Prescaler_8);
  WWDG_SetWindowValue( WATCH_COUNTER_RESET );
  WWDG_Enable( WATCH_COUNTER_RESET );
#endif

#ifdef BUILD_WOFS
  // Flash initialization (for WOFS)
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

  FLASH_Unlock();
#endif

#ifdef BUILD_USB_CDC
  #ifdef USE_USB_OTG_HS
    USBD_Init( &USB_OTG_dev, USB_OTG_HS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb );
  #else
    USBD_Init( &USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb );
  #endif
    DCD_DevDisconnect( &USB_OTG_dev );
    DCD_DevConnect( &USB_OTG_dev );
#endif

  cmn_platform_init();

  // All done
  return PLATFORM_OK;
}

// ****************************************************************************
// NVIC
// Shared by all STM32 devices.

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/* This struct is used for later reconfiguration of ADC interrupt */
NVIC_InitTypeDef nvic_init_structure_adc;

static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef nvic_init_structure;

#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

  /* Configure the NVIC Preemption Priority Bits */
  /* Priority group 0 disables interrupt nesting completely */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  // Lower the priority of the SysTick interrupt to let the
  // UART interrupt preempt it
  nvic_init_structure.NVIC_IRQChannel = SysTick_IRQn;
  nvic_init_structure.NVIC_IRQChannelPreemptionPriority = 0;
  nvic_init_structure.NVIC_IRQChannelSubPriority = 1;
  nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init_structure);

#ifdef BUILD_ADC
  nvic_init_structure_adc.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  nvic_init_structure_adc.NVIC_IRQChannelPreemptionPriority = 0;
  nvic_init_structure_adc.NVIC_IRQChannelSubPriority = 2;
  nvic_init_structure_adc.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&nvic_init_structure_adc);
#endif
}

// ****************************************************************************
// PIO
// This is pretty much common code to all STM32 devices.
// todo: Needs updates to support different processor lines.
GPIO_TypeDef * const pio_port[] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH };
static const u32 pio_port_clk[]        = { LL_AHB2_GRP1_PERIPH_GPIOA , LL_AHB2_GRP1_PERIPH_GPIOB
         , LL_AHB2_GRP1_PERIPH_GPIOC , LL_AHB2_GRP1_PERIPH_GPIOD , LL_AHB2_GRP1_PERIPH_GPIOE
         , LL_AHB2_GRP1_PERIPH_GPIOF , LL_AHB2_GRP1_PERIPH_GPIOG , LL_AHB2_GRP1_PERIPH_GPIOH };

static void pios_init()
{
  LL_GPIO_InitTypeDef GPIO_InitStructure;
  int port;

  for( port = 0; port < NUM_PIO; port++ )
  {
    // Enable clock to port.
    LL_AHB2_GRP1_EnableClock(pio_port_clk[port]);

    // Default all port pins to input and enable port.
    LL_GPIO_StructInit(&GPIO_InitStructure);
  
    //Protect JTAG pins from getting disconnected. In circuit debugging stops working if not skipped
    if(pio_port[port] == GPIOA)
      GPIO_InitStructure.Pin &= ~(LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);
    if(pio_port[port] == GPIOB)
      GPIO_InitStructure.Pin &= ~(LL_GPIO_PIN_3 | LL_GPIO_PIN_4);
  
    LL_GPIO_Init(pio_port[port], &GPIO_InitStructure);
  }

#if defined(ENABLE_JTAG_SWD) || defined(ENABLE_TRACE)
  //Mapping JTAG / SWD pins
  //LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_AF_4,  GPIO_AF0_SWJ); // PB4  TRST
  //LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_AF_3,  GPIO_AF0_SWJ); // PB3  TDO   / SWO

  //LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_AF_13, GPIO_AF0_SWJ); // PA13 TMS   / SWDIO
  //LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_AF_14, GPIO_AF0_SWJ); // PA14 TCK   / SWDCLK
  //LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_AF_15, GPIO_AF0_SWJ); // PA15 TDI

  GPIO_InitStructure.Pin = LL_GPIO_PIN_13 | LL_GPIO_PIN_15;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStructure.Alternate = GPIO_AF0_SWJ;
  LL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = LL_GPIO_PIN_14;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOA, &GPIO_InitStructure);

#endif

#ifdef ENABLE_TRACE
  //Mapping TRACE pins, PE2,3,4,5,6
  GPIO_InitStructure.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
  LL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  LL_GPIO_SetAFPin_0_7(GPIOE, LL_GPIO_AF_2, GPIO_AF_TRACE);
  LL_GPIO_SetAFPin_0_7(GPIOE, LL_GPIO_AF_3, GPIO_AF_TRACE);
  LL_GPIO_SetAFPin_0_7(GPIOE, LL_GPIO_AF_4, GPIO_AF_TRACE);
  LL_GPIO_SetAFPin_0_7(GPIOE, LL_GPIO_AF_5, GPIO_AF_TRACE);
  LL_GPIO_SetAFPin_0_7(GPIOE, LL_GPIO_AF_6, GPIO_AF_TRACE);
#endif

  LL_GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.Pin   = LL_GPIO_PIN_5;
  GPIO_InitStructure.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  LL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

pio_type platform_pio_op( unsigned port, pio_type pinmask, int op )
{
  pio_type retval = 1;
  LL_GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_TypeDef * base = pio_port[ port ];

  switch( op )
  {
    case PLATFORM_IO_PORT_SET_VALUE:
      LL_GPIO_WriteOutputPort(base, pinmask);
      break;

    case PLATFORM_IO_PIN_SET:
      LL_GPIO_SetOutputPin(base, pinmask);
      break;

    case PLATFORM_IO_PIN_CLEAR:
      LL_GPIO_ResetOutputPin(base, pinmask);
      break;

    case PLATFORM_IO_PORT_DIR_INPUT:
      pinmask = LL_GPIO_PIN_ALL;
    case PLATFORM_IO_PIN_DIR_INPUT:
      LL_GPIO_StructInit(&GPIO_InitStructure);
      GPIO_InitStructure.Pin  = pinmask;
      GPIO_InitStructure.Mode = LL_GPIO_MODE_INPUT;

      LL_GPIO_Init(base, &GPIO_InitStructure);
      break;

    case PLATFORM_IO_PORT_DIR_OUTPUT:
      pinmask = LL_GPIO_PIN_ALL;
    case PLATFORM_IO_PIN_DIR_OUTPUT:
      LL_GPIO_StructInit(&GPIO_InitStructure);
      GPIO_InitStructure.Pin   = pinmask;
      GPIO_InitStructure.Mode  = LL_GPIO_MODE_OUTPUT;
      GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;

      LL_GPIO_Init(base, &GPIO_InitStructure);
      break;

    case PLATFORM_IO_PORT_GET_VALUE:
      retval = pinmask == PLATFORM_IO_READ_IN_MASK ? LL_GPIO_ReadInputPort(base) : LL_GPIO_ReadOutputPort(base);
      break;

    case PLATFORM_IO_PIN_GET:
      retval = LL_GPIO_IsInputPinSet(base, pinmask);
      break;

    case PLATFORM_IO_PIN_PULLUP:
      LL_GPIO_StructInit(&GPIO_InitStructure);
      GPIO_InitStructure.Pin   = pinmask;
      GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;

      LL_GPIO_Init(base, &GPIO_InitStructure);
      break;

    case PLATFORM_IO_PIN_PULLDOWN:
      LL_GPIO_StructInit(&GPIO_InitStructure);
      GPIO_InitStructure.Pin   = pinmask;
      GPIO_InitStructure.Pull  = LL_GPIO_PULL_DOWN;

      LL_GPIO_Init(base, &GPIO_InitStructure);
      break;

    case PLATFORM_IO_PIN_NOPULL:
      LL_GPIO_StructInit(&GPIO_InitStructure);
      GPIO_InitStructure.Pin   = pinmask;
      GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;

      LL_GPIO_Init(base, &GPIO_InitStructure);
      break;

    default:
      retval = 0;
      break;
  }
  return retval;
}

// ****************************************************************************
// SPI
// NOTE: Only configuring 2 SPI peripherals, since the third one shares pins with JTAG

static SPI_TypeDef *const spi[]  = { SPI1, SPI2, SPI3 };
static const u8 spi_AF[]  = { GPIO_AF5_SPI1, GPIO_AF5_SPI2, GPIO_AF6_SPI3 };

static const u16 spi_prescaler[] = { LL_SPI_BAUDRATEPRESCALER_DIV2, LL_SPI_BAUDRATEPRESCALER_DIV4, LL_SPI_BAUDRATEPRESCALER_DIV8,
                                     LL_SPI_BAUDRATEPRESCALER_DIV16, LL_SPI_BAUDRATEPRESCALER_DIV32, LL_SPI_BAUDRATEPRESCALER_DIV64,
                                     LL_SPI_BAUDRATEPRESCALER_DIV128, LL_SPI_BAUDRATEPRESCALER_DIV256 };

static const u8 spi_gpio_pins_source[][3] = {
  //SCK,    MISO,   MOSI
  {LL_GPIO_AF_5  , LL_GPIO_AF_6  , LL_GPIO_AF_7},
  {LL_GPIO_AF_13 , LL_GPIO_AF_14 , LL_GPIO_AF_15},
  {LL_GPIO_AF_10 , LL_GPIO_AF_11 , LL_GPIO_AF_12}
};

static const u16 spi_gpio_pins[] = { LL_GPIO_PIN_5  | LL_GPIO_PIN_6  | LL_GPIO_PIN_7,
                                     LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
                                     LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12,
                                   };
//                                   SCK           MISO          MOSI
static GPIO_TypeDef *const spi_gpio_port[] = { GPIOA, GPIOB, GPIOC };

static void spis_init()
{
  // Enable Clocks
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
}

#define SPI_GET_BASE_CLK( id ) ( ( id ) == 0 ? ( HCLK / PCLK2_DIV ) : ( HCLK / PCLK1_DIV ) )

u32 platform_spi_setup( unsigned id, int mode, u32 clock, unsigned cpol, unsigned cpha, unsigned databits )
{
  LL_SPI_InitTypeDef SPI_InitStructure;
  LL_GPIO_InitTypeDef GPIO_InitStructure;
  LL_SPI_StructInit(&SPI_InitStructure);
  u8 prescaler_idx = intlog2( ( unsigned ) ( SPI_GET_BASE_CLK( id ) / clock ) );
  //int i;

  if ( prescaler_idx < 0 )
    prescaler_idx = 0;
  if ( prescaler_idx > 7 )
    prescaler_idx = 7;

  //Connect pin to SPI
  //for(i = 0; i < 3; i++)
  //{
  //  LL_GPIO_SetAFPin_0_7(spi_gpio_port[id], spi_gpio_pins_source[id][i], spi_AF[id]);
  //}

  /* Configure SPI pins */
  LL_GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.Pin = spi_gpio_pins[ id ];
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP; //pull-up or pull-down
  GPIO_InitStructure.Alternate = spi_AF[id];
  LL_GPIO_Init(spi_gpio_port[ id ], &GPIO_InitStructure);

  LL_SPI_DeInit(spi[ id ]);

  /* Take down, then reconfigure SPI peripheral */
  LL_SPI_Disable( spi[ id ] );
  SPI_InitStructure.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStructure.Mode = mode ? LL_SPI_MODE_MASTER : LL_SPI_MODE_SLAVE;
  SPI_InitStructure.DataWidth = ( databits == 16 ) ? LL_SPI_DATAWIDTH_16BIT : LL_SPI_DATAWIDTH_8BIT; // not ideal, but defaults to sane 8-bits
  SPI_InitStructure.ClockPolarity = cpol ? LL_SPI_POLARITY_HIGH : LL_SPI_POLARITY_LOW;
  SPI_InitStructure.ClockPhase = cpha ? LL_SPI_PHASE_2EDGE : LL_SPI_PHASE_1EDGE;
  SPI_InitStructure.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStructure.BaudRate = spi_prescaler[ prescaler_idx ];
  SPI_InitStructure.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStructure.CRCPoly = 7;
  LL_SPI_Init( spi[ id ], &SPI_InitStructure );
  LL_SPI_Enable( spi[ id ] );

  return ( SPI_GET_BASE_CLK( id ) / ( ( ( u16 )2 << ( prescaler_idx ) ) ) );
}

spi_data_type platform_spi_send_recv( unsigned id, spi_data_type data )
{
  if(LL_SPI_GetDataWidth(spi[ id ]) == LL_SPI_DATAWIDTH_16BIT)
    LL_SPI_TransmitData16( spi[ id ], data );
  else
    LL_SPI_TransmitData8( spi[ id ], data );

  while ( LL_SPI_IsActiveFlag_RXNE( spi[ id ] ) == 0 );

  if(LL_SPI_GetDataWidth(spi[ id ]) == LL_SPI_DATAWIDTH_16BIT)
    return LL_SPI_ReceiveData16( spi[ id ] );
  else
    return LL_SPI_ReceiveData8( spi[ id ] );
}

void platform_spi_select( unsigned id, int is_select )
{
  // This platform doesn't have a hardware SS pin, so there's nothing to do here
  id = id;
  is_select = is_select;
}


// ****************************************************************************
// UART
// TODO: Support timeouts.

// All possible STM32 uarts defs
USART_TypeDef *const stm32_usart[] =          { USART1, USART2, USART3, UART4, UART5, LPUART1};
const u8 stm32_usart_AF[] =       { GPIO_AF7_USART1, GPIO_AF7_USART2, GPIO_AF7_USART3, GPIO_AF8_UART4, GPIO_AF8_UART5, GPIO_AF8_LPUART1};
static GPIO_TypeDef *const usart_gpio_rx_port[] = { GPIOA, GPIOA, GPIOC, GPIOC, GPIOD };
static GPIO_TypeDef *const usart_gpio_tx_port[] = { GPIOA, GPIOA, GPIOC, GPIOC, GPIOD };
static const u16 usart_gpio_rx_pin[] = { LL_GPIO_PIN_10, LL_GPIO_PIN_3, LL_GPIO_PIN_11, LL_GPIO_PIN_11, LL_GPIO_PIN_2 };
static const u8 usart_gpio_rx_pin_source[] = { LL_GPIO_AF_10, LL_GPIO_AF_3, LL_GPIO_AF_11, LL_GPIO_AF_11, LL_GPIO_AF_2 };
static const u16 usart_gpio_tx_pin[] = { LL_GPIO_PIN_9, LL_GPIO_PIN_2, LL_GPIO_PIN_10, LL_GPIO_PIN_10, LL_GPIO_PIN_12 };
static const u8 usart_gpio_tx_pin_source[] = { LL_GPIO_AF_9, LL_GPIO_AF_2, LL_GPIO_AF_10, LL_GPIO_AF_10, LL_GPIO_AF_12 };


static GPIO_TypeDef *const usart_gpio_hwflow_port[] = { GPIOA, GPIOA, GPIOB };
static const u16 usart_gpio_cts_pin[] = { LL_GPIO_PIN_11, LL_GPIO_PIN_0, LL_GPIO_PIN_13 };
static const u16 usart_gpio_rts_pin[] = { LL_GPIO_PIN_12, LL_GPIO_PIN_1, LL_GPIO_PIN_14 };

static void usart_init(u32 id, LL_USART_InitTypeDef * initVals)
{
  /* Configure USART IO */
  LL_GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_TypeDef* prxport = usart_gpio_rx_port[ id ];
  GPIO_TypeDef* ptxport = usart_gpio_tx_port[ id ];
  u16 gpio_rx_pin = usart_gpio_rx_pin[ id ];
  u16 gpio_tx_pin = usart_gpio_tx_pin[ id ];

  // Overwrite console UART configuration with the parameters from the configuration file
/* Set this based on hardware...
  if( id == CON_UART_ID )
  {
    prxport = CON_GPIO_PORT_MACRO( STM32L4_CON_RX_PORT );
    ptxport = CON_GPIO_PORT_MACRO( STM32L4_CON_TX_PORT );
    gpio_rx_pin = CON_GPIO_PIN_MACRO( STM32L4_CON_RX_PIN );
    gpio_tx_pin = CON_GPIO_PIN_MACRO( STM32L4_CON_TX_PIN );
    gpio_rx_pinsource = CON_GPIO_SOURCE_MACRO( STM32L4_CON_RX_PIN );
    gpio_tx_pinsource = CON_GPIO_SOURCE_MACRO( STM32L4_CON_TX_PIN );
  }*/

  /* Configure USART Tx Pin as alternate function push-pull */
  LL_GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.Pin = gpio_tx_pin;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL; //push pull or open drain
  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP; //pull-up or pull-down
  GPIO_InitStructure.Alternate = stm32_usart_AF[id];
  LL_GPIO_Init(ptxport, &GPIO_InitStructure);

  /* Configure USART Rx Pin as input floating */
  GPIO_InitStructure.Pin = gpio_rx_pin;
  GPIO_InitStructure.Alternate = stm32_usart_AF[id];
  LL_GPIO_Init(prxport, &GPIO_InitStructure);

  LL_USART_DeInit(stm32_usart[id]);

  /* Configure USART */
  LL_USART_Init(stm32_usart[id], initVals);

  /* Enable USART */
  LL_USART_Enable(stm32_usart[id]);
}

static void uarts_init()
{
  // Enable clocks.
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
  //LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LPUART1, ENABLE);
}

u32 platform_uart_setup( unsigned id, u32 baud, int databits, int parity, int stopbits )
{
  LL_USART_InitTypeDef USART_InitStructure;

  if( id == CDC_UART_ID ) // no dynamic configuration yet
    return 0;

  LL_USART_StructInit(&USART_InitStructure);
  USART_InitStructure.BaudRate = baud;
  USART_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStructure.TransferDirection = LL_USART_DIRECTION_TX_RX;

  switch( databits )
  {
    case 5:
    case 6:
    case 7:
    case 8:
      USART_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B;
      break;
    case 9:
      //USART_InitStructure.DataWidth = LL_USART_DATAWIDTH_9B; //Have to update receive functions to properly receive 9 bits
      //break;
    default:
      USART_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B;
      break;
  }

  switch (stopbits)
  {
    case PLATFORM_UART_STOPBITS_1:
      USART_InitStructure.StopBits = LL_USART_STOPBITS_1;
      break;
    case PLATFORM_UART_STOPBITS_2:
      USART_InitStructure.StopBits = LL_USART_STOPBITS_2;
      break;
    default:
      USART_InitStructure.StopBits = LL_USART_STOPBITS_1;
      break;
  }

  switch (parity)
  {
    case PLATFORM_UART_PARITY_EVEN:
      USART_InitStructure.Parity = LL_USART_PARITY_EVEN;
      break;
    case PLATFORM_UART_PARITY_ODD:
      USART_InitStructure.Parity = LL_USART_PARITY_ODD;
      break;
    default:
      USART_InitStructure.Parity = LL_USART_PARITY_NONE;
      break;
  }

  usart_init(id, &USART_InitStructure);

  return baud;
}

extern uint16_t VCP_DataTx(uint8_t* Buf, uint32_t Len);
void platform_s_uart_send( unsigned id, u8 data )
{
#ifdef BUILD_USB_CDC
  if( id == CDC_UART_ID )
    VCP_DataTx( &data, 1 );
  else
#endif  
  {
    while(LL_USART_IsActiveFlag_TXE(stm32_usart[id]) == 0);
    LL_USART_TransmitData8(stm32_usart[id], data);
  }
}

int platform_s_uart_recv( unsigned id, timer_data_type timeout )
{
  if( id == CDC_UART_ID ) // this shouldn't happen
    return -1;
  if( timeout == 0 )
  {
    if (LL_USART_IsActiveFlag_RXNE(stm32_usart[id]) == 0)
      return -1;
    else
      return LL_USART_ReceiveData8(stm32_usart[id]);
  }
  // Receive char blocking
  while(LL_USART_IsActiveFlag_RXNE(stm32_usart[id]) == 0);
  return LL_USART_ReceiveData8(stm32_usart[id]);
}

int platform_s_uart_set_flow_control( unsigned id, int type )
{
  USART_TypeDef *usart = stm32_usart[ id ];
  int temp = 0;
  LL_GPIO_InitTypeDef GPIO_InitStructure;

  if( id >= 3 ) // on STM32 only USART1 through USART3 have hardware flow control ([TODO] but only on high density devices?)
    return PLATFORM_ERR;

  LL_GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_INPUT;

  if( type == PLATFORM_UART_FLOW_NONE )
  {
    usart->CR3 &= ~LL_USART_HWCONTROL_RTS_CTS;
    GPIO_InitStructure.Pin = usart_gpio_rts_pin[ id ] | usart_gpio_cts_pin[ id ];
    LL_GPIO_Init( usart_gpio_hwflow_port[ id ], &GPIO_InitStructure );
    return PLATFORM_OK;
  }
  if( type & PLATFORM_UART_FLOW_CTS )
  {
    temp |= LL_USART_HWCONTROL_CTS;
    GPIO_InitStructure.Pin = usart_gpio_cts_pin[ id ];
    LL_GPIO_Init( usart_gpio_hwflow_port[ id ], &GPIO_InitStructure );
  }
  if( type & PLATFORM_UART_FLOW_RTS )
  {
    temp |= LL_USART_HWCONTROL_RTS;
    GPIO_InitStructure.Pin = usart_gpio_rts_pin[ id ];
    GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    LL_GPIO_Init( usart_gpio_hwflow_port[ id ], &GPIO_InitStructure );
  }
  usart->CR3 |= temp;
  return PLATFORM_OK;
}

// ****************************************************************************
// Timers

u8 stm32_timer_int_periodic_flag[ NUM_PHYS_TIMER ];

// We leave out TIM6/TIM1 for now, as they are dedicated
// TODO: Utilize the other channels in these timers! Right now we are just using channel 1...
const TIM_TypeDef * const timer[] = {
  TIM2,   // ID: 0 (4 channels)
  TIM3,   // ID: 1 (4 channels)
  TIM4,   // ID: 2 (4 channels)
  TIM5,   // ID: 3 (4 channels)
  TIM15,  // ID: 4 (2 channels)
  TIM16,  // ID: 5 (1 channels)
  TIM17   // ID: 6 (1 channel)
};

const u8 timer_width[] = {
  32,  // ID: 0
  16,  // ID: 1
  16,  // ID: 2
  32,  // ID: 3
  16,  // ID: 4
  16,  // ID: 5
  16   // ID: 6
};

#define TIM_GET_PRESCALE( id ) ( (((id) == 4) || ((id) == 5)|| ((id) == 6)) ? ( PCLK2_DIV ) : ( PCLK1_DIV ) )
#define TIM_GET_BASE_CLK( id ) ( HCLK / ( TIM_GET_PRESCALE( id ) ) )

#define TIM_STARTUP_CLOCK       50000

static u32 platform_timer_set_clock( unsigned id, u32 clock );

void SysTick_Handler( void )
{
  // Handle virtual timers
  cmn_virtual_timer_cb();

  // Handle system timer call
  cmn_systimer_periodic();

#if defined( WATCHDOG_ENABLE )
  // Refresh watchdog if enabled
  WWDG_SetCounter( WATCH_COUNTER_RESET );
#endif

}

static void timers_init()
{
  unsigned i;

  // Enable PHB2 Clocks
  LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_TIM1 );
  LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_TIM8 );
  LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_TIM15 );
  LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_TIM16 );
  LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_TIM17 );

  // Enable PHB1 Clocks
  LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_TIM2 );
  LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_TIM3 );
  LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_TIM4 );
  LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_TIM5 );
  LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_TIM6 );


  // Configure timers
  for( i = 0; i < NUM_TIMER; i ++ )
    platform_timer_set_clock( i, TIM_STARTUP_CLOCK );
}

static u32 platform_timer_get_clock( unsigned id )
{
  TIM_TypeDef* ptimer = (TIM_TypeDef*)timer[ id ];
  return TIM_GET_BASE_CLK( id ) / ( LL_TIM_GetPrescaler( ptimer ) + 1 );
}

static u32 platform_timer_set_clock( unsigned id, u32 clock )
{
  LL_TIM_InitTypeDef timer_base_struct;
  LL_TIM_StructInit(&timer_base_struct);
  TIM_TypeDef *ptimer = (TIM_TypeDef*)timer[ id ];
  u32 pre = ( TIM_GET_BASE_CLK( id ) / clock ) - 1;

  if( pre > 65535 ) // Limit prescaler to 16-bits
    pre = 65535;

  timer_base_struct.Autoreload = ( timer_width[id] == 32 ? 0xFFFFFFFF : 0xFFFF );
  timer_base_struct.Prescaler = ( u16 )pre;
  timer_base_struct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  timer_base_struct.CounterMode = LL_TIM_COUNTERMODE_UP;
  timer_base_struct.RepetitionCounter = 0x0000;
  LL_TIM_Init( (TIM_TypeDef*)timer[ id ], &timer_base_struct );
  LL_TIM_EnableCounter( ptimer );

  return  platform_timer_get_clock( id );
}

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{
  TIM_TypeDef *ptimer = (TIM_TypeDef*)timer[ id ];
  volatile unsigned dummy;
  timer_data_type final;

  final = ( ( u64 )delay_us * platform_timer_get_clock( id ) ) / 1000000;
  LL_TIM_SetCounter( ptimer, 0 );
  // clear update flag so we can detect when it wraps
  ptimer->SR &= ~TIM_SR_UIF;
  for( dummy = 0; dummy < 200; dummy ++ );
  u64 timer_period = (u64)ptimer->ARR + 1;
  while( LL_TIM_GetCounter( ptimer ) < final )
  {
    if ( ptimer->SR & TIM_SR_UIF )
    {
      // timer has wrapped
      ptimer->SR &= ~TIM_SR_UIF;
      if ( final >= timer_period )
        final -= timer_period;
    }
  }
}

timer_data_type platform_s_timer_op( unsigned id, int op, timer_data_type data )
{
  u32 res = 0;
  TIM_TypeDef *ptimer = (TIM_TypeDef*)timer[ id ];
  volatile unsigned dummy;

  data = data;
  switch( op )
  {
    case PLATFORM_TIMER_OP_START:
      LL_TIM_SetCounter( ptimer, 0 ); //TIM_SetCounter( ptimer, 0 );
      for( dummy = 0; dummy < 200; dummy ++ );
      break;

    case PLATFORM_TIMER_OP_READ:
      res = LL_TIM_GetCounter( ptimer ); //TIM_GetCounter( ptimer );
      break;

    case PLATFORM_TIMER_OP_SET_CLOCK:
      res = platform_timer_set_clock( id, data );
      break;

    case PLATFORM_TIMER_OP_GET_CLOCK:
      res = platform_timer_get_clock( id );
      break;

    case PLATFORM_TIMER_OP_GET_MAX_CNT:
      res = ( timer_width[id] == 32 ? 0xFFFFFFFF : 0xFFFF );
      break;

  }
  return res;
}

int platform_s_timer_set_match_int( unsigned id, timer_data_type period_us, int type )
{
  TIM_TypeDef* base = ( TIM_TypeDef* )timer[ id ];
  u64 period;
  u32 prescaler, freq;
  timer_data_type final;
  LL_TIM_OC_InitTypeDef TIM_OCInitStructure;

  if( period_us == 0 )
  {
    LL_TIM_DisableIT_CC1(base); //TIM_ITConfig( base, TIM_IT_CC1, DISABLE );
    return PLATFORM_TIMER_INT_OK;
  }

  period = ( ( u64 )TIM_GET_BASE_CLK( id ) * period_us ) / 1000000;

  prescaler = (u32)( period / ((u64)1 << timer_width[id]) ) + 1;

  if( prescaler > 0xFFFF )
    return PLATFORM_TIMER_INT_TOO_LONG;

  platform_timer_set_clock( id, TIM_GET_BASE_CLK( id  ) / prescaler );
  freq = platform_timer_get_clock( id );
  final = ( ( u64 )period_us * freq ) / 1000000;

  if( final == 0 )
    return PLATFORM_TIMER_INT_TOO_SHORT;

  LL_TIM_DisableCounter( base );

  LL_TIM_OC_StructInit( &TIM_OCInitStructure );
  TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_FROZEN;
  TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OCInitStructure.CompareValue = final;
  TIM_OCInitStructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init( base, LL_TIM_CHANNEL_CH1, &TIM_OCInitStructure );

  // Patch timer configuration to reload when period is reached
  LL_TIM_SetAutoReload( base, final );

  LL_TIM_OC_EnablePreload( base, LL_TIM_CHANNEL_CH1 );

  stm32_timer_int_periodic_flag[ id ] = type;

  LL_TIM_SetCounter(base, 0); //TIM_SetCounter( base, 0 );
  LL_TIM_EnableCounter( base );

  return PLATFORM_TIMER_INT_OK;
}

u64 platform_timer_sys_raw_read()
{
  return SysTick->LOAD - SysTick->VAL;
}

void platform_timer_sys_disable_int()
{
  SysTick->CTRL &= ~( 1 << SysTick_CTRL_TICKINT_Pos );
}

void platform_timer_sys_enable_int()
{
  SysTick->CTRL |= 1 << SysTick_CTRL_TICKINT_Pos;
}

timer_data_type platform_timer_read_sys()
{
  return cmn_systimer_get();
}

// ****************************************************************************
// CAN
// TODO: Many things
#if (NUM_CAN > 0)

CAN_TypeDef *const stm32_can[] =             { CAN1 };
const u8 stm32_can_AF[] =                    { GPIO_AF_CAN1 };
static const u32 can_clock[] =               { LL_APB1_GRP1_PERIPH_CAN1 };

#if defined( ELUA_BOARD_STM32F4_CAN_PIN_CONFIG_1 )

static GPIO_TypeDef *const can_gpio_port[] = { GPIOD,               GPIOB };
static const u16 can_gpio_rx_pin[] =         { LL_GPIO_PIN_0,          LL_GPIO_PIN_12 };
static const u8 can_gpio_rx_pin_source[] =   { LL_GPIO_AF_0,     LL_GPIO_AF_12 };
static const u16 can_gpio_tx_pin[] =         { LL_GPIO_PIN_1,          LL_GPIO_PIN_13 };
static const u8 can_gpio_tx_pin_source[] =   { LL_GPIO_AF_1,     LL_GPIO_AF_13 };

#elif defined( ELUA_BOARD_STM32F4_CAN_PIN_CONFIG_2 )

static GPIO_TypeDef *const can_gpio_port[] = { GPIOD,               GPIOB };
static const u16 can_gpio_rx_pin[] =         { LL_GPIO_PIN_0,          LL_GPIO_PIN_5 };
static const u8 can_gpio_rx_pin_source[] =   { LL_GPIO_AF_0,     LL_GPIO_AF_5 };
static const u16 can_gpio_tx_pin[] =         { LL_GPIO_PIN_1,          LL_GPIO_PIN_6 };
static const u8 can_gpio_tx_pin_source[] =   { LL_GPIO_AF_1,     LL_GPIO_AF_6 };

#elif defined( ELUA_BOARD_STM32F4_CAN_PIN_CONFIG_3 )

static GPIO_TypeDef *const can_gpio_port[] = { GPIOB,               GPIOB };
static const u16 can_gpio_rx_pin[] =         { LL_GPIO_PIN_8,          LL_GPIO_PIN_12 };
static const u8 can_gpio_rx_pin_source[] =   { LL_GPIO_AF_8,     LL_GPIO_AF_12 };
static const u16 can_gpio_tx_pin[] =         { LL_GPIO_PIN_9,          LL_GPIO_PIN_13 };
static const u8 can_gpio_tx_pin_source[] =   { LL_GPIO_AF_9,     LL_GPIO_AF_13 };

#elif defined( ELUA_BOARD_STM32F4_CAN_PIN_CONFIG_4 )

static GPIO_TypeDef *const can_gpio_port[] = { GPIOB,               GPIOB };
static const u16 can_gpio_rx_pin[] =         { LL_GPIO_PIN_8,          LL_GPIO_PIN_5 };
static const u8 can_gpio_rx_pin_source[] =   { LL_GPIO_AF_8,     LL_GPIO_AF_5 };
static const u16 can_gpio_tx_pin[] =         { LL_GPIO_PIN_9,          LL_GPIO_PIN_6 };
static const u8 can_gpio_tx_pin_source[] =   { LL_GPIO_AF_9,     LL_GPIO_AF_6 };

#elif defined( ELUA_BOARD_STM32F4ALT ) || defined( ELUA_BOARD_STM32_E407 )

static GPIO_TypeDef *const can_gpio_port[] = { GPIOB,               GPIOB };
static const u16 can_gpio_rx_pin[] =         { LL_GPIO_PIN_8,          LL_GPIO_PIN_12 };
static const u8 can_gpio_rx_pin_source[] =   { LL_GPIO_AF_8,     LL_GPIO_AF_12 };
static const u16 can_gpio_tx_pin[] =         { LL_GPIO_PIN_9,          LL_GPIO_PIN_13 };
static const u8 can_gpio_tx_pin_source[] =   { LL_GPIO_AF_9,     LL_GPIO_AF_13 };

#else

static GPIO_TypeDef *const can_gpio_port[] = { GPIOD,               GPIOB };
static const u16 can_gpio_rx_pin[] =         { LL_GPIO_PIN_0,          LL_GPIO_PIN_12 };
static const u8 can_gpio_rx_pin_source[] =   { LL_GPIO_AF_0,     LL_GPIO_AF_12 };
static const u16 can_gpio_tx_pin[] =         { LL_GPIO_PIN_1,          LL_GPIO_PIN_13 };
static const u8 can_gpio_tx_pin_source[] =   { LL_GPIO_AF_1,     LL_GPIO_AF_13 };

#endif

void cans_init( void )
{
  // CAN Periph clock enable
  LL_APB1_GRP1_EnableClock(can_clock[0]);
}


#if ELUA_BOARD_CPU_CLOCK_HZ == 168000000

/*       BS1 BS2 SJW Pre
1M:      12  8   1   2
500k:    8   5   1   6
250k:    8   5   1   12
125k:    12  8   1   16
100k:    12  8   1   20 */

#define CAN_BAUD_COUNT 5
static const u8 can_baud_bs1[]    = { CAN_BS1_12tq, CAN_BS1_12tq, CAN_BS1_8tq, CAN_BS1_8tq, CAN_BS1_12tq };
static const u8 can_baud_bs2[]    = { CAN_BS1_8tq,  CAN_BS1_8tq,  CAN_BS1_5tq, CAN_BS1_5tq, CAN_BS1_8tq };
static const u8 can_baud_sjw[]    = { CAN_SJW_1tq,  CAN_SJW_1tq,  CAN_SJW_1tq, CAN_SJW_1tq, CAN_SJW_1tq };
static const u8 can_baud_pre[]    = { 20, 16, 12, 6, 2 };

#endif

#if ELUA_BOARD_CPU_CLOCK_HZ == 120000000

/*       BS1 BS2 SJW Pre
1M:      6   3   1   3
500k:    12  7   1   3
250k:    12  7   1   6
125k:    12  7   1   12
100k:    12  7   1   15 */

#define CAN_BAUD_COUNT 5
static const u8 can_baud_bs1[]    = { CAN_BS1_12tq, CAN_BS1_12tq, CAN_BS1_12tq, CAN_BS1_12tq, CAN_BS1_6tq };
static const u8 can_baud_bs2[]    = { CAN_BS1_7tq,  CAN_BS1_7tq,  CAN_BS1_7tq, CAN_BS1_7tq, CAN_BS1_3tq };
static const u8 can_baud_sjw[]    = { CAN_SJW_1tq,  CAN_SJW_1tq,  CAN_SJW_1tq, CAN_SJW_1tq, CAN_SJW_1tq };
static const u8 can_baud_pre[]    = { 15, 12, 6, 3, 3 };

#endif

#if ELUA_BOARD_CPU_CLOCK_HZ == 96000000

/*       BS1 BS2 SJW Pre
1M:      4   3   1   3
500k:    9   6   1   3
250k:    9   6   1   6
125k:    9   6   1   12
100k:    9   6   1   15 */

#define CAN_BAUD_COUNT 5
static const u8 can_baud_bs1[]    = { CAN_BS1_9tq, CAN_BS1_9tq, CAN_BS1_9tq, CAN_BS1_9tq, CAN_BS1_4tq };
static const u8 can_baud_bs2[]    = { CAN_BS1_6tq,  CAN_BS1_6tq,  CAN_BS1_6tq, CAN_BS1_6tq, CAN_BS1_3tq };
static const u8 can_baud_sjw[]    = { CAN_SJW_1tq,  CAN_SJW_1tq,  CAN_SJW_1tq, CAN_SJW_1tq, CAN_SJW_1tq };
static const u8 can_baud_pre[]    = { 15, 12, 6, 3, 3 };

#endif

#if ELUA_BOARD_CPU_CLOCK_HZ == 84000000

/*       BS1 BS2 SJW Pre
1M:      13   7   1   1
500k:    13   7   1   2
250k:     7   4   1   7
125k:    15   8   1   7
100k:    13   7   1  10 */

#define CAN_BAUD_COUNT 5
static const u8 can_baud_bs1[]    = { CAN_BS1_13tq, CAN_BS1_15tq, CAN_BS1_7tq, CAN_BS1_13tq, CAN_BS1_13tq };
static const u8 can_baud_bs2[]    = { CAN_BS1_7tq,  CAN_BS1_8tq,  CAN_BS1_4tq, CAN_BS1_7tq,  CAN_BS1_7tq };
static const u8 can_baud_sjw[]    = { CAN_SJW_1tq,  CAN_SJW_1tq,  CAN_SJW_1tq, CAN_SJW_1tq,  CAN_SJW_1tq };
static const u8 can_baud_pre[]    = { 10, 7, 7, 2, 1 };

#endif

static const u32 can_baud_rate[]  = { 100000, 125000, 250000, 500000, 1000000 };

u32 platform_can_setup( unsigned id, u32 clock )
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  int cbaudidx = -1;

  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(can_gpio_port[id], can_gpio_rx_pin_source[id], stm32_can_AF[id]);
  GPIO_PinAFConfig(can_gpio_port[id], can_gpio_tx_pin_source[id], stm32_can_AF[id]);

  // Configure IO Pins -- This is for STM32F103RE
  LL_GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.Pin   = can_gpio_rx_pin[id] | can_gpio_tx_pin[id];
  GPIO_InitStructure.Mode  = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull  = LL_GPIO_PULL_UP;
  LL_GPIO_Init( can_gpio_port[id], &GPIO_InitStructure );

  // Select baud rate up to requested rate, except for below min, where min is selected
  if ( clock >= can_baud_rate[ CAN_BAUD_COUNT - 1 ] ) // round down to peak rate if >= peak rate
    cbaudidx = CAN_BAUD_COUNT - 1;
  else
  {
    for( cbaudidx = 0; cbaudidx < CAN_BAUD_COUNT - 1; cbaudidx ++ )
    {
      if( clock < can_baud_rate[ cbaudidx + 1 ] ) // take current idx if next is too large
        break;
    }
  }

  /* Deinitialize CAN Peripheral */
  CAN_DeInit( stm32_can[id] );
  CAN_StructInit( &CAN_InitStructure );

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM=DISABLE;
  CAN_InitStructure.CAN_ABOM=DISABLE;
  CAN_InitStructure.CAN_AWUM=DISABLE;
  CAN_InitStructure.CAN_NART=DISABLE;
  CAN_InitStructure.CAN_RFLM=DISABLE;
  CAN_InitStructure.CAN_TXFP=DISABLE;
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW=can_baud_sjw[ cbaudidx ];
  CAN_InitStructure.CAN_BS1=can_baud_bs1[ cbaudidx ];
  CAN_InitStructure.CAN_BS2=can_baud_bs2[ cbaudidx ];
  CAN_InitStructure.CAN_Prescaler=can_baud_pre[ cbaudidx ];
  CAN_Init( stm32_can[id], &CAN_InitStructure );

  /* CAN filter init, start bank for CAN2 defaults to 0x0e */
  CAN_FilterInitStructure.CAN_FilterNumber=(CAN1==stm32_can[id])? 0 : 14;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  return can_baud_rate[ cbaudidx ];
}

u32 platform_can_op( unsigned id, int op, u32 data )
{
  u32 res = 0;
  TIM_TypeDef *ptimer = (TIM_TypeDef*)timer[ id ];
  //volatile unsigned dummy;

  data = data;
  switch( op )
  {
    case PLATFORM_TIMER_OP_READ:
      res = TIM_GetCounter( ptimer );
      break;
  }
  return res;
}

int platform_can_send( unsigned id, u32 canid, u8 idtype, u8 len, const u8 *data )
{
  CanTxMsg TxMessage;
  const char *s = ( char * )data;
  char *d;

  switch( idtype )
  {
    case ELUA_CAN_ID_STD:
      TxMessage.IDE = CAN_ID_STD;
      TxMessage.StdId = canid;
      break;
    case ELUA_CAN_ID_EXT:
      TxMessage.IDE = CAN_ID_EXT;
      TxMessage.ExtId = canid;
      break;
  }

  TxMessage.RTR=CAN_RTR_DATA;
  TxMessage.DLC=len;

  d = ( char * )TxMessage.Data;
  DUFF_DEVICE_8( len,  *d++ = *s++ );

  if( CAN_Transmit( stm32_can[id], &TxMessage ) == CAN_TxStatus_NoMailBox )
    return PLATFORM_ERR;

  return PLATFORM_OK;
}

void USB_LP_CAN_RX0_IRQHandler(void)
{
  /*
  CanRxMsg RxMessage;

  RxMessage.StdId=0x00;
  RxMessage.ExtId=0x00;
  RxMessage.IDE=0;
  RxMessage.DLC=0;
  RxMessage.FMI=0;
  RxMessage.Data[0]=0x00;
  RxMessage.Data[1]=0x00;

  CAN_Receive(stm32_can[id], CAN_FIFO0, &RxMessage);

  if((RxMessage.ExtId==0x1234) && (RxMessage.IDE==CAN_ID_EXT)
     && (RxMessage.DLC==2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8)==0xDECA))
  {
    ret = 1;
  }
  else
  {
    ret = 0;
  }*/
}

int platform_can_recv( unsigned id, u32 *canid, u8 *idtype, u8 *len, u8 *data )
{
  CanRxMsg RxMessage;
  const char *s;
  char *d;

  if( CAN_MessagePending( stm32_can[id], CAN_FIFO0 ) > 0 )
  {
    CAN_Receive(stm32_can[id], CAN_FIFO0, &RxMessage);

    if( RxMessage.IDE == CAN_ID_STD )
    {
      *canid = ( u32 )RxMessage.StdId;
      *idtype = ELUA_CAN_ID_STD;
    }
    else
    {
      *canid = ( u32 )RxMessage.ExtId;
      *idtype = ELUA_CAN_ID_EXT;
    }

    *len = RxMessage.DLC;

    s = ( const char * )RxMessage.Data;
    d = ( char* )data;
    DUFF_DEVICE_8( RxMessage.DLC,  *d++ = *s++ );
    return PLATFORM_OK;
  }
  else
    return PLATFORM_UNDERFLOW;
}
#endif


#ifdef ENABLE_ENC
// ****************************************************************************
// Quadrature Encoder Support (uses timers)
// No pin configuration, many of the timers should work with default config if
// pins aren't reconfigured for another peripheral

void stm32_enc_init( unsigned id )
{
  TIM_TypeDef *ptimer = (TIM_TypeDef *)timer[ id ];

  TIM_Cmd( ptimer, DISABLE );
  TIM_DeInit( ptimer );
  TIM_SetCounter( ptimer, 0 );
  TIM_EncoderInterfaceConfig( ptimer, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_Cmd( ptimer, ENABLE );
}

void stm32_enc_set_counter( unsigned id, unsigned count )
{
  TIM_TypeDef *ptimer = (TIM_TypeDef *)timer[ id ];

  TIM_SetCounter( ptimer, ( u16 )count );
}
#endif

// ****************************************************************************
// PWMs

/*
// Using Timer 8 (5 in eLua)
#define PWM_TIMER_ID    5
#define PWM_TIMER_NAME  TIM8
#define PWM_TIMER_AF    GPIO_AF_TIM8
#define PWM_GPIO_PORT   GPIOC
static const u8 pwm_gpio_pins_source[] = { LL_GPIO_AF_6, LL_GPIO_AF_7, LL_GPIO_AF_8, LL_GPIO_AF_9 };
*/
static void pwms_init()
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  //
}
/*
// Return the PWM clock
// NOTE: Can't find a function to query for the period set for the timer,
// therefore using the struct.
// This may require adjustment if driver libraries are updated.
u32 platform_pwm_get_clock( unsigned id )
{
  return ( ( TIM_GET_BASE_CLK( PWM_TIMER_ID ) / ( TIM_GetPrescaler( PWM_TIMER_NAME ) + 1 ) ) / ( PWM_TIMER_NAME->ARR + 1 ) );
}

// Set the PWM clock
u32 platform_pwm_set_clock( unsigned id, u32 clock )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TypeDef* ptimer = PWM_TIMER_NAME;
  unsigned period, prescaler;

  // Time base configuration 
  period = TIM_GET_BASE_CLK( PWM_TIMER_ID ) / clock;

  prescaler = (period / 0x10000) + 1;
  period /= prescaler;

  TIM_TimeBaseStructure.TIM_Period = period - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
  TIM_TimeBaseInit( ptimer, &TIM_TimeBaseStructure );

  return platform_pwm_get_clock( id );
}

u32 platform_pwm_setup( unsigned id, u32 frequency, unsigned duty )
{
  LL_TIM_OC_InitTypeDef TIM_OCInitStructure;
  LL_TIM_InitTypeDef* ptimer = PWM_TIMER_NAME;
  GPIO_InitTypeDef GPIO_InitStructure;
  u32 clock;

  LL_TIM_DisableCounter( ptimer );
  LL_TIM_SetCounter( ptimer, 0 );

  // Configure GPIO Pin as alternate function push-pull 
  GPIO_InitStructure.Pin = GPIO_SOURCE2PIN(pwm_gpio_pins_source[ id ]);
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PWM_GPIO_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(PWM_GPIO_PORT, pwm_gpio_pins_source[ id ], PWM_TIMER_AF);


  clock = platform_pwm_set_clock( id, frequency );
  TIM_ARRPreloadConfig( ptimer, ENABLE );

  // PWM Mode configuration
  TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OCInitStructure.OutputState = ( PWM_TIMER_NAME->CCER & ( ( u16 )1 << 4 * id ) ) ? TIM_OutputState_Enable : TIM_OutputState_Disable;
  TIM_OCInitStructure.OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.CompareValue = ( u16 )( duty * ( PWM_TIMER_NAME->ARR + 1 ) / 100 );
  TIM_OCInitStructure.OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.OCIdleState = TIM_OCIdleState_Set;

  switch ( id )
  {
    case 0:
      TIM_OC1Init( ptimer, &TIM_OCInitStructure );
      LL_TIM_OC_EnablePreload( ptimer, LL_TIM_CHANNEL_CH1 );
      break;
    case 1:
      TIM_OC2Init( ptimer, &TIM_OCInitStructure );
      LL_TIM_OC_EnablePreload( ptimer, LL_TIM_CHANNEL_CH2 );
      break;
    case 2:
      TIM_OC3Init( ptimer, &TIM_OCInitStructure );
      LL_TIM_OC_EnablePreload( ptimer, LL_TIM_CHANNEL_CH3 );
      break;
    case 3:
      TIM_OC4Init( ptimer, &TIM_OCInitStructure );
      LL_TIM_OC_EnablePreload( ptimer, LL_TIM_CHANNEL_CH4 ) ;
      break;
    default:
      return 0;
  }

  LL_TIM_EnableAllOutputs(ptimer);

  LL_TIM_EnableCounter( ptimer );

  return clock;
}

void platform_pwm_start( unsigned id )
{
  PWM_TIMER_NAME->CCER |= ( ( u16 )1 << 4 * id );
}

void platform_pwm_stop( unsigned id )
{
  PWM_TIMER_NAME->CCER &= ~( ( u16 )1 << 4 * id );
}
*/
// *****************************************************************************
// CPU specific functions

extern u32 SystemCoreClock;
u32 platform_s_cpu_get_frequency()
{
  SystemCoreClockUpdate();
  return SystemCoreClock;
}

void stm32_cpu_reset()
{
  NVIC_SystemReset();
}

// *****************************************************************************
// ADC specific functions and variables

#ifdef BUILD_ADC

static const u16 adc_gpio_pins[] = { LL_GPIO_PIN_0,  LL_GPIO_PIN_1,  LL_GPIO_PIN_2,  LL_GPIO_PIN_3,
                                     LL_GPIO_PIN_4,  LL_GPIO_PIN_5,  LL_GPIO_PIN_6,  LL_GPIO_PIN_7,
                                     LL_GPIO_PIN_0,  LL_GPIO_PIN_1,  LL_GPIO_PIN_0,  LL_GPIO_PIN_1,
                                     LL_GPIO_PIN_2,  LL_GPIO_PIN_3,  LL_GPIO_PIN_4,  LL_GPIO_PIN_5};

static GPIO_TypeDef * const adc_gpio_port[] = { GPIOA, GPIOA, GPIOA, GPIOA,
                                               GPIOA, GPIOA, GPIOA, GPIOA,
                                               GPIOB, GPIOB, GPIOC, GPIOC,
                                               GPIOC, GPIOC, GPIOC, GPIOC };

/* ADC EXTEN mask */
#define CR2_EXTEN_RESET           ((uint32_t)0xCFFFFFFF)

/**
  * @brief  Enables or disables the ADCx conversion through external trigger.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_ExternalTrigConvEdge: specifies the ADC external trigger edge
  *         to start  conversion.
  *          This parameter can be one of the following values:
  *            @arg ADC_ExternalTrigConvEdge_None: external trigger disabled for
  *                                                     injected conversion
  *            @arg ADC_ExternalTrigConvEdge_Rising: detection on rising edge
  *            @arg ADC_ExternalTrigConvEdge_Falling: detection on falling edge
  *            @arg ADC_ExternalTrigConvEdge_RisingFalling: detection on both rising
  *                                                               and falling edge

  * @retval None
  */
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigConvEdge)
{
  uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_EXT_TRIG_EDGE(ADC_ExternalTrigConvEdge));

  /* Get the old register value */
  tmpreg = ADCx->CR2;
  /* Clear the old external trigger edge for regular group */
  tmpreg &= CR2_EXTEN_RESET;
  /* Set the new external trigger edge for regular group */
  tmpreg |= ADC_ExternalTrigConvEdge;
  /* Store the new register value */
  ADCx->CR2 = tmpreg;
}

/**
  * @brief  Enables or disables the selected ADC software start conversion .
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  NewState: new state of the selected ADC software start conversion.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected ADC conversion on external event and start the selected
       ADC conversion */
    ADCx->CR2 |= ADC_CR2_SWSTART;
  }
  else
  {
    /* Disable the selected ADC conversion on external event and stop the selected
       ADC conversion */
    ADCx->CR2 &= (~ADC_CR2_SWSTART);
  }
}

#define ADC_DMA_STREAM  DMA2_Stream0
#define ADC_DMA_CHANNEL DMA_Channel_0
#define ADC_DMA_TCIF    DMA_IT_TCIF0

#define ADC_TRIG_CFG(adn, n) ADC_ExternalTrigConvCmd( (adn), (n)==ENABLE?ADC_ExternalTrigConvEdge_Rising:ADC_ExternalTrigConvEdge_None ) //ADC_AutoInjectedConvCmd( (adn), (n) )

#define ADC1_DR_Address ((u32)ADC1_BASE + 0x4C)

static ADC_TypeDef *const adc[] = { ADC1, ADC2, ADC3 };
static const u32 adc_timer[] = { ADC_ExternalTrigConv_T1_CC1, ADC_ExternalTrigConv_T2_TRGO, ADC_ExternalTrigConv_T3_TRGO, ADC_ExternalTrigConv_T4_CC4 };

ADC_InitTypeDef adc_init_struct;
DMA_InitTypeDef dma_init_struct;

int platform_adc_check_timer_id( unsigned id, unsigned timer_id )
{
  // NOTE: We only allow timer id 1, the TIM2, at the moment, for the sake of implementation simplicity
  return ( (timer_id == 1) ||(timer_id == 2) );
}

void platform_adc_stop( unsigned id )
{
  elua_adc_ch_state *s = adc_get_ch_state( id );
  elua_adc_dev_state *d = adc_get_dev_state( 0 );

  s->op_pending = 0;
  INACTIVATE_CHANNEL( d, id );

  // If there are no more active channels, stop the sequencer
  if( d->ch_active == 0 )
  {
    // Ensure that no external triggers are firing
    ADC_TRIG_CFG( adc[ d->seq_id ], DISABLE );

    // Also ensure that DMA interrupt won't fire ( this shouldn't really be necessary )
    nvic_init_structure_adc.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&nvic_init_structure_adc);

    d->running = 0;
  }
}

int platform_adc_update_sequence( )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  LL_GPIO_StructInit(&GPIO_InitStructure);

  // NOTE: this shutdown/startup stuff may or may not be absolutely necessary
  //       it is here to deal with the situation that a dma conversion has
  //       already started and should be reset.
  ADC_TRIG_CFG( adc[ d->seq_id ], DISABLE );

  // Stop in-progress adc dma transfers
  // Later de/reinitialization should flush out synchronization problems
  ADC_DMACmd( adc[ d->seq_id ], DISABLE );

  // Bring down adc, update setup, bring back up
  ADC_Cmd( adc[ d->seq_id ], DISABLE );
  ADC_DeInit();

  // prep for configuring pins as analog input with no pull
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;

  d->seq_ctr = 0;
  while( d->seq_ctr < d->seq_len )
  {
    // Map pin as analog input if real channel (16 & 17 are temperature and vref)
    if( d->ch_state[ d->seq_ctr ]->id < 16 )
    {
      GPIO_InitStructure.Pin = adc_gpio_pins[ d->ch_state[ d->seq_ctr ]->id ];
      LL_GPIO_Init(adc_gpio_port[ d->ch_state[ d->seq_ctr ]->id ], &GPIO_InitStructure);

      ADC_RegularChannelConfig( adc[ d->seq_id ], d->ch_state[ d->seq_ctr ]->id, d->seq_ctr+1, ADC_SampleTime_28Cycles );
    }
    else
      ADC_RegularChannelConfig( adc[ d->seq_id ], d->ch_state[ d->seq_ctr ]->id, d->seq_ctr+1, ADC_SampleTime_144Cycles );

    d->seq_ctr++;
  }
  d->seq_ctr = 0;

  adc_init_struct.ADC_NbrOfConversion = d->seq_len;
  ADC_Init( adc[ d->seq_id ], &adc_init_struct );
  ADC_TempSensorVrefintCmd(ENABLE);
  ADC_Cmd( adc[ d->seq_id ], ENABLE );

  // Bring down adc dma, update setup, bring back up
  DMA_Cmd( ADC_DMA_STREAM, DISABLE );
  DMA_DeInit( ADC_DMA_STREAM );
  dma_init_struct.DMA_BufferSize = d->seq_len;
  dma_init_struct.DMA_Memory0BaseAddr = (u32)d->sample_buf;
  DMA_Init( ADC_DMA_STREAM, &dma_init_struct );
  DMA_Cmd( ADC_DMA_STREAM, ENABLE );

  ADC_DMARequestAfterLastTransferCmd( ADC1, ENABLE );

  ADC_DMACmd( adc[ d->seq_id ], ENABLE );
  DMA_ITConfig( ADC_DMA_STREAM, DMA_IT_TC , ENABLE );

  if ( d->clocked == 1 && d->running == 1 )
  {
    ADC_TRIG_CFG( adc[ d->seq_id ], ENABLE );
  }

  return PLATFORM_OK;
}

void DMA2_Stream0_IRQHandler(void)
{
  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  elua_adc_ch_state *s;

  DMA_ClearITPendingBit(ADC_DMA_STREAM, ADC_DMA_TCIF );

  d->seq_ctr = 0;
  while( d->seq_ctr < d->seq_len )
  {
    s = d->ch_state[ d->seq_ctr ];
    s->value_fresh = 1;

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

  if( d->running == 1 )
    adc_update_dev_sequence( 0 );

  if ( d->clocked == 0 && d->running == 1 )
    ADC_SoftwareStartConvCmd( adc[ d->seq_id ], ENABLE );
}

static void adcs_init()
{
  unsigned id;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  LL_ADC_CommonStructInit(&ADC_CommonInitStructure);
  elua_adc_dev_state *d = adc_get_dev_state( 0 );

  for( id = 0; id < NUM_ADC; id ++ )
    adc_init_ch_state( id );

  LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_ADC );

  ADC_DeInit();
  ADC_StructInit( &adc_init_struct );

  // Universal Converter Setup
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  adc_init_struct.ADC_Resolution = ADC_Resolution_12b;
  adc_init_struct.ADC_ScanConvMode = ENABLE;
  adc_init_struct.ADC_ContinuousConvMode = DISABLE;
  adc_init_struct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  adc_init_struct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  adc_init_struct.ADC_DataAlign = ADC_DataAlign_Right;
  adc_init_struct.ADC_NbrOfConversion = 1;

  // Apply default config
  ADC_Init( adc[ d->seq_id ], &adc_init_struct );

  // Enable ADC
  ADC_Cmd( adc[ d->seq_id ], ENABLE );

  // Enable VREF & temperature sensor channels (16, 17)
  ADC_TempSensorVrefintCmd(ENABLE);

  // Set up DMA to handle samples
  LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_DMA2 );

  DMA_DeInit( ADC_DMA_STREAM );

  DMA_StructInit(&dma_init_struct);
  dma_init_struct.DMA_Channel = ADC_DMA_CHANNEL;
  dma_init_struct.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  dma_init_struct.DMA_Memory0BaseAddr = (u32)d->sample_buf;
  dma_init_struct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  dma_init_struct.DMA_BufferSize = 1;
  dma_init_struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma_init_struct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma_init_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  dma_init_struct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  dma_init_struct.DMA_Mode = DMA_Mode_Circular;
  dma_init_struct.DMA_Priority = DMA_Priority_Low;
  DMA_Init( ADC_DMA_STREAM, &dma_init_struct );

  // Clear flags
  DMA_ClearFlag(ADC_DMA_STREAM, DMA_FLAG_TEIF0 | DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);

  DMA_ClearITPendingBit(ADC_DMA_STREAM, ADC_DMA_TCIF );

  ADC_DMARequestAfterLastTransferCmd( ADC1, DISABLE );

  ADC_DMACmd(ADC1, ENABLE );

  DMA_Cmd( ADC_DMA_STREAM, ENABLE );
  DMA_ITConfig( ADC_DMA_STREAM, DMA_IT_TC , ENABLE );

  platform_adc_set_clock( 0, 0 );
}

u32 platform_adc_set_clock( unsigned id, u32 frequency )
{
  TIM_TimeBaseInitTypeDef timer_base_struct;
  elua_adc_dev_state *d = adc_get_dev_state( 0 );

  unsigned period, prescaler;

  // Make sure sequencer is disabled before making changes
  ADC_TRIG_CFG( adc[ d->seq_id ], DISABLE );

  if ( frequency > 0 )
  {
    d->clocked = 1;
    // Attach timer to converter
    adc_init_struct.ADC_ExternalTrigConv = adc_timer[ d->timer_id ];
    adc_init_struct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;

    period = TIM_GET_BASE_CLK( id ) / frequency;

    prescaler = (period / 0x10000) + 1;
    period /= prescaler;

    timer_base_struct.TIM_Period = period - 1;
    timer_base_struct.TIM_Prescaler = prescaler - 1;
    timer_base_struct.TIM_ClockDivision = TIM_CKD_DIV1;
    timer_base_struct.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit( (TIM_TypeDef*)timer[ d->timer_id ], &timer_base_struct );

    frequency = ( TIM_GET_BASE_CLK( id ) / ( TIM_GetPrescaler( (TIM_TypeDef*)timer[ d->timer_id ] ) + 1 ) ) / period;

    // Set up output compare for timer
    TIM_SelectOutputTrigger((TIM_TypeDef*)timer[ d->timer_id ], TIM_TRGOSource_Update);
  }
  else
  {
    d->clocked = 0;

    // Switch to Software-only Trigger
    adc_init_struct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  }

  // Apply config
  ADC_Init( adc[ d->seq_id ], &adc_init_struct );

  return frequency;
}

int platform_adc_start_sequence( )
{
  elua_adc_dev_state *d = adc_get_dev_state( 0 );

  // Only force update and initiate if we weren't already running
  // changes will get picked up during next interrupt cycle
  if ( d->running != 1 )
  {
    adc_update_dev_sequence( 0 );

    d->running = 1;

    DMA_ClearITPendingBit( ADC_DMA_STREAM, ADC_DMA_TCIF );

    nvic_init_structure_adc.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init_structure_adc);

    if( d->clocked == 1 )
      ADC_TRIG_CFG( adc[ d->seq_id ], ENABLE );
    else
      ADC_SoftwareStartConvCmd( adc[ d->seq_id ], ENABLE );
  }

  return PLATFORM_OK;
}

#endif // ifdef BUILD_ADC

// ****************************************************************************
// Flash access functions

#ifdef BUILD_WOFS
u32 platform_s_flash_write( const void *from, u32 toaddr, u32 size )
{
  u32 ssize = 0;
  const u16 *psrc = ( const u16* )from;
  FLASH_Status flstat;

  while( ssize < size )
  {
    if( ( flstat = FLASH_ProgramHalfWord( toaddr, *psrc ++ ) ) != FLASH_COMPLETE )
    {
      printf( "ERROR in platform_s_flash_write: stat=%d at %08X\n", ( int )flstat, ( unsigned )toaddr );
      break;
    }
    toaddr += 2;
    ssize += 2;
  }
  return ssize;
}

static const u16 flash_sectors[] = { FLASH_Sector_0, FLASH_Sector_1, FLASH_Sector_2, FLASH_Sector_3,
                                     FLASH_Sector_4, FLASH_Sector_5, FLASH_Sector_6, FLASH_Sector_7,
                                     FLASH_Sector_8, FLASH_Sector_9, FLASH_Sector_10, FLASH_Sector_11 };

int platform_flash_erase_sector( u32 sector_id )
{
  return FLASH_EraseSector( flash_sectors[ sector_id ], VoltageRange_3 ) == FLASH_COMPLETE ? PLATFORM_OK : PLATFORM_ERR;
}

#endif // #ifdef BUILD_WOFS


void memory_error( void )
{
  //sim3_pmu_reboot(); //TODO: IMPLEMENT ERROR HANDLING
}

//TODO: IMPLEMENT WATCHDOG TIMER
volatile u16 wdt_reset_counter = 20;

void watchdog_counter_set( u16 value )
{
  wdt_reset_counter = value;
}


