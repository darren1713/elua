#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "type.h"
#include "devman.h"
#include "platform.h"
#include "romfs.h"
#include "xmodem.h"
#include "shell.h"
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
#include "term.h"
#include "platform_conf.h"
#include "elua_rfs.h"
#ifdef ELUA_SIMULATOR
#include "hostif.h"
#endif

#define DEBUG
#include "trace.h"

// Validate eLua configuratin options
#include "validate.h"

#include "mmcfs.h"
#include "romfs.h"
#include "semifs.h"

// Define here your autorun/boot files,
// in the order you want eLua to search for them
char *boot_order[] = {
#if defined(BUILD_MMCFS)
  "/mmc/autorun.lua",
  "/mmc/autorun.lc",
#endif
#if defined(BUILD_ROMFS)
  "/rom/autorun.lua",
  "/rom/autorun.lc",
#endif
};

extern char etext[];


#ifdef ELUA_BOOT_RPC

#ifndef RPC_UART_ID
  #define RPC_UART_ID     CON_UART_ID
#endif

#ifndef RPC_TIMER_ID
  #define RPC_TIMER_ID    PLATFORM_TIMER_SYS_ID
#endif

#ifndef RPC_UART_SPEED
  #define RPC_UART_SPEED  CON_UART_SPEED
#endif

void boot_rpc( void )
{
  TRACE("1");
  lua_State *L = lua_open();
  luaL_openlibs(L);  /* open libraries */

  TRACE("2");

  // Set up UART for 8N1 w/ adjustable baud rate
  platform_uart_setup( RPC_UART_ID, RPC_UART_SPEED, 8, PLATFORM_UART_PARITY_NONE, PLATFORM_UART_STOPBITS_1 );

  TRACE("3");

  // Start RPC Server
  lua_getglobal( L, "rpc" ); TRACE("4");
  lua_getfield( L, -1, "server" ); TRACE("5");
  lua_pushnumber( L, RPC_UART_ID ); TRACE("6");
  lua_pushnumber( L, RPC_TIMER_ID ); TRACE("7");
  lua_pcall( L, 2, 0, 0 ); TRACE("8");
}
#endif

// ****************************************************************************
//  Program entry point

int main( void )
{
  int i;
  FILE* fp;

  //#ifdef DEBUG
  //delay_ms(1000);
  //#endif

  if( platform_init() != PLATFORM_OK )
  {
    // This should never happen
    while( 1 );
  }

  TRACE("OmnimaExt - eLua - Initialized\n");

  TRACE("eLua - Initialize device manager\n");
  dm_init();

  TRACE("eLua - Register the ROM filesystem\n");
  dm_register( romfs_init() );

  TRACE("eLua - Register the MMC filesystem\n");
  dm_register( mmcfs_init() );

  TRACE("eLua - Register the Semihosting filesystem\n");
  dm_register( semifs_init() );

  TRACE("eLua - Register the remote filesystem\n");
  dm_register( remotefs_init() );

  TRACE("eLua - Search for autorun files\n");
  //in the defined order and execute the 1st if found
  for( i = 0; i < sizeof( boot_order ) / sizeof( *boot_order ); i++ )
  {
    if( ( fp = fopen( boot_order[ i ], "r" ) ) != NULL )
    {
      fclose( fp );
      char* lua_argv[] = { "lua", boot_order[i], NULL };
      lua_main( 2, lua_argv );
      break; // autoruns only the first found
    }
  }

#ifdef ELUA_BOOT_RPC
  TRACE("eLua - boot_rpc");
  boot_rpc();
#else

  // Run the shell
  if( shell_init() == 0 )
  {
    TRACE("eLua - Start Lua directly");
    char* lua_argv[] = { "lua", NULL };
    lua_main( 1, lua_argv );
  }
  else
  {
    TRACE("eLua - shell_start");
    shell_start();
  }
#endif // #ifdef ELUA_BOOT_RPC

#ifdef ELUA_SIMULATOR
  hostif_exit(0);
  return 0;
#else
  while( 1 );
#endif
}
