#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
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

// Validate eLua configuratin options
#include "validate.h"

#include "mmcfs.h"
#include "romfs.h"
#include "semifs.h"
#include "nffs.h"

// Define here your autorun/boot files, 
// in the order you want eLua to search for them
const char *boot_order[] = {
#if defined(BUILD_MMCFS)
  "/mmc/autorun.lua",
  "/mmc/autorun.lc",
#endif
#if defined(BUILD_WOFS)
  "/wo/autorun.lua",
  "/wo/autorun.lc",
#endif
#if defined(BUILD_NIFFS)
  "/f/autorun.lua",
  "/f/autorun.lc",
#endif
#if defined(BUILD_ROMFS)
  "/rom/autorun.lua",
  "/rom/autorun.lc",
#endif
#if defined(BUILD_SEMIFS)
  "/semi/autorun.lua",
  "/semi/autorun.lc",
#endif
};

extern char etext[];

#ifdef ELUA_BOOT_RPC
void boot_rpc( void )
{
  lua_State *L = lua_open();
  luaL_openlibs(L);  /* open libraries */
  
  // Set up UART for 8N1 w/ adjustable baud rate
  platform_uart_setup( RPC_UART_ID, RPC_UART_SPEED, 8, PLATFORM_UART_PARITY_NONE, PLATFORM_UART_STOPBITS_1 );
  
  // Start RPC Server
  lua_getglobal( L, "rpc" );
  lua_getfield( L, -1, "server" );
  lua_pushnumber( L, RPC_UART_ID );
  lua_pushnumber( L, RPC_TIMER_ID );
  lua_pcall( L, 2, 0, 0 );
}
#endif // #ifdef ELUA_BOOT_RPC

// ****************************************************************************
//  Program entry point

// Define a weak symbol for an user init function which is called during startup
// The user code can override this function with a regular (non-weak) symbol.
// Suggested use is printing startup data, if the board is booting up and not going back to sleep.
void __attribute__((weak)) elua_user_init( void )
{
}

// Define a weak symbol for a very early user init function which is called during startup
// The user code can override this function with a regular (non-weak) symbol.
// Suggested use is setup of I/O's, timers, and very fast checking of sleep states.
void __attribute__((weak)) elua_user_init_early( void )
{
}

// Define a weak symbol for notifying user modules if eLua VM exits
// The user code can override this function with a regular (non-weak) symbol.
// Suggested use is rebooting or restarting the eLua VM
void __attribute__((weak)) elua_user_exit( void )
{
}

// Define a weak symbol for pausing the watchdog while the VM is not spinning (shell for example)
// The user code can override this function with a regular (non-weak) symbol.
void __attribute__((weak)) elua_user_watchdog_pause( void )
{
}

int main( void )
{
  int i;
  FILE* fp;

  // Initialize platform first
  if( platform_init() != PLATFORM_OK )
  {
    // This should never happen
    while( 1 );
  }

  // Early Init Extras
  elua_user_init_early();

  // Initialize device manager
  dm_init();

  // Register the ROM filesystem
  romfs_init();

  // Register the MMC filesystem
  mmcfs_init();

  // Register the Semihosting filesystem
  semifs_init();

  // Register the remote filesystem
  remotefs_init();

  // Register NIFFS
  nffs_init();

  // User-specific initialization code
  elua_user_init();

  // Search for autorun files in the defined order and execute the 1st if found
  for( i = 0; i < sizeof( boot_order ) / sizeof( *boot_order ); i++ )
  {
    if( ( fp = fopen( boot_order[ i ], "r" ) ) != NULL )
    {
      fclose( fp );
#ifdef ENABLE_INTERACTIVE_AUTORUN
      char* lua_argv[] = { (char *)"lua", (char *)"-i" , (char *)boot_order[i], NULL };
#else
      char* lua_argv[] = { (char *)"lua" , (char *)boot_order[i], NULL };
#endif
      lua_main( 2, lua_argv );
      break; // autoruns only the first found
    }
  }

  // Watchdog ticks only occur while in lua VM, pause watchdog if we are going to shell...
  elua_user_watchdog_pause();

  printf("Lua no autorun or exited\n");

#ifdef ELUA_BOOT_RPC
  boot_rpc();
#else
  
  // Run the shell
  if( shell_init() == 0 )
  {
    // Start Lua directly
    printf("Lua starting directly\n");
    char* lua_argv[] = { (char *)"lua", NULL };
    lua_main( 1, lua_argv );
  }
  else
    shell_start();
#endif // #ifdef ELUA_BOOT_RPC

printf("Lua exited\n");

  elua_user_exit();

#ifdef ELUA_SIMULATOR
  hostif_exit(0);
  return 0;
#else
  while( 1 );
#endif
}
