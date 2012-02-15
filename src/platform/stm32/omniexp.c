
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "lrotable.h"
#include "platform_conf.h"
#include "auxmods.h"
#include "elua_int.h"

#define DEBUG
#include "trace.h"

//Lua: init(id)
static int omniexp_get_stm32id( lua_State *L )
{
  unsigned id;
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;
  
  //id = luaL_checkint( L, 1 );
  
  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);  
  
  Device_Serial0 += Device_Serial2;
    
  lua_pushinteger(L, Device_Serial0);
  lua_pushinteger(L, Device_Serial1);
  
  return 2;
}

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"  

// Module function map
const LUA_REG_TYPE omniexp_map[] =
{ 
  { LSTRKEY( "get_stm32id" ),  LFUNCVAL( omniexp_get_stm32id ) },
//  { LSTRKEY( "enable_zwave" ),  LFUNCVAL( omniexp_enable_zwave ) },
//  { LSTRKEY( "enable_spi_flash" ),  LFUNCVAL( omniexp_enable_spi_flash ) },
//  { LSTRKEY( "enable_1wire" ),  LFUNCVAL( omniexp_enable_1wire ) },
//  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_omniexp( lua_State *L )
{
  LREGISTER( L, AUXLIB_OMNIEXP, omniexp_map );
}
