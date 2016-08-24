// eLua Module for SIM3 power features

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "lrotable.h"
#include "platform_conf.h"
#include "auxmods.h"
#include "elua_int.h"
#include "pmu.h"


//Lua: sleep(time)
static int pmu_sleep( lua_State *L )
{
  unsigned seconds;
  seconds = luaL_checkinteger( L, 1 );

  //We go to low power mode stop2	
  stm32l4_EnterStop2Mode(seconds);

  //Get here when something wakes us up 
  //or RTC timeout passes.
  LL_RTC_DisableIT_WUT(RTC);
  LL_RTC_WAKEUP_Disable(RTC);

  return 0;
}

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"  

// Module function map
const LUA_REG_TYPE pmu_map[] =
{ 
  { LSTRKEY( "sleep" ),  LFUNCVAL( pmu_sleep ) },
  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_pmu( lua_State *L )
{
  LREGISTER( L, AUXLIB_PMU, pmu_map );
}  

