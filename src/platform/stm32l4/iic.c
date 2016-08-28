// eLua Module for STM32L4 I2C features based on interrupts

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "lrotable.h"
#include "platform_conf.h"
#include "auxmods.h"
#include "elua_int.h"
#include "iic.h"


//Lua: iic.write(data)
static int iic_write( lua_State *L )
{
  unsigned char data;
  data = luaL_checkinteger( L, 1 );

  //Invoke the platform function	
  stm32l4_i2c_write(data);

  //Zero values return to Lua	
  return 0;
}

//Lua: data=iic.read()
static int iic_read( lua_State *L )
{
  unsigned char data;

  //Invoke the platform function    
  stm32l4_i2c_read();

  //stm32l4_i2c_read() and stm32l4_i2c_write() are interrupt based
  //We loop here for the read to complete and return result to Lua 	
  data = stm32l4_i2c_read_result();

  lua_pushnumber(L, data);

  //One value returned to Lua
  return 1; 
}


#define MIN_OPT_LEVEL 2
#include "lrodefs.h"  

// Module function map
const LUA_REG_TYPE iic_map[] =
{ 
  { LSTRKEY( "write" ),  LFUNCVAL( iic_write ) },
  { LSTRKEY( "read" ),  LFUNCVAL( iic_read ) },
  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_iic( lua_State *L )
{
  LREGISTER( L, AUXLIB_IIC, iic_map );
}  

