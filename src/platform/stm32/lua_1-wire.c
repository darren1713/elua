/*
	1-wire communication from/to Lua
	
	License: Released under the Creative Commons Attribution Share-Alike 3.0 License. 
			 http://creativecommons.org/licenses/by-sa/3.0	
			 
	Author:	 Omnima Limited
	Target:  Omnima STM32Expander, GCC compiler
	
	Distribution: Must preserve this header and the list of authors who have contributed to this module
	
	Supported 1-wire devices:
	
		Temperature sensor DS18B20
		8-port switch DS2408
		
		Additional devices can be supoprted by extending the code found in ds2482.c
		
	Example use:
	
		w1.init()
		s,r=w1.first()
		w1.get(s) -- return the temoperature or status of the 1-wire device
		s,r=w1.next()
		if s[1]==0x28 then print("This is DS18B20 temperature sensor") end
		if s[1]==0x29 then print("This is DS2408 8 port switch") w1.set(s,0xff) end --sets all 8 ports to 1s
*/

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "lrotable.h"
#include "platform_conf.h"
#include "auxmods.h"
#include "elua_int.h"

#include "i2c-bb.h"
#include "ds2482.h"
#include "ow.h"

//Lua: init(id)
static int w1_init( lua_State *L )
{  
  //1-wire DS2482-100 Initialisation
  i2c_port_type i2c2;
  i2c2.init=0;
  i2c2.start=0;
  i2c2.scl_port=GPIOB;
  i2c2.scl_pin=GPIO_Pin_10;
  i2c2.sda_port=GPIOB;
  i2c2.sda_pin=GPIO_Pin_11;  
  
  init_I2C(&i2c2);
  
  if (ds2482_detect(&i2c2))
  {
      //All initialised ok and ds2482 found
      //OWSearchCache();
      lua_pushinteger(L, 1);
  } else
  {
      //Falied to initialise
      lua_pushinteger(L, 0);
  }
  
  //1-wire bus network layer initialisation
  OWInit(&i2c2);
  
  return 1;
}

void w1_helper_set_w1_id(lua_State *L)
//Convert 8 byte 1-wire ID from a C array to Lua array
{
  int i;
  
  lua_newtable(L);
  
  for(i=0;i<sizeof(ROM_NO);i++)
  {
      lua_pushinteger(L, ROM_NO[i]);
      lua_rawseti(L, -2, i+1);
  }  
}

void w1_helper_get_w1_id(lua_State *L, uint8_t buf[8])
//Convert the Lua 8 byte array to C array
{
  int i,n;
  
  luaL_checktype(L, 1, LUA_TTABLE);
  n=lua_objlen(L,1);
  
  for(i=0;i<n;i++)
  {
      lua_rawgeti(L, 1, i+1);
      buf[i]=lua_tonumber(L, -1);
      lua_pop(L, 1);
  }
}

static int w1_first( lua_State *L )
//Enumerates 1-wire bus and returns Id of the first device found
//Arg1 - device id - array of 8 numbers
//Arg2 - 1 is successful, 0 - if not
{
  int ret=OWFirst();  
  w1_helper_set_w1_id(L);
  lua_pushinteger(L, ret);
  return 2;
}

static int w1_next( lua_State *L )
//Enumerates 1-wire bus and returns Id of the next available device
//Arg1 - device id - array of 8 numbers
//Arg2 - 1 is successful, 0 - if not
{
  int ret=OWNext();
  w1_helper_set_w1_id(L);
  lua_pushinteger(L, ret);
  return 2;
}

static int w1_set( lua_State *L )
//Sets 1-wire device value
//Arg1 - device id - array of 8 numbers
//Arg2 - value to set
{
  uint8_t romid[8], ret; 
  w1_helper_get_w1_id(L, romid);
  luaL_checktype(L, 2, LUA_TNUMBER);
  ret=OWSetDeviceValue(romid, lua_tointeger(L, 2));
  lua_pushinteger(L, ret);  
  return 1;
}

static int w1_get( lua_State *L )
//Retreives status/value from a 1-wire device
//Arg1 - device id - array of 8 numbers
//Returns: device value
{ 
  uint8_t i,romid[8];  
  uint16_t val;
  
  w1_helper_get_w1_id(L, romid);
  val=OWGetDeviceValue(romid);
  lua_pushinteger(L, val);
  return 1;
}

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"  

// Module function map
const LUA_REG_TYPE w1_map[] =
{ 
  { LSTRKEY( "init" ),    LFUNCVAL( w1_init ) },
  { LSTRKEY( "first" ), LFUNCVAL( w1_first ) },
  { LSTRKEY( "next" ),  LFUNCVAL( w1_next ) },
  { LSTRKEY( "set" ),  LFUNCVAL( w1_set ) },
  { LSTRKEY( "get" ),  LFUNCVAL( w1_get ) },
  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_1wire( lua_State *L )
{
  LREGISTER( L, AUXLIB_1WIRE, w1_map );
}  
