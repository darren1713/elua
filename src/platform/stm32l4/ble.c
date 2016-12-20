// eLua Module for STM32L4 I2C features based on interrupts

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "lrotable.h"
#include "platform_conf.h"
#include "auxmods.h"
#include "elua_int.h"
#include <string.h>
#include "ble.h"
#include <stdlib.h>

//Lua: ble.reset()
static int ble_reset( lua_State *L )
{
  unsigned char data[512]; //All packets below 512 bytes
  int i;	

  //Create the packet	
  create_packet(0x01, data, 0);

  //Not sure why this needs to be called each time
  platform_uart_setup(0, 115200, 8, PLATFORM_UART_PARITY_NONE, PLATFORM_UART_STOPBITS_1);

  //Send packet thru UART
  for(i=0; i<PACKET_LEN(0); i++)		
  	platform_s_uart_send(UART_BLE, data[i]);

  //Zero values return to Lua	
  return 0;
}


//Lua:
//t=ble.ibeacons()
//print((t[1]):byte(1,#t[1]))
//print((t[2]):byte(1,#t[2]))
// ...
static int ble_ibeacons( lua_State *L )
{
  unsigned char *data; //All packets below 512 bytes
  int i, j, start, iBeacons;

  //Allocate memory
  data=malloc(IBEACON_BUFFER_SIZE);
  if(data == NULL)  return 0; //return to lua	

  //Create the packet   
  create_packet(0x02, data, 0);

  //Not sure why this needs to be called each time
  platform_uart_setup(0, 115200, 8, PLATFORM_UART_PARITY_NONE, PLATFORM_UART_STOPBITS_1);

  //Send packet thru UART
  for(i=0; i<PACKET_LEN(0); i++)
    platform_s_uart_send(UART_BLE, data[i]);

  //Receive ibeacons from BLE with timeout
  i = platform_ble_ibeacons_uart_recv(UART_BLE, data);

  //Push iBeacon data into a table
  if(i){
	iBeacons=0;
	start=0;
	for(j=1;j<i;j++){
	  if((data[j-1] == 0x5A) && (data[j] == 0x7E)){
		if(!start) 
			lua_newtable(L);
		lua_pushnumber(L,++iBeacons);
		lua_pushlstring(L, (char *)(data+start), j-1-start);
		lua_settable(L, -3);

		start=j+1;

	  }

	} 
  }	

  free(data);

  //Zero values return to Lua   
  return 1;
}



//Lua: print(ble.getparam(param_id))
//param_id:
//  1 - Device_Name - BLE advertising name, string having maximum 15 characters.
//  2 - Adv_Interval - The advertising interval is four bytes integer (in steps of 0.625 ms.)
static int ble_getparam( lua_State *L )
{
  
  unsigned char param_id, data[512]; //All packets below 512 bytes
  unsigned char *PacketID, *Param;
  unsigned short ParamLen;
  int i;

  //Get the param_id from the argument stack
  param_id = luaL_checkinteger( L, 1 );

  //the requested param type we pass as first byte of Data
  data[0] = param_id;

  //Create the packet   
  create_packet(0x03, data, 1);

  //Not sure why this needs to be called each time
  platform_uart_setup(0, 115200, 8, PLATFORM_UART_PARITY_NONE, PLATFORM_UART_STOPBITS_1);

  //Send packet thru UART
  for(i=0; i<PACKET_LEN(1); i++)
    platform_s_uart_send(UART_BLE, data[i]);

  //Receive data from BLE with timeout
  i = platform_ble_uart_recv(UART_BLE, data); 

  if(parse_packet(data, i, &PacketID, &Param, &ParamLen) == -1)
	return 0; //Wrong Packet format, no value returned to Lua

  if(*Param == 1) { //BLE advertising name
    Param[ParamLen]=0; //Terminate the string
	lua_pushstring(L, (char *)(Param+1));
	
  }else if(*Param == 2){ //The advertising interval
	lua_pushnumber(L, *(unsigned int *)(Param+1));
  }else
	return 0; //Unsupported param_id, no value returned to Lua  


  //One value return to Lua, if param_id == 1 it is string, if param_id == 2 it is a number   
  return 1;
}


//Lua: ble.setparam(param_id, value)
//param_id:
//	1 - Device_Name - BLE advertising name, string having maximum 15 characters.
//  2 - Adv_Interval - The advertising interval is four bytes integer (in steps of 0.625 ms.)
static int ble_setparam( lua_State *L )
{
  unsigned char data[512]; //All packets below 512 bytes
  int i, datalen;

  //the requested param type we pass as first byte of Data
  data[0] = luaL_checkinteger( L, 1 );

  if(data[0] == 1){ //Device_Name
	const char *Device_Name = luaL_checkstring( L, 2 );
	
	datalen = strlen(Device_Name);
	
	for(i=0; i<datalen; i++){
		data[i+1] = Device_Name[i];
	} 	

	datalen++; //Add one byte for the param_id

  } else if(data[0] == 2){ //Adv_Interval
  	*(unsigned int *)(data+1) = luaL_checkinteger( L, 2 );

	datalen = 5; //One byte for param_id and 4 bytes for Adv_Interval
  }

  //Create the packet   
  create_packet(0x04, data, datalen);

  //Not sure why this needs to be called each time
  platform_uart_setup(0, 115200, 8, PLATFORM_UART_PARITY_NONE, PLATFORM_UART_STOPBITS_1);

  //Send packet thru UART
  for(i=0; i<PACKET_LEN(datalen); i++)
    platform_s_uart_send(UART_BLE, data[i]);

  //Zero values return to Lua   
  return 0;
}


//Lua: print(ble.getstat())
static int ble_getstat( lua_State *L )
{
  unsigned char data[512]; //All packets below 512 bytes
  unsigned char *PacketID, *Param;
  unsigned short ParamLen;
  char address[18];
  int i;

  //the requested param type we pass as first byte of Data
  data[0] = luaL_checkinteger( L, 1 );

  //Create the packet   
  create_packet(0x05, data, 1);

  //Not sure why this needs to be called each time
  platform_uart_setup(0, 115200, 8, PLATFORM_UART_PARITY_NONE, PLATFORM_UART_STOPBITS_1);

  //Send packet thru UART
  for(i=0; i<PACKET_LEN(1); i++)
    platform_s_uart_send(UART_BLE, data[i]);


  //Receive data from BLE with timeout
  i = platform_ble_uart_recv(UART_BLE, data);

  if(parse_packet(data, i, &PacketID, &Param, &ParamLen) == -1)
    return 0; //Wrong Packet format, no value returned to Lua

  if((*Param == 1) && (ParamLen == 7)) { //BLE Central Address
	sprintf(address, "%2x:%2x:%2x:%2x:%2x:%2x", *(Param+6), *(Param+5), *(Param+4), *(Param+3), *(Param+2), *(Param+1)); 
    lua_pushstring(L, address);
	  
	//One value return to Lua, if param_id == 1 it is string, if param_id == 2 it is a number   
  	return 1;

  }else
    return 0; //Unsupported param_id, no value returned to Lua  

}

//Lua: ble.cdc(enable)
//enable can be:
//  0 - disabling the CDC thru the BLE module
//  1 - enable CDC thru the BLE module
static int ble_cdc( lua_State *L )
{

  //the requested param type we pass as first byte of Data
  BLE_CDC_ENABLED = (luaL_checkinteger( L, 1 ) > 0);

  //Zero values return to Lua   
  return 0;
}


#define MIN_OPT_LEVEL 2
#include "lrodefs.h"  

// Module function map
const LUA_REG_TYPE ble_map[] =
{ 
  { LSTRKEY( "reset" ),  LFUNCVAL( ble_reset ) },
  { LSTRKEY( "ibeacons" ),  LFUNCVAL( ble_ibeacons ) },
  { LSTRKEY( "getparam" ),  LFUNCVAL( ble_getparam ) },
  { LSTRKEY( "setparam" ),  LFUNCVAL( ble_setparam ) },
  { LSTRKEY( "getstat" ),  LFUNCVAL( ble_getstat ) },
  { LSTRKEY( "cdc" ),  LFUNCVAL( ble_cdc ) },
  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_ble( lua_State *L )
{
  LREGISTER( L, AUXLIB_BLE, ble_map );
}  

/*
 * PacketID - single byte to put in the packet
 * Data 	- Array alocated by the caller having DataLen bytes to be put in the packet
 * 			  the same array is used to return the whole packet. Caller muts alocate 
 *			  7 bytes more for the packet header
 */
void create_packet(unsigned char PacketID, unsigned char *Data, unsigned short DataLen){
	
	unsigned char sum;
	int i;

	//Shift the data 6 bytes to reserve space for the header
	for(i=DataLen-1; i>=0; i--){
		Data[i+6]=Data[i];
	}

	Data[0] = 0x5A;	//Start bytes
	Data[1] = 0x7E;

	Data[2] = PacketID;	 	

	Data[3] = 0;	//In this file we deal with single packet comunication so 'Packets Left' is zero

					//Add the length
	Data[4] = (unsigned char)(DataLen + 7);
	Data[5] = (unsigned char)((DataLen + 7)>>8);

	sum=0;			//Add the Sum
	for(i=0;i<6+DataLen;i++) sum+=Data[i];
	Data[6+DataLen] = sum; 

}

/* 
 * Input arguments:
 *   Packet		- Contains data we got from the BLE module
 *   PacketLen	- Specifies the bytes count in the Packet
 *
 * Output arguments: (should not be allocated by the caller)
 *   PacketID - Packet ID from the packet
 *   Data     - Data contained in teh Packet
 *   DataLen  - Specifies the bytes count in the Data
 *
 * Returns 0 if Packet format OK or -1 otherwise  
 */
int parse_packet(unsigned char *Packet, unsigned short PacketLen, unsigned char **PacketID, unsigned char **Data, unsigned short *DataLen){

    int i;
	unsigned char sum=0;	

	//Check the checksum
	for(i=0;i<(PacketLen-1);i++) sum+=Packet[i];

	if((Packet[0]!=0x5A) || (Packet[1]!=0x7E) || (((unsigned short)Packet[4])!=PacketLen) || (Packet[PacketLen-1]!=sum))
		return -1; //Wrong Packet format

	*PacketID = Packet+2;

	*Data = Packet+6;

	*DataLen = PacketLen - 7;
			
	return 0;
}
