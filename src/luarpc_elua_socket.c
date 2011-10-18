#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "platform_conf.h"
#include "luarpc_rpc.h"
#include "elua_net.h"
#include "uip.h"

#if defined (BUILD_RPC) && defined (ELUA_BOOT_RPC_SOCKET)

#define ELUA_PORT_ID	12346

/* Too lazy for elua_net_get_last_err(###) */
int elua_sock_err(int sockfd)
{
  return elua_net_get_last_err(sockfd);
}

/* Setup transport */
void transport_init(Transport *tpt)
{
  tpt->fd = INVALID_TRANSPORT;
  tpt->tmr_id = 0;
}

/* Check if transport is open */
int transport_is_open(Transport *tpt)
{
  return (tpt->fd != INVALID_TRANSPORT);
}

/* Open a socket */
void transport_open(Transport *tpt)
{
  struct exception e;
  tpt->fd = elua_net_socket(ELUA_NET_SOCK_STREAM);
  if (tpt->fd == INVALID_TRANSPORT) {
    e.errnum = elua_sock_err(tpt->fd);
    e.type = fatal;
    Throw( e );
  }
}

/* Close a socket */
void transport_close (Transport *tpt)
{
  if (tpt->fd != INVALID_TRANSPORT)
    elua_net_close(tpt->fd);
  tpt->fd = INVALID_TRANSPORT;
}

/* Connect the socket to a host */
static void transport_connect (Transport *tpt, elua_net_ip ip_address, u16 ip_port)
{
  struct exception e;
  TRANSPORT_VERIFY_OPEN;
  if (elua_net_connect(tpt->fd, ip_address, ip_port) != 0) {
    e.errnum = elua_sock_err(tpt->fd);
    e.type = fatal;
    Throw( e );
  }
}

/* Accept an incoming connection */
void transport_accept (Transport *tpt, Transport *atpt)
{
  struct exception e;
  elua_net_ip ret_ip;
  u32 to_us = 0; /* Not sure how to use this */
  TRANSPORT_VERIFY_OPEN;
  atpt->fd = elua_accept(ELUA_PORT_ID, tpt->tmr_id, to_us, &ret_ip);
  if (atpt->fd == INVALID_TRANSPORT) {
    e.errnum = elua_sock_err(atpt->fd);
    e.type = fatal;
    Throw( e );
  }
}

/* Open listener for connection */
void transport_open_listener(lua_State *L, ServerHandle *handle)
{
  // Get args & Set up connection
  unsigned port_id, tmr_id;
  
  check_num_args( L,3 ); // 1st arg is port num, 2nd arg is tmr_id, 3nd is handle
  if ( !lua_isnumber( L, 1 ) ) 
    luaL_error( L, "1st arg must be port num" );
    
  if ( !lua_isnumber( L, 2 ) ) 
    luaL_error( L, "2nd arg must be timer num" );

  // @@@ Error handling could likely be better here
  port_id = lua_tonumber( L, 1 );
  if( port_id < 0 || port_id > 0xffff )
    luaL_error( L, "port number must be in the range 0..65535" );
  
  tmr_id = lua_tonumber( L, 2 );
  if( !platform_timer_exists( tmr_id ) )
    luaL_error( L, "invalid timer id" );
   
  /* Initialize the timer in the handle */
  handle->ltpt.tmr_id = tmr_id;

  transport_open(&handle->ltpt);
  /* Accept also listens */
  transport_accept(&handle->ltpt, &handle->atpt);
}

/* Open connection */
int transport_open_connection(lua_State *L, Handle *handle)
{
  struct exception e;
  unsigned int port;
  elua_net_ip ip;
  
  check_num_args( L,3 ); // 1st arg is ip addr, 2nd arg is port num, 3rd is handle
  if ( !lua_isnumber( L, 1 ) ) 
    return luaL_error( L, "1st arg must be an ip addr" );
  ip.ipaddr = (u32) luaL_checkinteger(L, 1);
    
  if ( !lua_isnumber( L, 2 ) ) 
    return luaL_error( L, "2nd arg must be port num" );
  port = lua_tonumber( L, 2 );
  
  transport_open(&handle->tpt);
  if (elua_net_connect(&handle->tpt.fd, ip, port) != 0) {
    e.errnum = elua_sock_err(&handle->tpt.fd);
    e.type = fatal;
    Throw( e );
  }
  return 1; 
}

/* Read from socket into the buffer */
void transport_read_buffer (Transport *tpt, u8 *buffer, int length)
{
  int n = 0;
  int timeout = -1; /* Took this from UART INFINITE timeout */
  struct exception e;
  TRANSPORT_VERIFY_OPEN;
  /* Check on the arguments again */
  n = elua_net_recv (tpt->fd, (void*) buffer, length, -1, tpt->tmr_id, timeout);
  if (n == 0) {
    e.errnum = ERR_EOF;
    e.type = nonfatal;
    Throw( e );
  }
  if (n < 0) {
    e.errnum = elua_sock_err(tpt->fd);
    e.type = fatal;
    Throw( e );
  }
}

/* Write to socket */
void transport_write_buffer( Transport *tpt, const u8 *buffer, int length )
{
  int n;
  struct exception e;
  TRANSPORT_VERIFY_OPEN;
  n = elua_net_send(tpt->fd, buffer, (elua_net_size)length);
  if (n == 0) {
    e.errnum = ERR_EOF;
    e.type = nonfatal;
    Throw( e );
  }
  if (n < 0) {
    e.errnum = elua_sock_err(tpt->fd);
    e.type = fatal;
    Throw( e );
  }
}

// Check if data is available on connection without reading:
// 		- 1 = data available, 0 = no data available
int transport_readable (Transport *tpt)
{
  return 1; // no really easy way to check this unless platform support is added
}

#endif
