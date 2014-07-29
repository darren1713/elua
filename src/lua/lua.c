/*
** $Id: lua.c,v 1.160.1.2 2007/12/28 15:32:23 roberto Exp $
** Lua stand-alone interpreter
** See Copyright Notice in lua.h
*/


#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define lua_c

#include "lua.h"

#include "lauxlib.h"
#include "lualib.h"
#include "lvm.h"

#ifndef LUA_RPC
#include "platform_conf.h"
#include "platform.h"
#endif

static lua_State *globalL = NULL;

static const char *progname = LUA_PROGNAME;

#ifdef EXTRA_EVENT_HOOK
extern void extras_event_loop( void );
#endif

static void lstop (lua_State *L, lua_Debug *ar) {
  (void)ar;  /* unused arg. */
  lua_sethook(L, NULL, 0, 0);
  luaL_error(L, "interrupted!");
}


static void laction (int i) {
  signal(i, SIG_DFL); /* if another SIGINT happens before lstop,
                              terminate process (default action) */
  lua_sethook(globalL, lstop, LUA_MASKCALL | LUA_MASKRET | LUA_MASKCOUNT, 1);
}


static void print_usage (void) {
  fprintf(stderr,
  "usage: %s [options] [script [args]].\n"
  "Available options are:\n"
  "  -e stat  execute string " LUA_QL("stat") "\n"
  "  -l name  require library " LUA_QL("name") "\n"
  "  -m limit set memory limit. (units are in Kbytes)\n"
  "  -i       enter interactive mode after executing " LUA_QL("script") "\n"
  "  -v       show version information\n"
  "  --       stop handling options\n"
  "  -        execute stdin and stop handling options\n"
  ,
  progname);
  fflush(stderr);
}


static void l_message (const char *pname, const char *msg) {
  if (pname) fprintf(stderr, "%s: ", pname);
  fprintf(stderr, "%s\n", msg);
  fflush(stderr);
}


static int report (lua_State *L, int status) {
  if (status && !lua_isnil(L, -1)) {
    const char *msg = lua_tostring(L, -1);
    if (msg == NULL) msg = "(error object is not a string)";
    l_message(progname, msg);
    lua_pop(L, 1);
  }
  return status;
}


static int traceback (lua_State *L) {
  if (!lua_isstring(L, 1))  /* 'message' not a string? */
    return 1;  /* keep it intact */
  lua_getfield(L, LUA_GLOBALSINDEX, "debug");
  if (!lua_istable(L, -1) && !lua_isrotable(L, -1)) {
    lua_pop(L, 1);
    return 1;
  }
  lua_getfield(L, -1, "traceback");
  if (!lua_isfunction(L, -1) && !lua_islightfunction(L, -1)) {
    lua_pop(L, 2);
    return 1;
  }
  lua_pushvalue(L, 1);  /* pass error message */
  lua_pushinteger(L, 2);  /* skip this function and traceback */
  lua_call(L, 2, 1);  /* call debug.traceback */
  return 1;
}


static int docall (lua_State *L, int narg, int clear) {
  int status;
  int base = lua_gettop(L) - narg;  /* function index */
  lua_pushcfunction(L, traceback);  /* push traceback function */
  lua_insert(L, base);  /* put it under chunk and args */
  signal(SIGINT, laction);
  status = lua_pcall(L, narg, (clear ? 0 : LUA_MULTRET), base);
  signal(SIGINT, SIG_DFL);
  lua_remove(L, base);  /* remove traceback function */
  /* force a complete garbage collection in case of errors */
  if (status != 0) lua_gc(L, LUA_GCCOLLECT, 0);
  return status;
}


static void print_version (void) {
  l_message(NULL, LUA_RELEASE "  " LUA_COPYRIGHT);
}


static int getargs (lua_State *L, char **argv, int n) {
  int narg;
  int i;
  int argc = 0;
  while (argv[argc]) argc++;  /* count total number of arguments */
  narg = argc - (n + 1);  /* number of arguments to the script */
  luaL_checkstack(L, narg + 3, "too many arguments to script");
  for (i=n+1; i < argc; i++)
    lua_pushstring(L, argv[i]);
  lua_createtable(L, narg, n + 1);
  for (i=0; i < argc; i++) {
    lua_pushstring(L, argv[i]);
    lua_rawseti(L, -2, i - n);
  }
  return narg;
}


static int dofile (lua_State *L, const char *name) {
  int status = luaL_loadfile(L, name) || docall(L, 0, 1);
  return report(L, status);
}


static int dostring (lua_State *L, const char *s, const char *name) {
  int status = luaL_loadbuffer(L, s, strlen(s), name) || docall(L, 0, 1);
  return report(L, status);
}


static int dolibrary (lua_State *L, const char *name) {
  lua_getglobal(L, "require");
  lua_pushstring(L, name);
  return report(L, docall(L, 1, 1));
}


static const char *get_prompt (lua_State *L, int firstline) {
  const char *p;
  lua_getfield(L, LUA_GLOBALSINDEX, firstline ? "_PROMPT" : "_PROMPT2");
  p = lua_tostring(L, -1);
  if (p == NULL) p = (firstline ? LUA_PROMPT : LUA_PROMPT2);
  lua_pop(L, 1);  /* remove global */
  return p;
}


static int incomplete (lua_State *L, int status) {
  if (status == LUA_ERRSYNTAX) {
    size_t lmsg;
    const char *msg = lua_tolstring(L, -1, &lmsg);
    const char *tp = msg + lmsg - (sizeof(LUA_QL("<eof>")) - 1);
    if (strstr(msg, LUA_QL("<eof>")) == tp) {
      lua_pop(L, 1);
      return 1;
    }
  }
  return 0;  /* else... */
}

int spin_vm( lua_State *L )
{
#if 0
  //char *buf = "local function a () end a()";
  char *buf = "";
  int n = lua_gettop(L);
  luaL_loadbuffer(L, buf, strlen(buf), "=stdin");
  lua_pcall (L, 0, 0, 0);
  lua_settop(L, n);
#else
  lua_lock(L);
  luaD_callhook( L, LUA_HOOKCOUNT, -1 );
  lua_unlock(L);
#endif
  return 0;
}

// #define LUA_COMMAND_STACK_MAX 8
// volatile int lua_command_stack[LUA_COMMAND_STACK_MAX];
// volatile int lua_command_top = 0;
// volatile int lua_command_bottom = 0;

// //Push commands onto the stack to be processed when the cons
// int lua_command_push( char * data )
// {
//   if((lua_command_top + 1) % LUA_COMMAND_STACK_MAX == lua_command_bottom )
//   {
//     //Going to overrun ring buffer
//     return -1;
//   }
//   else if(strlen(data) < (LUA_MAXINPUT + 2) )
//   {
//     //char * store = malloc( strlen(data)+1 );
//     //strncpy(store, data, LUA_MAXINPUT);
//     lua_command_stack[lua_command_top] = (int)data;
//     lua_command_top = (lua_command_top + 1) % LUA_COMMAND_STACK_MAX;
//     return 1;
//   }
//   return -1;
// }

// char * lua_command_pop( )
// {
//   if( lua_command_top == lua_command_bottom )
//     return NULL;
//   else
//   {
//     char * ret = (char *)lua_command_stack[lua_command_bottom];
//     lua_command_bottom = (lua_command_bottom + 1) % LUA_COMMAND_STACK_MAX;
//     return(ret);
//   }
// }

static char * lua_command_buf = NULL;
static void (*c_command)() = NULL;

int c_command_enqueue( void (*command)() )
{
  if( c_command != NULL )
    return -1;

  c_command = command;
  return 0;
}

int c_command_run( void )
{
  if( c_command != NULL )
  {
    c_command();
    c_command = NULL;
  }

  return 0;
}

int c_command_pending ( void )
{
  if( c_command != NULL )
    return 1;
  else
    return 0;
}

int lua_command_enqueue( const char * buf, size_t len )
{
  if( lua_command_buf != NULL )
    return -1;

  lua_command_buf = ( char * )malloc( sizeof( char ) * len + 1 );

  if( lua_command_buf == NULL )
    return -1;

  memcpy( lua_command_buf, buf, len );
  lua_command_buf[ len ] = 0;

  return 0;
}

int lua_command_run( lua_State *L )
{

  if( lua_command_buf != NULL )
  {
    int error = luaL_loadbuffer(L, lua_command_buf, strlen( lua_command_buf ), "=queue") || lua_pcall(L, 0, 0, 0);
    if( error )
    {
      fprintf (stderr, "%s", lua_tostring (L, -1));
      lua_pop (L, 1);
    }  
    free( lua_command_buf );
    lua_command_buf = NULL;

    return error;
  }
  return 0;
}

int lua_command_pending ( void )
{
  if( lua_command_buf != NULL )
    return 1;
  else
    return 0;
}

#if defined( LUA_RPC )
// void setup_pipe( void )
// {
//   int fd = fileno(stdin);  
//   int flags;
//   flags = fcntl(fd, F_GETFL, 0); 
//   flags |= O_NONBLOCK; 
//   fcntl(fd, F_SETFL, flags); 
// }

// int is_key_pressed(void)
// {
//      struct timeval tv;
//      fd_set fds;
//      tv.tv_sec = 1;
//      tv.tv_usec = 0;

//      FD_ZERO(&fds);
//      FD_SET(STDIN_FILENO, &fds); 

//      select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
//      return FD_ISSET(STDIN_FILENO, &fds);
// }

// int slip_readline(lua_State *L, char *b, char *p)
// {
//   char *t = b;

//   setup_pipe();

//   while( fgets(b, LUA_MAXINPUT, stdin) == NULL )
//   {
//     spin_vm(L);
//     is_key_pressed();
//   }
// }
#define slip_readline(L,b,p) \
  ((void)L, \
  fgets(b, LUA_MAXINPUT, stdin) != NULL)  /* get line */
#else
int prev_line_end = -1;
#include "utils.h"

int slip_readline(lua_State *L, char *b, const char *p)
{
  char *ptr = b;
  int c;
  int i = 0;

  while( 1 )
  {
    c = platform_uart_recv( CON_UART_ID, PLATFORM_TIMER_SYS_ID, 0 );

    if( c != -1 )
    {
      //Check if our last character plus current character are a combination line ending
      //Skip this character if true, if not, clear last character
      if( ( prev_line_end + c ) == ( '\r' + '\n' ) )
      {
        prev_line_end = -1;
          continue;
      }
      else if( prev_line_end != -1 )
        prev_line_end = -1;
  
      if( ( c == 8 ) || ( c == 0x7F ) ) // Backspace
      {
        if( i > 0 )
        {
          i --;
          platform_uart_send( CON_UART_ID, 8 );
          platform_uart_send( CON_UART_ID, ' ' );
          platform_uart_send( CON_UART_ID, 8 );
        }
        continue;
      }
      if( !isprint( c ) && c != '\n' && c != '\r' && c != STD_CTRLZ_CODE )
        continue;
      if( c == STD_CTRLZ_CODE )
        return 0;
      platform_uart_send( CON_UART_ID, c );
      if( c == '\r' || c == '\n' )
      {
        // Handle both '\r\n' and '\n\r' here
        prev_line_end = c;
        platform_uart_send( CON_UART_ID, '\r' + '\n' - c );
        ptr[ i ++ ] = '\n';
        ptr[ i ] = 0;
        return i + 1;
      }
      ptr[ i ++ ] = c;

      //Watch our buffer and reset if necessary
      if(i+2 == LUA_MAXINPUT)
      {
          platform_uart_send( CON_UART_ID, '^' );
          platform_uart_send( CON_UART_ID, '\r' );
          platform_uart_send( CON_UART_ID, '\n' );
          i = 0;
          ptr[ i ] = 0;
      }
    }
    else
    {
      lua_command_run(L);
      c_command_run();
#ifdef EXTRA_EVENT_HOOK
      extras_event_loop();
#endif
      spin_vm(L);
    }
    // {
    //   char * data;
    //   if( (data = lua_command_pop() ) == NULL)
    //     spin_vm(L);
    //   else
    //   {
    //     strncpy(ptr, data, LUA_MAXINPUT);
    //     //free(data);
    //     //Make sure last character is a zero in case of bad string
    //     ptr[LUA_MAXINPUT-1] = 0;
    //     i = 0;
    //     prev_line_end = -1;
    //     if(strlen(ptr) > 0)
    //       return(strlen(ptr) + 1);
    //   }
    // }
  }
}
#endif



static int pushline (lua_State *L, int firstline) {
  char buffer[LUA_MAXINPUT];
  char *b = buffer;
  size_t l;
  const char *prmt = get_prompt(L, firstline);
  fputs(prmt, stdout);
  fflush(stdout);
  if (slip_readline(L, b, prmt) == 0)
    return 0;  /* no input */
  l = strlen(b);
  if (l > 0 && b[l-1] == '\n')  /* line ends with newline? */
    b[l-1] = '\0';  /* remove it */
  if (firstline && b[0] == '=')  /* first line starts with `=' ? */
    lua_pushfstring(L, "return %s", b+1);  /* change it to `return' */
  else
    lua_pushstring(L, b);
  lua_freeline(L, b);
  return 1;
}


static int loadline (lua_State *L) {
  int status;
  lua_settop(L, 0);
  if (!pushline(L, 1))
    return -1;  /* no input */
  for (;;) {  /* repeat until gets a complete line */
    status = luaL_loadbuffer(L, lua_tostring(L, 1), lua_strlen(L, 1), "=stdin");
    if (!incomplete(L, status)) break;  /* cannot try to add lines? */
    if (!pushline(L, 0))  /* no more input? */
      return -1;
    lua_pushliteral(L, "\n");  /* add a new line... */
    lua_insert(L, -2);  /* ...between the two lines */
    lua_concat(L, 3);  /* join them */
  }
  lua_saveline(L, 1);
  lua_remove(L, 1);  /* remove line */
  return status;
}


static void dotty (lua_State *L) {
  int status;
  const char *oldprogname = progname;
  progname = NULL;
  while ((status = loadline(L)) != -1) {
    if (status == 0) status = docall(L, 0, 0);
    report(L, status);
    if (status == 0 && lua_gettop(L) > 0) {  /* any result to print? */
      lua_getglobal(L, "print");
      lua_insert(L, 1);
      if (lua_pcall(L, lua_gettop(L)-1, 0, 0) != 0)
        l_message(progname, lua_pushfstring(L,
                               "error calling " LUA_QL("print") " (%s)",
                               lua_tostring(L, -1)));
    }
  }
  lua_settop(L, 0);  /* clear stack */
  fputs("\n", stdout);
  fflush(stdout);
  progname = oldprogname;
}


static int handle_script (lua_State *L, char **argv, int n) {
  int status;
  const char *fname;
  int narg = getargs(L, argv, n);  /* collect arguments */
  lua_setglobal(L, "arg");
  fname = argv[n];
  if (strcmp(fname, "-") == 0 && strcmp(argv[n-1], "--") != 0) 
    fname = NULL;  /* stdin */
  status = luaL_loadfile(L, fname);
  lua_insert(L, -(narg+1));
  if (status == 0)
    status = docall(L, narg, 0);
  else
    lua_pop(L, narg);      
  return report(L, status);
}


/* check that argument has no extra characters at the end */
#define notail(x)	{if ((x)[2] != '\0') return -1;}


static int collectargs (char **argv, int *pi, int *pv, int *pe) {
  int i;
  for (i = 1; argv[i] != NULL; i++) {
    if (argv[i][0] != '-')  /* not an option? */
        return i;
    switch (argv[i][1]) {  /* option */
      case '-':
        notail(argv[i]);
        return (argv[i+1] != NULL ? i+1 : 0);
      case '\0':
        return i;
      case 'i':
        notail(argv[i]);
        *pi = 1;  /* go through */
      case 'v':
        notail(argv[i]);
        *pv = 1;
        break;
      case 'e':
        *pe = 1;  /* go through */
      case 'm':   /* go through */
      case 'l':
        if (argv[i][2] == '\0') {
          i++;
          if (argv[i] == NULL) return -1;
        }
        break;
      default: return -1;  /* invalid option */
    }
  }
  return 0;
}


static int runargs (lua_State *L, char **argv, int n) {
  int i;
  for (i = 1; i < n; i++) {
    if (argv[i] == NULL) continue;
    lua_assert(argv[i][0] == '-');
    switch (argv[i][1]) {  /* option */
      case 'e': {
        const char *chunk = argv[i] + 2;
        if (*chunk == '\0') chunk = argv[++i];
        lua_assert(chunk != NULL);
        if (dostring(L, chunk, "=(command line)") != 0)
          return 1;
        break;
      }
      case 'm': {
        const char *limit = argv[i] + 2;
        int memlimit=0;
        if (*limit == '\0') limit = argv[++i];
        lua_assert(limit != NULL);
        memlimit = atoi(limit);
        lua_gc(L, LUA_GCSETMEMLIMIT, memlimit);
        break;
      }
      case 'l': {
        const char *filename = argv[i] + 2;
        if (*filename == '\0') filename = argv[++i];
        lua_assert(filename != NULL);
        if (dolibrary(L, filename))
          return 1;  /* stop if file fails */
        break;
      }
      default: break;
    }
  }
  return 0;
}


static int handle_luainit (lua_State *L) {
  const char *init = getenv(LUA_INIT);
  if (init == NULL) return 0;  /* status OK */
  else if (init[0] == '@')
    return dofile(L, init+1);
  else
    return dostring(L, init, "=" LUA_INIT);
}


struct Smain {
  int argc;
  char **argv;
  int status;
};


static int pmain (lua_State *L) {
  struct Smain *s = (struct Smain *)lua_touserdata(L, 1);
  char **argv = s->argv;
  int script;
  int has_i = 0, has_v = 0, has_e = 0;
  globalL = L;
  if (argv[0] && argv[0][0]) progname = argv[0];
  lua_gc(L, LUA_GCSTOP, 0);  /* stop collector during initialization */
  luaL_openlibs(L);  /* open libraries */
  lua_gc(L, LUA_GCRESTART, 0);
  s->status = handle_luainit(L);
  if (s->status != 0) return 0;
  script = collectargs(argv, &has_i, &has_v, &has_e);
  if (script < 0) {  /* invalid args? */
    print_usage();
    s->status = 1;
    return 0;
  }
  if (has_v) print_version();
  s->status = runargs(L, argv, (script > 0) ? script : s->argc);
  if (s->status != 0) return 0;
  if (script)
    s->status = handle_script(L, argv, script);
  if (s->status != 0) return 0;
  if (has_i)
    dotty(L);
  else if (script == 0 && !has_e && !has_v) {
    if (lua_stdin_is_tty()) {
      print_version();
      dotty(L);
    }
    else dofile(L, NULL);  /* executes stdin as a file */
  }
  return 0;
}

#ifdef LUA_RPC
int main (int argc, char **argv) {
#else
int lua_main (int argc, char **argv) {
#endif
  int status;
  struct Smain s;
  lua_State *L = lua_open();  /* create state */
  if (L == NULL) {
    l_message(argv[0], "cannot create state: not enough memory");
    return EXIT_FAILURE;
  }
  s.argc = argc;
  s.argv = argv;
  status = lua_cpcall(L, &pmain, &s);
  report(L, status);
  lua_close(L);
  return (status || s.status) ? EXIT_FAILURE : EXIT_SUCCESS;
}

