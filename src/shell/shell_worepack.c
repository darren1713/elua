// Shell: 'worepack' implementation

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "shell.h"
#include "common.h"
#include "type.h"
#include "platform_conf.h"
#include "romfs.h"

#ifdef BUILD_WOFS

const char shell_help_worepack[] = "\n"
  "Repack WOFS, minimizing filesystem usage\n";
const char shell_help_summary_worepack[] = "WOFS repack";

void shell_worepack( int argc, char **argv )
{
  if( argc != 1 )
  {
    SHELL_SHOW_HELP( worepack );
    return;
  }
  printf( "Repacking ..." );
  if( !wofs_repack( 255 ) )
  {
    printf( "\ni*** ERROR ***: unable to repack the internal flash. WOFS might be compromised.\n" );
    printf( "It is advised to re-flash the eLua image.\n" );
  }
  else
    printf( " done.\n" );
}

#else // #ifdef BUILD_WOFS

const char shell_help_worepack[] = "";
const char shell_help_summary_worepack[] = "";

void shell_worepack( int argc, char **argv )
{
  shellh_not_implemented_handler( argc, argv );
}

#endif // #ifdef BUILD_WOFS

