// Read-only ROM filesystem

#ifndef __FS_H__
#define __FS_H__

#include "type.h"
#include "devman.h"
#include "romfs_common_defs.h"
#include "platform_conf.h"

/*******************************************************************************
The Read-Only "filesystem" resides in a contiguous zone of memory, with the
following structure (repeated for each file):

Filename: ASCIIZ, max length is DM_MAX_FNAME_LENGTH, first byte is 0xFF if last file
File size: (4 bytes), aligned to ROMFS_ALIGN bytes 
File data: (file size bytes)

The WOFS (Write Once File System) uses much of the ROMFS functions, thuss it is
also implemented in romfs.c. It resides in a contiguous zone of memory, with a
structure that is quite similar with ROMFS' structure (repeated for each file):

Filename: ASCIIZ, max length is DM_MAX_FNAME_LENGTH, first byte is 0xFF if last file.
          WOFS filenames always begin at an address which is a multiple of ROMFS_ALIGN.
File deleted flag: (WOFS_DEL_FIELD_SIZE bytes), aligned to ROMFS_ALIGN bytes
File size: (4 bytes), aligned to ROMFS_ALIGN bytes
File data: (file size bytes)

*******************************************************************************/

enum
{
  FS_FILE_NOT_FOUND,
  FS_FILE_OK
};

// File flags
#define ROMFS_FILE_FLAG_READ      0x01
#define ROMFS_FILE_FLAG_WRITE     0x02
#define ROMFS_FILE_FLAG_APPEND    0x04

// A small "FILE" structure
typedef struct 
{
  u32 baseaddr;
  u32 offset;
  u32 size;
  u8 flags;
} FD;

// WOFS constants
// The miminum size we need in order to create another file
// This size will be added to the size of the filename when creating a new file
// to ensure that there's enough space left on the device
// This comes from the size of the file length field (4) + the maximum number of
// bytes needed to align this field (3) + a single 0xFF byte which marks the end
// of the filesystem (1) + the maximum number of bytes needed to align the contents 
// of a file (3)
#define WOFS_MIN_NEEDED_SIZE      11   

#define romfs_fs_set_flag( p, f )     (p)->flags |= ( f )
#define romfs_fs_clear_flag( p, f )   (p)->flags &= ( u8 )~( f )
#define romfs_fs_is_flag_set( p, f )  ( ( (p)->flags & ( f ) ) != 0 )

// FS functions
int romfs_init( void );
int wofs_format( void );

#ifdef ROMFS_SECURE_FILENAMES_WITH_CHAR
void romfs_sec_lock();
void romfs_sec_unlock();
#endif

#endif

