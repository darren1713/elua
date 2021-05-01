// Common definition for the ROM file system

#ifndef __ROMFS_COMMON_DEFS_H__
#define __ROMFS_COMMON_DEFS_H__

#include "type.h"

// Filesystem flags
#define ROMFS_FS_FLAG_DIRECT      0x01    // direct mode (the file is mapped in a memory area directly accesible by the CPU)
#define ROMFS_FS_FLAG_WO          0x02    // this FS is actually a WO (Write-Once) FS
#define ROMFS_FS_FLAG_WRITING     0x04    // for WO only: there is already a file opened in write mode
#define ROMFS_FS_FLAG_READY_WRITE 0x08    // for WO only: filesystem consistency OK for writing
#define ROMFS_FS_FLAG_READY_READ  0x10    // for WO only: filesystem consistency OK for reading

// ROMFS/WOFS functions
typedef u32 ( *p_fs_read )( void *to, u32 fromaddr, u32 size, const void *pdata );
typedef u32 ( *p_fs_write )( const void *from, u32 toaddr, u32 size, const void *pdata );

// File system descriptor
typedef struct
{
  u8 *pbase;                      // pointer to FS base in memory (only for ROMFS_FS_FLAG_DIRECT)
  u8 flags;                       // flags (see above)
  p_fs_read readf;                // pointer to read function (for non-direct mode FS)
  p_fs_write writef;              // pointer to write function (only for ROMFS_FS_FLAG_WO)
  u32 max_size;                   // maximum size of the FS (in bytes)
} FSDATA;

#endif
