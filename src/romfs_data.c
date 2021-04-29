// All compile-time ROMFS data is keps in this file

#include "romfs_common_defs.h"
#include "romfiles.h"

const FSDATA romfs_fsdata =
{
  ( u8* )romfiles_fs,
  ROMFS_FS_FLAG_DIRECT | ROMFS_FS_FLAG_READY_READ,
  NULL,
  NULL,
  sizeof( romfiles_fs )
};
