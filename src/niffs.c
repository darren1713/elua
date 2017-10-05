// Filesystem implementation
#include "romfs.h"
#include <string.h>
#include <errno.h>
#include "romfiles.h"
#include <stdio.h>
#include <stdlib.h>
#include "ioctl.h"
#include <fcntl.h>
#include "platform.h"
#include "platform_conf.h"
#include "niffs.h"

#if defined( BUILD_NIFFS ) 

//#define NFFS_DBG(...)              printf(__VA_ARGS__)
#ifndef NFFS_DBG
#define NFFS_DBG(...)
#endif

// FIXME: Needs to be adjusted to support non-uniform sector sizes
#define LAST_SECTOR_NUM ( platform_flash_get_num_sectors() - 1 )
#define LAST_SECTOR_END  ( INTERNAL_FLASH_SIZE - INTERNAL_FLASH_START_ADDRESS )

niffs fs;

#define NIFFS_BUF_SIZE           128
#define NIFFS_FILE_DESCS         4
static u8_t buf[NIFFS_BUF_SIZE];
static niffs_file_desc descs[NIFFS_FILE_DESCS];
static u8_t * niffs_pbase;
static u32_t niffs_max_size;
static u32_t niffs_sectors;

static int nffs_open_r( struct _reent *r, const char *path, int flags, int mode, void *pdata )
{
  u8 lflags = 0;
  int ret;
  
  if(flags == 0) {
    lflags = NIFFS_O_RDONLY;
    NFFS_DBG("-R");
  }
  if(flags & O_CREAT) {
    lflags |= NIFFS_O_CREAT;
    NFFS_DBG("-C");
  }
  if(flags & O_TRUNC) {
    lflags |= NIFFS_O_TRUNC;
    NFFS_DBG("-T");
  }
  if(flags & O_RDWR) {
    lflags |= NIFFS_O_RDWR;
    NFFS_DBG("-S");
  }
  if(flags & O_WRONLY) {
    lflags |= NIFFS_O_WRONLY;
    NFFS_DBG("-W");
  }
  if(flags & O_APPEND) {
    lflags |= NIFFS_O_APPEND;
    NFFS_DBG("-A");
  }
  if(flags & O_EXCL) {
    lflags |= NIFFS_O_EXCL;
    NFFS_DBG("-E");
  }
  ret = NIFFS_open(&fs, (char *)path, lflags, mode);
  NFFS_DBG("\nN_O:%s,%i,%i,%i,%i\n", path, ret, flags, lflags, mode);
  return ret;
}

static int nffs_close_r( struct _reent *r, int fd, void *pdata )
{
  return NIFFS_close(&fs, fd);
}

static _ssize_t nffs_write_r( struct _reent *r, int fd, const void* ptr, size_t len, void *pdata )
{
  return NIFFS_write(&fs, fd, (void *)ptr, len);
}

static _ssize_t nffs_read_r( struct _reent *r, int fd, void* ptr, size_t len, void *pdata )
{
  return NIFFS_read(&fs, fd, ptr, len);
}

// lseek
static off_t nffs_lseek_r( struct _reent *r, int fd, off_t off, int whence, void *pdata )
{
  return NIFFS_lseek(&fs, fd, off, whence);
}

// Directory operations
//static u32 romfs_dir_data = 0;


niffs_DIR d;
// opendir
static void* nffs_opendir_r( struct _reent *r, const char* dname, void *pdata )
{
  return NIFFS_opendir(&fs, (char *)dname, &d);
}
/*
NIFFS_readdir
// readdir
extern struct dm_dirent dm_shared_dirent;
extern char dm_shared_fname[ DM_MAX_FNAME_LENGTH + 1 ];
static struct dm_dirent* romfs_readdir_r( struct _reent *r, void *d, void *pdata )
{
  u32 off = *( u32* )d;
  struct dm_dirent *pent = &dm_shared_dirent;
  unsigned j;
  FSDATA *pfsdata = ( FSDATA* )pdata;
  int is_deleted;
 
  while( 1 )
  {
    // Check if we're at the end of the FS
    if( romfsh_read8( off, pfsdata ) == WOFS_END_MARKER_CHAR )
      return NULL;

    // Read filename
    j = 0;
    while( ( dm_shared_fname[ j ++ ] = romfsh_read8( off ++, pfsdata ) ) != '\0' );
    pent->fname = dm_shared_fname;

    // Move to next aligned offset after name
    off = ( off + ROMFS_ALIGN - 1 ) & ~( ROMFS_ALIGN - 1 );

    // If WOFS, check if file is marked as deleted
    if( romfsh_is_wofs( pfsdata ) )
    {
      is_deleted = romfsh_read8( off, pfsdata ) == WOFS_FILE_DELETED;
      off += WOFS_DEL_FIELD_SIZE;
    }
    else
      is_deleted = 0;

    // Get file size file and pack into fsize
    pent->fsize = romfsh_read8( off, pfsdata ) + ( romfsh_read8( off + 1, pfsdata ) << 8 );
    pent->fsize += ( romfsh_read8( off + 2, pfsdata ) << 16 ) + ( romfsh_read8( off + 3, pfsdata ) << 24 );
    
    if( (( u64 )off + ( u64 )ROMFS_SIZE_LEN + ( u64 )pent->fsize) > ( u64 )pfsdata->max_size )
    {
      if( romfs_fs_is_flag_set( pfsdata, ROMFS_FS_FLAG_READY_WRITE ) &&
          !( romfs_fs_is_flag_set( pfsdata, ROMFS_FS_FLAG_WRITING ) && ( u64 )pent->fsize == 0xFFFFFFFF ) )
      {
        fprintf(stderr, "[ERROR] Read File too long, making filesystem readonly\n");
        romfs_fs_clear_flag( pfsdata, ROMFS_FS_FLAG_READY_WRITE );
      }
      return NULL;
    }
    
    // fill in file time & flags
    pent->ftime = 0;
    pent->flags = 0;

    // Jump offset ahead by length field & file size
    off += ROMFS_SIZE_LEN;
    off += pent->fsize;


    // If WOFS, also advance offset by deleted file field
    if( romfsh_is_wofs( pfsdata ) )
      off = ( off + ROMFS_ALIGN - 1 ) & ~( ROMFS_ALIGN - 1 );

    //If file security is enabled, don't return a matching filename as valid
#ifdef ROMFS_SECURE_FILENAMES_WITH_CHAR
    if( !is_deleted && ( romfs_security_lock == 0 || strstr( pent->fname, ROMFS_SECURE_FILENAMES_WITH_CHAR) == NULL ) )
      break;
#else
    if( !is_deleted )
      break;
#endif    
  }
  *( u32* )d = off;
  return pent;
}*/

// closedir
static int nffs_closedir_r( struct _reent *r, void *d, void *pdata )
{
  *( u32* )d = 0;
  return 0;
}

// unlink
static int nffs_unlink_r( struct _reent *r, const char *path, void *pdata )
{
  return NIFFS_remove(&fs, (char *)path);
}



static const DM_DEVICE niffs_device = 
{
  nffs_open_r,         // open
  nffs_close_r,        // close
  nffs_write_r,        // write
  nffs_read_r,         // read
  nffs_lseek_r,        // lseek
  nffs_opendir_r,      // opendir
  NULL,      // readdir //TODO: COMPLICATED TRANSFORM FROM romfs_readdir_r
  nffs_closedir_r,     // closedir
  NULL,                 // getaddr
  NULL,                 // mkdir
  nffs_unlink_r,       // unlink
  NULL,                 // rmdir
  NULL                  // rename
};


static int platform_hal_erase_f(u8_t *addr, u32_t len) {
/*  if (addr < &_flash[0]) return ERR_NIFFS_TEST_BAD_ADDR;
  if (addr+len > &_flash[0] + EMUL_SECTORS * EMUL_SECTOR_SIZE) return ERR_NIFFS_TEST_BAD_ADDR;
  if ((addr - &_flash[0]) % EMUL_SECTOR_SIZE) return ERR_NIFFS_TEST_BAD_ADDR;
  if (len != EMUL_SECTOR_SIZE) return ERR_NIFFS_TEST_BAD_ADDR;*/
  platform_flash_erase_sector(((u32_t)addr)/INTERNAL_FLASH_SECTOR_SIZE); // == PLATFORM_OK;
  NFFS_DBG("N_E:%li,%li\n", (u32_t)addr, len);
  return NIFFS_OK;
}

static int platform_hal_write_f(u8_t *addr, u8_t *src, u32_t len) {
  //TODO: Write actual data to addresses here
  //  toaddr += ( u32 )pfsdata->pbase;
  //platform_flash_write( const void *from, u32 toaddr, u32 size )
  platform_flash_write( src, (u32_t)addr, len );
  NFFS_DBG("N_W:%li,%li,%li\n", (u32_t)addr, (u32_t)src, len);
  return NIFFS_OK;
}

int nffs_init( void )
{
  niffs_pbase = ( u8* )platform_flash_get_first_free_block_address( NULL );
  niffs_max_size = INTERNAL_FLASH_SIZE - ( ( u32 )niffs_pbase - INTERNAL_FLASH_START_ADDRESS ) - (INTERNAL_FLASH_SECTOR_SIZE);
  niffs_sectors = niffs_max_size / INTERNAL_FLASH_SECTOR_SIZE;

  NIFFS_init(&fs, niffs_pbase, niffs_sectors, INTERNAL_FLASH_SECTOR_SIZE, 128,
      buf, sizeof(buf),
      descs, NIFFS_FILE_DESCS,
      platform_hal_erase_f, platform_hal_write_f);

  //Check filesystem, and format if it has not been formatted or corrupted...
  if(NIFFS_chk(&fs) == ERR_NIFFS_NOT_A_FILESYSTEM)
    NIFFS_format(&fs);

  NIFFS_mount(&fs);

  dm_register( "/f", &fs, &niffs_device );
  //romfs_fs_set_flag( &fs, ROMFS_FS_FLAG_READY_WRITE | ROMFS_FS_FLAG_READY_READ );

  return dm_register( NULL, NULL, NULL );
}

int nffs_format()
{
  NFFS_DBG("N_I:%li,%li,%li\n", (u32_t)niffs_pbase, niffs_max_size, niffs_sectors); //N_I:36000,39936,39N_E:20007C84,1024N_W:20007C84,20007C80,8
  return NIFFS_format(&fs);
}

int nffs_mount()
{
  return NIFFS_mount(&fs);
}

int nffs_unmount()
{
  return NIFFS_unmount(&fs);
}

int nffs_check()
{
  return NIFFS_chk(&fs);
}

int nffs_info(s32_t *total, s32_t *used, u8_t *overflow)
{
  //Check filesystem, and format if it has not been formatted or corrupted...
  NIFFS_unmount(&fs);
  if(NIFFS_chk(&fs) == ERR_NIFFS_NOT_A_FILESYSTEM)
  {
    NFFS_DBG("chk NOT FS\n");
    NIFFS_format(&fs);
  } else
    NFFS_DBG("chk OK\n");

  if(NIFFS_mount(&fs))
    NFFS_DBG("MNT FAIL\n");
  else
    NFFS_DBG("MNT OK\n");

  return NIFFS_info(&fs, total, used, overflow);
}

#else // #if defined( BUILD_ROMFS ) || defined( BUILD_WOFS )

int nffs_init( void )
{
  return dm_register( NULL, NULL, NULL );
}

#endif // #if defined( BUILD_NIFFS ) 

