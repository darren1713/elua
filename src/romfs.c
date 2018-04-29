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
#ifdef ELUA_SIMULATOR
#include "hostif.h"
#endif
#include "platform_conf.h"
#if defined( BUILD_ROMFS ) || defined( BUILD_WOFS )

#define TOTAL_MAX_FDS   8
// DO NOT CHANGE THE ROMFS ALIGNMENT.
// UNLESS YOU _LIKE_ TO WATCH THE WORLD BURN.
#define ROMFS_ALIGN     4

#define fsmin( x , y ) ( ( x ) < ( y ) ? ( x ) : ( y ) )

static FD fd_table[ TOTAL_MAX_FDS ];
static int romfs_num_fd;
#ifdef ELUA_CPU_LINUX
static int wofs_sim_fd;
#define WOFS_FNAME    "/tmp/wofs.dat"
#define WOFS_SIZE     (256 * 1024)
#endif

#define WOFS_END_MARKER_CHAR  0xFF
#define WOFS_DEL_FIELD_SIZE   ( ROMFS_ALIGN )
#define WOFS_FILE_DELETED     0xAA

// Length of the 'file size' field for both ROMFS/WOFS
#define ROMFS_SIZE_LEN        4

#ifdef ROMFS_SECURE_FILENAMES_WITH_CHAR
static int romfs_security_lock = 1;

void romfs_sec_lock()
{
  romfs_security_lock = 1;
}

void romfs_sec_unlock()
{
  romfs_security_lock = 0;
}
#endif

static int romfs_check_open_fds()
{
  int i;
  
  for( i = 0; i < TOTAL_MAX_FDS; i ++ )
    if( fd_table[ i ].baseaddr != 0xFFFFFFFF &&
        fd_table[ i ].offset != 0xFFFFFFFF &&
        fd_table[ i ].size != 0xFFFFFFFF )
      return 1;
  return 0;
}

static int romfs_find_empty_fd()
{
  int i;
  
  for( i = 0; i < TOTAL_MAX_FDS; i ++ )
    if( fd_table[ i ].baseaddr == 0xFFFFFFFF &&
        fd_table[ i ].offset == 0xFFFFFFFF &&
        fd_table[ i ].size == 0xFFFFFFFF )
      return i;
  return -1;
}

static void romfs_close_fd( int fd )
{
  memset( fd_table + fd, 0xFF, sizeof( FD ) );
  fd_table[ fd ].flags = 0;
}

// Helper function: read a byte from the FS
static u8 romfsh_read8( u32 addr, const FSDATA *pfs )
{
  u8 temp;
  if( pfs->flags & ROMFS_FS_FLAG_DIRECT )
    return pfs->pbase[ addr ];
  pfs->readf( &temp, addr, 1, pfs );
  return temp;
}

// Helper function: return 1 if PFS reffers to a WOFS, 0 otherwise
static int romfsh_is_wofs( const FSDATA* pfs )
{
  return ( pfs->flags & ROMFS_FS_FLAG_WO ) != 0;
}

// Open the given file, returning one of FS_FILE_NOT_FOUND, FS_FILE_ALREADY_OPENED
// or FS_FILE_OK
static u8 romfs_open_file( const char* fname, FD* pfd, FSDATA *pfs, u32 *plast, u32 *pnameaddr )
{
  u32 i, j, n;
  char fsname[ DM_MAX_FNAME_LENGTH + 1 ];
  u32 fsize;
  int is_deleted;

#ifdef ROMFS_SECURE_FILENAMES_WITH_CHAR
  if(romfs_security_lock && strstr(fname,ROMFS_SECURE_FILENAMES_WITH_CHAR) != NULL)
    return FS_FILE_NOT_FOUND;
#endif

  
  // Look for the file
  i = 0;
  while( 1 )
  {
    if( romfsh_read8( i, pfs ) == WOFS_END_MARKER_CHAR )
    {
      *plast = i;
      return FS_FILE_NOT_FOUND;
    }
    // Read file name
    n = i;
    for( j = 0; j < DM_MAX_FNAME_LENGTH; j ++ )
    {
      fsname[ j ] = romfsh_read8( i + j, pfs );
      if( fsname[ j ] == 0 )
         break;
    }
    // ' i + j' now points at the '0' byte
    j = i + j + 1;
    // Round to a multiple of ROMFS_ALIGN
    j = ( j + ROMFS_ALIGN - 1 ) & ~( ROMFS_ALIGN - 1 );
    // WOFS has an additional WOFS_DEL_FIELD_SIZE bytes before the size as an indication for "file deleted"
    if( romfsh_is_wofs( pfs ) )
    {
      is_deleted = romfsh_read8( j, pfs ) == WOFS_FILE_DELETED;
      j += WOFS_DEL_FIELD_SIZE;
    }
    else
      is_deleted = 0;
    // And read the size
    fsize = romfsh_read8( j, pfs ) + ( romfsh_read8( j + 1, pfs ) << 8 );
    fsize += ( romfsh_read8( j + 2, pfs ) << 16 ) + ( romfsh_read8( j + 3, pfs ) << 24 );
    
    if( (( u64 )j + ( u64 )ROMFS_SIZE_LEN + ( u64 )fsize) > ( u64 )pfs->max_size )
    {
      /* POSSIBLE BUG HERE: When opening a new file for writing, this function scans past the end of the fs
      and triggers this flag to be set and then we cannot do anything with the filesystem :(
      if( romfs_fs_is_flag_set( pfs, ROMFS_FS_FLAG_READY_WRITE ) )
      {
        fprintf(stderr, "[ERROR] Open File too long, making filesystem readonly\n");
        romfs_fs_clear_flag( pfs, ROMFS_FS_FLAG_READY_WRITE );
      }*/
      *plast = i;
      return FS_FILE_NOT_FOUND;
    }

    j += ROMFS_SIZE_LEN;
    if( !strncasecmp( fname, fsname, DM_MAX_FNAME_LENGTH ) && !is_deleted )
    {
      // Found the file
      pfd->baseaddr = j;
      pfd->offset = 0;
      pfd->size = fsize;
      if( pnameaddr )
        *pnameaddr = n;
      return FS_FILE_OK;
    }
    // Move to next file
    i = j + fsize;
    // On WOFS, all file names must begin at a multiple of ROMFS_ALIGN
    if( romfsh_is_wofs( pfs ) )
      i = ( i + ROMFS_ALIGN - 1 ) & ~( ROMFS_ALIGN - 1 );
  }
  *plast = 0;
  return FS_FILE_NOT_FOUND;
}

static int romfs_open_r( struct _reent *r, const char *path, int flags, int mode, void *pdata )
{
  printf("Ro %s\n", path);
  FD tempfs;
  int i;
  FSDATA *pfsdata = ( FSDATA* )pdata;
  int must_create = 0;
  int exists;
  u8 lflags = ROMFS_FILE_FLAG_READ;
  u32 firstfree, nameaddr;

  if( romfs_num_fd == TOTAL_MAX_FDS )
  {
    r->_errno = ENFILE;
    return -1;
  }

  // Is the FS ready for reading
  if( !romfs_fs_is_flag_set( pfsdata, ROMFS_FS_FLAG_READY_READ ) )
  {
    r->_errno = EBUSY;
    return -1;
  }

  // Does the file exist?
  exists = romfs_open_file( path, &tempfs, pfsdata, &firstfree, &nameaddr ) == FS_FILE_OK;
  // Now interpret "flags" to set file flags and to check if we should create the file
  if( flags & O_CREAT )
  {
    // If O_CREAT is specified with O_EXCL and the file already exists, return with error
    if( ( flags & O_EXCL ) && exists )
    {
      r->_errno = EEXIST;
      return -1;
    }
    // Otherwise create the file if it does not exist
    must_create = !exists;
  }
  if( ( flags & O_TRUNC ) && ( flags & ( O_WRONLY | O_RDWR ) ) && exists )
  {
    // The file exists, but it must be truncated
    // In the case of WOFS, this effectively means "create a new file"
    must_create = 1;
  }
  // ROMFS can't create files
  if( must_create && ( ( pfsdata->flags & ROMFS_FS_FLAG_WO ) == 0 ) )
  {
    r->_errno = EROFS;
    return -1;
  }
  // Decode access mode
  if( flags & O_WRONLY )
    lflags = ROMFS_FILE_FLAG_WRITE;
  else if( flags & O_RDWR )
    lflags = ROMFS_FILE_FLAG_READ | ROMFS_FILE_FLAG_WRITE;
  if( flags & O_APPEND )
    lflags |= ROMFS_FILE_FLAG_APPEND;
  // If a write access is requested when the file must NOT be created, this
  // is an error
  if( ( lflags & ( ROMFS_FILE_FLAG_WRITE | ROMFS_FILE_FLAG_APPEND ) ) && !must_create )
  {
    r->_errno = EACCES;
    return -1;
  }
  if( ( lflags & ( ROMFS_FILE_FLAG_WRITE | ROMFS_FILE_FLAG_APPEND ) ) && 
      ( romfs_fs_is_flag_set( pfsdata, ROMFS_FS_FLAG_WRITING ) || !romfs_fs_is_flag_set( pfsdata, ROMFS_FS_FLAG_READY_WRITE ) ) )
  {
    // At most one file can be opened in write mode at any given time on WOFS
    r->_errno = EROFS;
    return -1;
  }
  // Do we need to create the file ?
  if( must_create )
  {
    if( exists )
    {
      // Invalidate the file first by changing WOFS_DEL_FIELD_SIZE bytes before
      // the file length to WOFS_FILE_DELETED
      u8 tempb[] = { WOFS_FILE_DELETED, 0xFF, 0xFF, 0xFF };
      pfsdata->writef( tempb, tempfs.baseaddr - ROMFS_SIZE_LEN - WOFS_DEL_FIELD_SIZE, WOFS_DEL_FIELD_SIZE, pfsdata );
    }
    // Find the last available position by asking romfs_open_file to look for a file
    // with an invalid name
    romfs_open_file( "\1", &tempfs, pfsdata, &firstfree, NULL );
    // Is there enough space on the FS for another file?
    if( pfsdata->max_size - firstfree + 1 < strlen( path ) + 1 + WOFS_MIN_NEEDED_SIZE + WOFS_DEL_FIELD_SIZE )
    {
      r->_errno = ENOSPC; // hook point for when to GC
      return -1;
    }

    // Make sure we can get a file descriptor before writing
    if( ( i = romfs_find_empty_fd() ) < 0 )
    {
      r->_errno = ENFILE;
      return -1;
    }

    // Write the name of the file
    pfsdata->writef( path, firstfree, strlen( path ) + 1, pfsdata );
    firstfree += strlen( path ) + 1; // skip over the name
    // Align to a multiple of ROMFS_ALIGN
    firstfree = ( firstfree + ROMFS_ALIGN - 1 ) & ~( ROMFS_ALIGN - 1 );
    firstfree += ROMFS_SIZE_LEN + WOFS_DEL_FIELD_SIZE; // skip over the size and the deleted flags area
    tempfs.baseaddr = firstfree;
    tempfs.offset = tempfs.size = 0;
    // Set the "writing" flag on the FS to indicate that there is a file opened in write mode
    romfs_fs_set_flag( pfsdata, ROMFS_FS_FLAG_WRITING );
  }
  else // File must exist (and was found in the previous 'romfs_open_file' call)
  {
    if( !exists )
    {
      r->_errno = ENOENT;
      return -1;
    }

    if( ( i = romfs_find_empty_fd() ) < 0 )
    {
      r->_errno = ENFILE;
      return -1;
    }
  }
  // Copy the descriptor information
  tempfs.flags = lflags;
  memcpy( fd_table + i, &tempfs, sizeof( FD ) );
  romfs_num_fd ++;
  return i;
}

static int romfs_close_r( struct _reent *r, int fd, void *pdata )
{
  printf("Rc%i\n",fd);
  FD* pfd = fd_table + fd;
  FSDATA *pfsdata = ( FSDATA* )pdata;
  u8 temp[ ROMFS_SIZE_LEN ];

  if( pfd->flags & ( ROMFS_FILE_FLAG_WRITE | ROMFS_FILE_FLAG_APPEND ) )
  {
    // Write back the size
    temp[ 0 ] = pfd->size & 0xFF;
    temp[ 1 ] = ( pfd->size >> 8 ) & 0xFF;
    temp[ 2 ] = ( pfd->size >> 16 ) & 0xFF;
    temp[ 3 ] = ( pfd->size >> 24 ) & 0xFF;
    pfsdata->writef( temp, pfd->baseaddr - ROMFS_SIZE_LEN, ROMFS_SIZE_LEN, pfsdata );
    // Clear the "writing" flag on the FS instance to allow other files to be opened
    // in write mode
    romfs_fs_clear_flag( pfsdata, ROMFS_FS_FLAG_WRITING );
  }
  romfs_close_fd( fd );
  romfs_num_fd --;
  return 0;
}

static _ssize_t romfs_write_r( struct _reent *r, int fd, const void* ptr, size_t len, void *pdata )
{
  printf("Rw\n");
  FD* pfd = fd_table + fd;
  FSDATA *pfsdata = ( FSDATA* )pdata;

  if( ( pfd->flags & ( ROMFS_FILE_FLAG_WRITE | ROMFS_FILE_FLAG_APPEND ) ) == 0 )
  {
    r->_errno = EINVAL;
    return -1;
  }
  // Append mode: set the file pointer to the end
  if( pfd->flags & ROMFS_FILE_FLAG_APPEND )
    pfd->offset = pfd->size;
  // Only write at the end of the file!
  if( pfd->offset != pfd->size )
    return 0;
  // Check if we have enough space left on the device. Always keep 1 byte for the final 0xFF
  // and ROMFS_ALIGN - 1 bytes for aligning the contents of the file data in the worst case
  // scenario (so ROMFS_ALIGN bytes in total)
  if( pfd->baseaddr + pfd->size + len > pfsdata->max_size - ROMFS_ALIGN )
    len = pfsdata->max_size - ( pfd->baseaddr + pfd->size ) - ROMFS_ALIGN;
  pfsdata->writef( ptr, pfd->offset + pfd->baseaddr, len, pfsdata );
  pfd->offset += len;
  pfd->size += len;
  return len;
}

static _ssize_t romfs_read_r( struct _reent *r, int fd, void* ptr, size_t len, void *pdata )
{
  printf("Rr\n");
  FD* pfd = fd_table + fd;
  long actlen = fsmin( len, pfd->size - pfd->offset );
  FSDATA *pfsdata = ( FSDATA* )pdata;

  if( ( pfd->flags & ROMFS_FILE_FLAG_READ ) == 0 )
  {
    r->_errno = EBADF;
    return -1;
  }
  if( pfsdata->flags & ROMFS_FS_FLAG_DIRECT )
  {
    printf("Rrmc#p:%li,l:%i,a:%li,o:%li\n", (u32)ptr, len, actlen, pfd->offset);
    memcpy( ptr, pfsdata->pbase + pfd->offset + pfd->baseaddr, actlen );
    int i;
    for(i=0;i<actlen;i++)
      printf("%02X",((char *)ptr)[i]);
    printf("\n");
  } else {
    actlen = pfsdata->readf( ptr, pfd->offset + pfd->baseaddr, actlen, pfsdata );
    printf("Rrrf#p:%li,l:%i,a:%li,o:%li\n", (u32)ptr, len, actlen, pfd->offset);
  }
  pfd->offset += actlen;
  return actlen;
}

// lseek
static off_t romfs_lseek_r( struct _reent *r, int fd, off_t off, int whence, void *pdata )
{
  printf("Rl o:%li w:%i\n", off, whence);
  FD* pfd = fd_table + fd;
  u32 newpos = 0;
  
  switch( whence )
  {
    case SEEK_SET:
      newpos = off;
      break;
      
    case SEEK_CUR:
      newpos = pfd->offset + off;
      break;
      
    case SEEK_END:
      newpos = pfd->size + off;
      break;
      
    default:
      return -1;
  }    
  if( newpos > pfd->size )
    return -1;
  pfd->offset = newpos;
  printf("Rl o:%li w:%i p:%li\n", off, whence, newpos);
  return newpos;
}

// Directory operations
static u32 romfs_dir_data = 0;

// opendir
static void* romfs_opendir_r( struct _reent *r, const char* dname, void *pdata )
{
  printf("Rod\n");
  if( !dname || strlen( dname ) == 0 || ( strlen( dname ) == 1 && !strcmp( dname, "/" ) ) )
  {
    romfs_dir_data = 0;
    return &romfs_dir_data;
  }
  return NULL;
}

// readdir
extern struct dm_dirent dm_shared_dirent;
extern char dm_shared_fname[ DM_MAX_FNAME_LENGTH + 1 ];
static struct dm_dirent* romfs_readdir_r( struct _reent *r, void *d, void *pdata )
{
  printf("Rrd\n");
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
}

// closedir
static int romfs_closedir_r( struct _reent *r, void *d, void *pdata )
{
  *( u32* )d = 0;
  return 0;
}

// getaddr
static const char* romfs_getaddr_r( struct _reent *r, int fd, void *pdata )
{
  FD* pfd = fd_table + fd;
  FSDATA *pfsdata = ( FSDATA* )pdata;

  if( pfsdata->flags & ROMFS_FS_FLAG_DIRECT )
  {
    printf("Rgetaddr %li\n", (u32)((char* )pfsdata->pbase + pfd->baseaddr));
    return ( const char* )pfsdata->pbase + pfd->baseaddr;
  } else {
    printf("Rgetaddr NULL\n");
    return NULL;
  }
}

// unlink
static int romfs_unlink_r( struct _reent *r, const char *path, void *pdata )
{
  printf("Ru\n");
  FSDATA *pfsdata = ( FSDATA* )pdata;
  FD tempfs;
  int exists;
  u32 firstfree, nameaddr;
  
  // Does the file exist?
  exists = romfs_open_file( path, &tempfs, pfsdata, &firstfree, &nameaddr ) == FS_FILE_OK;
  if( exists && romfsh_is_wofs( pfsdata ) )
  {
    u8 tempb[] = { WOFS_FILE_DELETED, 0xFF, 0xFF, 0xFF };
    pfsdata->writef( tempb, tempfs.baseaddr - ROMFS_SIZE_LEN - WOFS_DEL_FIELD_SIZE, WOFS_DEL_FIELD_SIZE, pfsdata );
    return 0;
  }
  else
    return -1;
}

// ****************************************************************************
// Our ROMFS device descriptor structure
// These functions apply to both ROMFS and WOFS

static const DM_DEVICE romfs_device = 
{
  romfs_open_r,         // open
  romfs_close_r,        // close
  romfs_write_r,        // write
  romfs_read_r,         // read
  romfs_lseek_r,        // lseek
  romfs_opendir_r,      // opendir
  romfs_readdir_r,      // readdir
  romfs_closedir_r,     // closedir
  romfs_getaddr_r,      // getaddr
  NULL,                 // mkdir
  romfs_unlink_r,       // unlink
  NULL,                 // rmdir
  NULL                  // rename
};

// ****************************************************************************
// ROMFS instance descriptor

static const FSDATA romfs_fsdata =
{
  ( u8* )romfiles_fs,
  ROMFS_FS_FLAG_DIRECT | ROMFS_FS_FLAG_READY_READ,
  NULL,
  NULL,
  sizeof( romfiles_fs )
};

// ****************************************************************************
// WOFS functions and instance descriptor for the simulator (testing)

#if defined( ELUA_CPU_LINUX ) && defined( BUILD_WOFS )
static u32 sim_wofs_read( void *to, u32 fromaddr, u32 size, const void *pdata )
{
  hostif_lseek( wofs_sim_fd, ( long )fromaddr, SEEK_SET );
  return hostif_read( wofs_sim_fd, to, size );
}

static u32 sim_wofs_write( const void *from, u32 toaddr, u32 size, const void *pdata )
{
  hostif_lseek( wofs_sim_fd, ( long )toaddr, SEEK_SET );
  return hostif_write( wofs_sim_fd, from, size );
}

// This must NOT be a const!
static FSDATA wofs_sim_fsdata =
{
  NULL,
  ROMFS_FS_FLAG_WO,
  sim_wofs_read,
  sim_wofs_write,
  WOFS_SIZE
};

// WOFS formatting function
// Returns 1 if OK, 0 for error
int wofs_format( void )
{
  unsigned i;

  hostif_lseek( wofs_sim_fd, 0, SEEK_SET );
  u8 temp = WOFS_END_MARKER_CHAR;
  for( i = 0; i < WOFS_SIZE; i ++ )
    hostif_write( wofs_sim_fd, &temp, 1 );
  return 1;
}

#endif // #ifdef ELUA_CPU_LINUX

// ****************************************************************************
// WOFS functions and instance descriptor for real hardware

#if defined( BUILD_WOFS ) && !defined( ELUA_CPU_LINUX )
static u32 sim_wofs_write( const void *from, u32 toaddr, u32 size, const void *pdata )
{
  const FSDATA *pfsdata = ( const FSDATA* )pdata;

  toaddr += ( u32 )pfsdata->pbase;
  return platform_flash_write( from, toaddr, size );
}

// This must NOT be a const!
static FSDATA wofs_fsdata =
{
  NULL,
  ROMFS_FS_FLAG_WO | ROMFS_FS_FLAG_DIRECT,
  NULL,
  sim_wofs_write,
  0
};

// WOFS formatting function
// Returns 1 if OK, 0 for error
int wofs_format( void )
{
  u32 sect_first, sect_last;
  FD tempfd;

  platform_flash_get_first_free_block_address( &sect_first );

  // Check if filesystem is marked as ready for writing
  if( romfs_fs_is_flag_set( &wofs_fsdata, ROMFS_FS_FLAG_READY_WRITE ) )
  {
    // If filesystem is OK
    // Get the first free address in WOFS. We use this address to compute the last block that we need to
    // erase, instead of simply erasing everything from sect_first to the last Flash page. 
    romfs_open_file( "\1", &tempfd, &wofs_fsdata, &sect_last, NULL );
    sect_last = platform_flash_get_sector_of_address( sect_last + ( u32 )wofs_fsdata.pbase );
  }
  else
  {
    // If filesystem has been marked as unwriteable, erase all sectors
    sect_last = platform_flash_get_num_sectors() - 1;
  }

  while( sect_first <= sect_last )
    if( platform_flash_erase_sector( sect_first ++ ) == PLATFORM_ERR )
      return 0;
  return 1;
}

// Returns file info (0=end of FS, )
int romfs_walk_fs( u32 *start, u32 *end, void *pdata  )
{
  FSDATA *pfsdata = ( FSDATA* )pdata;
  u32 off = *start;
  u32 fsize;
  int is_deleted;

  if( romfsh_read8( off, pfsdata ) == WOFS_END_MARKER_CHAR )
    return -1;

  //Skip over file name that ends with \0
  while( ( romfsh_read8( off ++, pfsdata ) ) != '\0' );

  // Move to next aligned position
  off = ( off + ROMFS_ALIGN - 1 ) & ~( ROMFS_ALIGN - 1 );

  // If WOFS, check if file is marked as deleted
  if( romfsh_is_wofs( pfsdata ) )
  {
    is_deleted = romfsh_read8( off, pfsdata ) == WOFS_FILE_DELETED;
    off += WOFS_DEL_FIELD_SIZE;
  }
  else
    is_deleted = 0;

  fsize = romfsh_read8( off, pfsdata ) + ( romfsh_read8( off + 1, pfsdata ) << 8 );
  fsize += ( romfsh_read8( off + 2, pfsdata ) << 16 ) + ( romfsh_read8( off + 3, pfsdata ) << 24 );

  if( (( u64 )off + ( u64 )ROMFS_SIZE_LEN + ( u64 )fsize) > ( u64 )pfsdata->max_size )
  {
    if( romfs_fs_is_flag_set( pfsdata, ROMFS_FS_FLAG_READY_WRITE ) )
    {
      fprintf(stderr, "[ERROR] Dir File too long, making filesystem readonly\n");
      romfs_fs_clear_flag( pfsdata, ROMFS_FS_FLAG_READY_WRITE );
    }
    return -1;
  }

  // Jump offset ahead by length field & file size
  off += ROMFS_SIZE_LEN;
  off += fsize - 1;

  // Point to last byte in file
  *end = off;

  // Return whether file is deleted or not
  if( is_deleted )
    return 1;
  else
    return 0;
}

// FIXME: Needs to be adjusted to support non-uniform sector sizes
#define LAST_SECTOR_NUM ( platform_flash_get_num_sectors() - 1 )
#define LAST_SECTOR_END  ( INTERNAL_FLASH_SIZE - INTERNAL_FLASH_START_ADDRESS )


// POSIX.1-2001 Sample Implementation of rand & srand
// Using these to prevent changes to libc random number generator
static unsigned long next = 1;
static int myrand(void)  /* RAND_MAX assumed to be 32767. */
{
    next = next * 1103515245 + 12345;
    return((unsigned)(next/65536) % 32768);
}

static void mysrand(unsigned seed)
{
    next = seed;
}

// Sets bit in bit array
// Returns 1 if OK, 0 if out of range
int bit_array_set( u8* arr, u32 bitnum, u8 value )
{
  int byte = bitnum / 8;
  int bit = bitnum % 8;

  if( byte >= ( ( platform_flash_get_num_sectors() + ( 8 - 1 ) ) / 8 ) )
    return 0;

  if( value > 0)
    arr[byte] |= ( u8 )( 1 << bit );
  else
    arr[byte] &= ~( u8 )( 1 << bit );

  return 1;
}

int bit_array_get( u8* arr, u32 bitnum )
{
  int byte = bitnum / 8;
  int bit = bitnum % 8;

  if( byte >= ( ( platform_flash_get_num_sectors() + ( 8 - 1 ) ) / 8 ) )
    return -1;

  return ( ( arr[byte] & ( u8 )( 1 << bit ) ) > 0 );
}

// Returns lowest set bit, -1 if none found
int bit_array_get_lowest( u8* arr )
{
  int byte;
  int bit;
  u8 value;

  for( byte = 0; byte < ( ( platform_flash_get_num_sectors() + ( 8 - 1 ) ) / 8 ); byte++ )
  {
    if( arr[ byte ] )
    {
      bit = 0;
      value = arr[ byte ];
      while( !( value & 1) )
      {
        value >>= 1;
        ++bit;
      }
      return ( bit + byte * 8 );
    }
  }
  return -1;
}

// Returns highest set bit, -1 if none found
int bit_array_get_highest( u8* arr )
{
  int byte;
  int bit;
  u8 value;

  for( byte = ( ( platform_flash_get_num_sectors() + ( 8 - 1 ) ) / 8 ) - 1; byte >= 0; byte-- )
  {
    if( arr[ byte ] )
    {
      bit = 0;
      value = arr[ byte ];
      while (value >>= 1)
      {
        bit++;
      }
      return ( bit + byte * 8 );
    }
  }
  return -1;
}

#define WOFS_REPACK_DEBUG

// Erase a range of sectors and mark in freed sectors in bit array
// Returns 1 if OK, 0 for error
int wofs_repack_erase_sector_range(u32 start_sect, u32 end_sect, u8* freed_sectors )
{
  int i;
#if defined( WOFS_REPACK_DEBUG )
  printf("Sector Range: %lu - %lu\n",start_sect, end_sect );
#endif
  for( i = start_sect; i <= end_sect; i++ )
  {
    if( bit_array_get( freed_sectors, i ) == 0 )
    {
#if defined( WOFS_REPACK_DEBUG )
      printf("Erasing: %d\n", i);
#endif

      if( !bit_array_set(freed_sectors, i, 1) )
      {
        fprintf(stderr, "[ERROR] Sector out of range.");
        return 0;
      }

      if( platform_flash_erase_sector( i ) == PLATFORM_ERR )
      {
        fprintf(stderr, "[ERROR] Couldn't erase: %d", i);
        return 0;
      }
    }
  }
  return 1;
}


int wofs_repack_init_spare_sectors( u8* freed_sectors, u32* lowest_spare )
{
  FSDATA *pdata = &wofs_fsdata;
  FD tempfd;
  u32 sect_last;
  int i;

  // Clear freed sector list
  for( i = 0; i < ( ( platform_flash_get_num_sectors() + ( 8 - 1 ) ) / 8 ); i++ )
    freed_sectors[ i ] = 0;

  // Find the last wector we've written into
  romfs_open_file( "\1", &tempfd, pdata, &sect_last, NULL );
  *lowest_spare = platform_flash_get_sector_of_address( sect_last + ( u32 )wofs_fsdata.pbase ) + 1;

  // If filesystem hasn't been flagged as broken, mark sectors after FS as free
  if( romfs_fs_is_flag_set( pdata, ROMFS_FS_FLAG_READY_WRITE ) )
  {
    for( i = *lowest_spare; i <= LAST_SECTOR_NUM; i++ )
    {
      if( !bit_array_set(freed_sectors, i, 1) )
      {
        fprintf(stderr, "[ERROR] Sector out of range.");
        return 0;
      }
    }
  }
  return 1;
}

int wofs_free_sectors( void )
{
  u32 lowest_spare;
  u8 freed_sectors[ ( platform_flash_get_num_sectors() + ( 8 - 1 ) ) / 8 ];

  // Check filesystem and mark spare sectors as freed
  if( !wofs_repack_init_spare_sectors( freed_sectors, &lowest_spare ) )
    return 0;

  return ( platform_flash_get_num_sectors() - ( lowest_spare + 1 ) );
}

// Returns 1 if OK, 0 for error
int wofs_repack( u32 threshold )
{
  FSDATA *pdata = &wofs_fsdata;
  u8 freed_sectors[ ( platform_flash_get_num_sectors() + ( 8 - 1 ) ) / 8 ];
  u32 startf, endf, last_endf;
  u32 sstart, send, snum_startf, snum_endf, snum_startf2, snum_endf2;
  u32 snum_tmp_sect, sstart_tmp_sect;
  u32 write_ptr, tmp, last_sector, lowest_spare;
  u32 fs_read_ptr = 0;
  int ret;
  int i;

  if( romfs_check_open_fds() )
  {
    fprintf(stderr, "[ERROR] Can't repack with open files.");
    return 0;
  }

  // Check filesystem and mark spare sectors as freed
  if( !wofs_repack_init_spare_sectors( freed_sectors, &lowest_spare ) )
    return 0;

  if( ( ( lowest_spare + threshold + 1 ) < platform_flash_get_num_sectors() )  )
  {
#if defined( WOFS_REPACK_DEBUG )
    printf("Spare sectors: %lu.\n", platform_flash_get_num_sectors() - ( lowest_spare + 1 ) );
#endif
    return 1;
  }

  // Mark FS as unwriteable/unreadable while we work on it
  romfs_fs_clear_flag( pdata, ROMFS_FS_FLAG_READY_WRITE | ROMFS_FS_FLAG_READY_READ );

  // 1 - Find first of any deleted files
  endf = 0;
  startf = 0; // starting offset at beginning of fs
  do
  {
    startf = ( endf + 1 + ROMFS_ALIGN - 1 ) & ~( ROMFS_ALIGN - 1 );
    last_endf = endf; // Cache previous file's end for when we find an empty file
    ret = romfs_walk_fs( &startf, &endf, pdata );
#if defined( WOFS_REPACK_DEBUG )
    printf("S: %lu E: %lu F: %d\n", startf, endf, ret);
    //for(i=startf; i<endf; i++)
    //  printf("%02X", romfsh_read8( i, pdata ));
#endif
  } while( ret == 0 ); // Exit when we find something other than a "regular" file

  // If we get to the end and find no deleted files, end
  if( ret == -1 )
  {
#if defined( WOFS_REPACK_DEBUG )
    printf("No need to repack.\n");
#endif
    romfs_fs_set_flag( pdata, ROMFS_FS_FLAG_READY_READ | ROMFS_FS_FLAG_READY_WRITE );
    return 1;
  }

  mysrand( platform_timer_read( PLATFORM_TIMER_SYS_ID ) );

  while( ret != -1 ) // while we haven't hit the end of the FS
  {
    // 2 - Find sector erased file came from
    if( fs_read_ptr == 0 )
    {
      // Make sure free sectors are free
      wofs_repack_erase_sector_range( lowest_spare, LAST_SECTOR_NUM, freed_sectors );
#if defined( WOFS_REPACK_DEBUG )
      printf("Lowest: %d, Highest %d\n", bit_array_get_lowest( freed_sectors), bit_array_get_highest( freed_sectors));
#endif
      snum_endf = platform_flash_find_sector( endf + ( u32 )pdata->pbase, NULL, NULL );
      snum_startf = platform_flash_find_sector( startf + ( u32 )pdata->pbase, &sstart, &send );
      if( !wofs_repack_erase_sector_range(snum_startf + 1, snum_endf - 1, freed_sectors ) )
        return 0;
    }
    else
      snum_startf = platform_flash_find_sector( fs_read_ptr + ( u32 )pdata->pbase, &sstart, &send );

#if defined( WOFS_REPACK_DEBUG )
    printf("S: %lu E:%lu PB: %lu\n", sstart, send, ( u32 )pdata->pbase );
#endif
    sstart -= ( u32 )pdata->pbase;
    send -= ( u32 )pdata->pbase;
#if defined( WOFS_REPACK_DEBUG )
    printf("Source Sector: %lu\n", snum_startf );
#endif

    // 3 - start copying files until source sector is exhausted
    //JEFF: MIGHT BE LOCKING UP HERE ??IF WE WROTE LOTS OF DATA TO FILL ALL SECTORS??
    do
    {
      // Uniformity is skewed a bit towards lower numbers with this method
      snum_tmp_sect = ( myrand() % (LAST_SECTOR_NUM-lowest_spare+1) )+lowest_spare;
    } while( bit_array_get( freed_sectors, snum_tmp_sect ) == 0 );

    platform_flash_get_sector_range(snum_tmp_sect, &sstart_tmp_sect, NULL);
    // Copy data exactly up until deleted file
    write_ptr = sstart_tmp_sect - ( u32 )pdata->pbase; // Beginning of "spare" sector
    bit_array_set(freed_sectors, snum_tmp_sect, 0);
#if defined( WOFS_REPACK_DEBUG )
    printf("Tmp Sector: %lu\n", snum_tmp_sect );
#endif

    if( fs_read_ptr == 0 ) // First run
    {
      fs_read_ptr = sstart; // Beginning of first deleted file's sector
      tmp = last_endf + 1 - sstart;
    }
    else // Secondary runs, finish or continue file
    {
#if defined( WOFS_REPACK_DEBUG )
      printf("Ef: %lu, rptr: %lu, se: %lu\n", endf, fs_read_ptr, send);
#endif
      tmp = fsmin( endf + 1 - fs_read_ptr, 1024 ); // max out at end of current read sector
    }

    if( tmp > 1 ) // If initial file was deleted, tmp will be 1 and we should skip it
    {
#if defined( WOFS_REPACK_DEBUG )
      printf("CP F: %lu, T: %lu, L: %lu\n", fs_read_ptr, write_ptr, tmp);
#endif
      pdata->writef( ( u32* )(fs_read_ptr + ( u32 )pdata->pbase), write_ptr, tmp, pdata );
      write_ptr += tmp;
      fs_read_ptr += tmp;
    }

    // Fill out last sector from FS until source sector is exhausted or end of FS
    while( write_ptr <= ( sstart_tmp_sect - ( u32 )pdata->pbase ) + send - sstart && ret != -1 )
    {
      startf = ( endf + 1 + ROMFS_ALIGN - 1 ) & ~( ROMFS_ALIGN - 1 ); // Starting is next aligned chunk
      ret = romfs_walk_fs( &startf, &endf, pdata ); // find end of file & type

      // if we've discovered a non-deleted file, copy it
      if( ret == 0 )
      {
        fs_read_ptr = startf;
        write_ptr = ( write_ptr + ROMFS_ALIGN - 1 ) & ~( ROMFS_ALIGN - 1 );

        // Amount to write will be lesser of file length or remaining sector space
        tmp = fsmin( endf + 1 - fs_read_ptr, ( sstart_tmp_sect - ( u32 )pdata->pbase ) + ( send + 1 - sstart ) - write_ptr );
#if defined( WOFS_REPACK_DEBUG )
        printf("CP F: %lu, T: %lu, L: %lu, L2 %lu\n", fs_read_ptr, write_ptr, endf + 1 - fs_read_ptr, ( sstart_tmp_sect - ( u32 )pdata->pbase ) + ( send + 1 - sstart ) - write_ptr );
#endif
        pdata->writef( ( u32* )(fs_read_ptr + ( u32 )pdata->pbase), write_ptr, tmp, pdata);
        last_sector = platform_flash_get_sector_of_address( fs_read_ptr + ( u32 )pdata->pbase );
        fs_read_ptr += tmp; // Advance so that read ptr is correct when we exit this loop
        write_ptr += tmp;

        // erase last sector if we just exited it
        if( platform_flash_get_sector_of_address( fs_read_ptr + ( u32 )pdata->pbase ) > last_sector )
        {
          if( !wofs_repack_erase_sector_range(last_sector, last_sector, freed_sectors ) )
            return 0;
        }
      }
      else
      {
        // if we found a deleted file, erase sectors that ended during the duration of the file
        snum_endf2 = platform_flash_find_sector( endf + ( u32 )pdata->pbase, NULL, NULL );
        snum_startf2 = platform_flash_find_sector( startf + ( u32 )pdata->pbase, &sstart, &send );
        if( !wofs_repack_erase_sector_range(snum_startf2, snum_endf2 - 1, freed_sectors ) )
          return 0;
      }
    }

    // 4 - erase origin sector
    if( !wofs_repack_erase_sector_range(snum_startf, snum_startf, freed_sectors ) )
      return 0;


    // copy back to lowest available sector
    tmp = bit_array_get_lowest( freed_sectors );
    if( tmp < 0 )
    {
      fprintf(stderr, "[ERROR] No free sector");
      return 0;
    }

    platform_flash_get_sector_range(tmp, &sstart, &send);
#if defined( WOFS_REPACK_DEBUG )
    printf("S: %lu E:%lu PB: %lu\n", sstart, send, ( u32 )pdata->pbase );
#endif
    sstart -= ( u32 )pdata->pbase;
    send -= ( u32 )pdata->pbase;
#if defined( WOFS_REPACK_DEBUG )
    printf("Back Sector: %lu\n", snum_startf );
#endif
    bit_array_set(freed_sectors, tmp, 0);
    tmp = sstart_tmp_sect - ( u32 )pdata->pbase;
#if defined( WOFS_REPACK_DEBUG )
    printf("WF: %lu, SS: %lu, LN: %lu\n", tmp, sstart, write_ptr - ( sstart_tmp_sect - ( u32 )pdata->pbase));
#endif
    pdata->writef( ( u32* )(tmp + ( u32 )pdata->pbase), sstart, write_ptr - ( sstart_tmp_sect - ( u32 )pdata->pbase ), pdata);
    write_ptr = sstart + write_ptr - ( sstart_tmp_sect - ( u32 )pdata->pbase ); // Now points into origin sector


    // 5 - erase spare sector if only one available, otherwise we'll get it at the end
    if( bit_array_get_lowest( freed_sectors ) < 0 )
    {
      if( !wofs_repack_erase_sector_range( snum_tmp_sect, snum_tmp_sect, freed_sectors ) )
        return 0;
    }

    // 6 - Fill "source" sector from FS, if space remains
    // Start filling in unfinished files
    if( fs_read_ptr < endf && write_ptr < send && ret == 0 )
    {
      tmp = fsmin( endf + 1 - fs_read_ptr, send + 1 - write_ptr);
#if defined( WOFS_REPACK_DEBUG )
      printf("CP F: %lu, T: %lu, L: %lu, EF: %lu\n", fs_read_ptr, write_ptr, tmp, endf);
#endif
      pdata->writef( ( u32* )fs_read_ptr, write_ptr, tmp, pdata);
      last_sector = platform_flash_get_sector_of_address( fs_read_ptr + ( u32 )pdata->pbase );
      fs_read_ptr += tmp;
      write_ptr += tmp;
      if( platform_flash_get_sector_of_address( fs_read_ptr + ( u32 )pdata->pbase ) > last_sector )
      {
        if( !wofs_repack_erase_sector_range(last_sector, last_sector, freed_sectors ) )
          return 0;
      }
    }

    // Put more files if they will fit
    while( write_ptr <= send  && ret != -1 )
    {
      startf = ( endf + 1 + ROMFS_ALIGN - 1 ) & ~( ROMFS_ALIGN - 1 );
      ret = romfs_walk_fs( &startf, &endf, pdata ); // find end of file & type

      // if we've discovered a non-deleted file, copy it
      if( ret == 0 )
      {
        fs_read_ptr = startf;
        write_ptr = ( write_ptr + ROMFS_ALIGN - 1 ) & ~( ROMFS_ALIGN - 1 );

        // Amount to write will be lesser of file length or remaining sector space
        tmp = fsmin( endf + 1 - fs_read_ptr, ( sstart_tmp_sect - ( u32 )pdata->pbase ) + ( send + 1 - sstart ) - write_ptr );
#if defined( WOFS_REPACK_DEBUG )
        printf("CP F: %lu, T: %lu, L: %lu\n", fs_read_ptr, write_ptr, tmp);
#endif
        pdata->writef( ( u32* )(fs_read_ptr + ( u32 )pdata->pbase), write_ptr, tmp, pdata);
        last_sector = platform_flash_get_sector_of_address( fs_read_ptr + ( u32 )pdata->pbase );
        fs_read_ptr += tmp; // Advance so that read ptr is correct when we exit this loop
        write_ptr += tmp;

        // erase last sector if we just exited it
        if( platform_flash_get_sector_of_address( fs_read_ptr + ( u32 )pdata->pbase ) > last_sector )
        {
          if( !wofs_repack_erase_sector_range(last_sector, last_sector, freed_sectors ) )
            return 0;
        }
      }
      else
      {
        // if we found a deleted file, erase sectors that ended during the duration of the file
        snum_endf = platform_flash_find_sector( endf + ( u32 )pdata->pbase, NULL, NULL );
        snum_startf = platform_flash_find_sector( startf + ( u32 )pdata->pbase, &sstart, &send );
        if( !wofs_repack_erase_sector_range(snum_startf, snum_endf - 1, freed_sectors ) )
          return 0;
      }
    }

    // At the end fs_read_ptr: next spot in old FS
    // write_ptr should be just into next sector if we have any more work
  }

  // 7 - Clean Up
  // If end of FS was found in sector after last write pointer position, erase sector 
  // after write pointer up to where we were last looking for another file
  snum_startf = platform_flash_find_sector( write_ptr + ( u32 )pdata->pbase, &sstart, &send );
  if( !wofs_repack_erase_sector_range(snum_startf+1, LAST_SECTOR_NUM, freed_sectors ) )
    return 0;

  // Mark FS as ready
  romfs_fs_set_flag( pdata, ROMFS_FS_FLAG_READY_READ | ROMFS_FS_FLAG_READY_WRITE );
  return 1;
}



#endif // #ifdef BUILD_WOFS

// Initialize both ROMFS and WOFS as needed
int romfs_init( void )
{
  unsigned i;

  for( i = 0; i < TOTAL_MAX_FDS; i ++ )
  {
    memset( fd_table + i, 0xFF, sizeof( FD ) );
    fd_table[ i ].flags = 0;
  }
#if defined( ELUA_CPU_LINUX ) && defined( BUILD_WOFS )
  // Initialize and register WOFS for the simulator
  wofs_sim_fd = hostif_open( WOFS_FNAME, 2, 0666 ); // try to open directly first
  if( -1 == wofs_sim_fd )
  {
    wofs_sim_fd = hostif_open( WOFS_FNAME, 66, 0666) ; // 66 == O_RDWR | O_CREAT
    u8 temp = WOFS_END_MARKER_CHAR;
    for( i = 0; i < WOFS_SIZE; i ++ )
      hostif_write( wofs_sim_fd, &temp, 1 );
    printf( "SIM_WOFS: creating WOFS file\n" );
    hostif_close( wofs_sim_fd );
    wofs_sim_fd = hostif_open( WOFS_FNAME, 2, 0666 );
  }
  dm_register( "/wo", ( void* )&wofs_sim_fsdata, &romfs_device );
  romfs_fs_set_flag( &wofs_fsdata, ROMFS_FS_FLAG_READY_WRITE | ROMFS_FS_FLAG_READY_READ );
#endif // #if defined( ELUA_CPU_LINUX ) && defined( BUILD_WOFS )
#if defined( BUILD_WOFS ) && !defined( ELUA_CPU_LINUX )
  // Get the start address and size of WOFS and register it
  wofs_fsdata.pbase = ( u8* )platform_flash_get_first_free_block_address( NULL );
#ifdef INTERNAL_FLASH_SECTOR_SIZE
  wofs_fsdata.max_size = INTERNAL_FLASH_SIZE - ( ( u32 )wofs_fsdata.pbase - INTERNAL_FLASH_START_ADDRESS ) - INTERNAL_FLASH_SECTOR_SIZE;
#else // #ifdef INTERNAL_FLASH_SECTOR_SIZE
  wofs_fsdata.max_size = INTERNAL_FLASH_SIZE - ( ( u32 )wofs_fsdata.pbase - INTERNAL_FLASH_START_ADDRESS ) - INTERNAL_FLASH_SECTOR_ARRAY[ LAST_SECTOR_NUM ];
#endif // #ifdef INTERNAL_FLASH_SECTOR_SIZE
  dm_register( "/wo", &wofs_fsdata, &romfs_device );
  romfs_fs_set_flag( &wofs_fsdata, ROMFS_FS_FLAG_READY_WRITE | ROMFS_FS_FLAG_READY_READ );
#endif // ifdef BUILD_WOFS
#ifdef BUILD_ROMFS
  // Register the ROM filesystem
  dm_register( "/rom", ( void* )&romfs_fsdata, &romfs_device );
#endif // #ifdef BUILD_ROMFS

  return 0;
}

#else // #if defined( BUILD_ROMFS ) || defined( BUILD_WOFS )

int romfs_init( void )
{
  return dm_register( NULL, NULL, NULL );
}

#endif // #if defined( BUILD_ROMFS ) || defined( BUILD_WOFS )

