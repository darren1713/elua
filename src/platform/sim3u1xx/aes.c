#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "lrotable.h"
#include "common.h"
#include "sermux.h"
#include "platform_conf.h"
#include "auxmods.h"
#include "elua_int.h"
#include "buf.h"
#include "aes.h"
#include "auxmods.h"
#include "stacks.h"
#include "type.h"
#include "settings.h"
#include "SI32_AES_A_Support.h"
#include "SI32_AES_A_Type.h"
#include "sim3u1xx.h"
#include "log.h"
#include "aes_config.h"
#include "romfs.h"
#include "gsattrack.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


void aes_error_handler(void)
{
  SI32_AES_A_clear_error_interrupt(SI32_AES_0);
   logconsole(LOG_LEVEL_BASIC, LOG_MODULE_AES, "Encrypt error\n");
}

static int aes_ccm_encode_message( lua_State *L )
{
  luaL_Buffer b;
  uint8_t key[32];
  size_t len;
  const uint8_t *data;
  uint8_t nonce[GSATTRACK_CCM_NONCE_LEN];
  uint8_t i;

  for(i = 0; i < sizeof(nonce); i++)
  {
    while( buf_get_count( BUF_ID_RNG, 0 ) == 0 );
    buf_read(BUF_ID_RNG, 0, ( t_buf_data* )&nonce[i] );
  }

  romfs_sec_unlock();
  len = param_get( "~key", "", PARAM_SECURE, (uint8_t *)key, 32);
  romfs_sec_lock();

  if(len != 32)
  {
    logconsole(LOG_LEVEL_BASIC, LOG_MODULE_AES, "T#invalidKey=%d", len);
    return 0;
  }

  data = (uint8_t *) luaL_checklstring ( L, 1, &len);

  unsigned char *encoded_message = aes256_ccm_encrypt(data, len, key, 32, nonce, sizeof(nonce), GSATTRACK_CCM_TAG_LEN );
  luaL_buffinit(L,&b);
  luaL_addlstring(&b,( const char * )nonce, sizeof( nonce ));
  luaL_addlstring(&b,( const char * )encoded_message, len + GSATTRACK_CCM_TAG_LEN );
  luaL_pushresult( &b );
  free(encoded_message);
  return 1;
}

static int aes_ccm_decode_message( lua_State *L )
{
  luaL_Buffer b;
  uint8_t key[32];
  size_t len;
  size_t message_len;
  const uint8_t *data;

  romfs_sec_unlock();
  len = param_get( "~key", "", PARAM_SECURE, (uint8_t *)key, 32);
  romfs_sec_lock();

  if(len != 32)
  {
    logconsole(LOG_LEVEL_BASIC, LOG_MODULE_AES, "T#invalidKey=%d", len);
    return 0;
  }

  data = ( uint8_t * )luaL_checklstring ( L, 1, &len);

  unsigned char *decoded_message = aes256_ccm_decrypt( data, len, key, 32, GSATTRACK_CCM_NONCE_LEN, GSATTRACK_CCM_TAG_LEN, &message_len );
  
  if( decoded_message != NULL )
  {
    luaL_buffinit(L,&b);
    luaL_addlstring(&b,(const char *)decoded_message,len-GSATTRACK_CCM_TAG_LEN-GSATTRACK_CCM_NONCE_LEN);
    luaL_pushresult( &b );
    free(decoded_message);
    return 1;
  }
  else
  {
    return 0;
  }

}

unsigned char * aes256_ccm_encrypt(const unsigned char *data, size_t data_len, const unsigned char *key, size_t key_len, const unsigned char *nonce, size_t nonce_len, size_t tag_len )
{
  aes256_cbc_context ctxcbc;
  aes256_ctr_context ctxctr;
  unsigned char *encoded_message;
  unsigned char tag_buf[16];
  unsigned char ctr_buf[16];
  int i;
  uint8_t cur_len;

  // Only handle 256-bit keys for nowsd
  if( key_len != 32 )
  {
    printf("wrong key len, %u", key_len);
    return NULL;
  }

  // TODO: Add check that l(m) will have enough bytes

  // Tag len must be 4,6,8,10,12,14,16
  if( ( tag_len % 2 != 0 ) || ( ( tag_len / 2 ) < 2 ) || ( ( tag_len / 2 ) > 8 ) )
  {
    printf("wrong tag len: %u", tag_len);
    return NULL;
  }

  if( ( nonce_len > 13 ) || (nonce_len < 7 ) )
  {
    printf("wrong nonce len: %u", nonce_len);
    return NULL;
  }

  encoded_message = malloc( data_len + tag_len );

  if( encoded_message == NULL )
  {
    return NULL;
  }

  // Generate CBC-MAC Tag
  // Flags for B0
  tag_buf[0] = 0;
  tag_buf[0] |= ( ( ( tag_len - 2 ) / 2 ) & 7 ) << 3;
  tag_buf[0] |= ( ( 15 - nonce_len ) - 1 ) & 7;

  // Fill in nonce
  memcpy(&tag_buf[1], nonce, nonce_len );

  // Encode l(m) in MSB order
  for( i = 15; i > nonce_len; i-- )
  {
    if( ( 15 - i ) >= ( sizeof( data_len ) ) )
      tag_buf[ i ] = 0;
    else
      tag_buf[ i ] =  ( ( data_len >> ( 8 * ( 15 - i ) ) ) & 0xff );
  }

  aes256_cbc_init(&ctxcbc, key, NULL );
  aes256_cbc_encrypt(&ctxcbc, ( void * )tag_buf, 16);

  for( i = 0; i < ( ( data_len + (16 - 1) ) / 16 ); i++)
  {
    cur_len = ( (data_len - i * 16 ) >= 16 ) ? 16 : (data_len - i * 16 );
    memset((void *)tag_buf, 0, 16);
    memcpy((void *)tag_buf, ( (uint8_t *)data ) + i * 16, cur_len );

    aes256_cbc_encrypt(&ctxcbc, ( void * )tag_buf, 16);
  }

  aes256_cbc_done(&ctxcbc);

  // Generte CTR Encrypted Message
  ctr_buf[0] = 0;
  ctr_buf[0] |= ( ( 15 - nonce_len ) - 1 ) & 7;

  // Fill in nonce
  memcpy(&ctr_buf[1], nonce, nonce_len );
  for( i = 15; i > nonce_len; i-- )
  {
    ctr_buf[ i ] =  0;
  }

  // Initialize CTR Mode
  aes256_ctr_init( &ctxctr, key, ctr_buf );

  // Get our authentication value
  aes256_ctr_encrypt( &ctxctr, tag_buf, 16 );
  memcpy( encoded_message + data_len, tag_buf, tag_len );
  memcpy( encoded_message, data, data_len );
  aes256_ctr_encrypt( &ctxctr, encoded_message, data_len );

  aes256_ctr_done( &ctxctr );

  return encoded_message;
}


unsigned char * aes256_ccm_decrypt(const unsigned char *data, size_t data_len, const unsigned char *key, size_t key_len, size_t nonce_len, size_t tag_len, size_t *message_len )
{
  aes256_cbc_context ctxcbc;
  aes256_ctr_context ctxctr;
  unsigned char *decoded_message;
  unsigned char tag_buf[16];
  unsigned char ctr_buf[16];
  int i;
  uint8_t cur_len;

  // Only handle 256-bit keys for now
  if( key_len != 32 )
  {
    printf("wrong key len, %lu", ( unsigned long )key_len);
    return NULL;
  }

  // TODO: Add check that l(m) will have enough bytes

  // Tag len must be 4,6,8,10,12,14,16
  if( ( tag_len % 2 != 0 ) || ( ( tag_len / 2 ) < 2 ) || ( ( tag_len / 2 ) > 8 ) )
  {
    printf("wrong tag len, %lu", ( unsigned long )tag_len);
    return NULL;
  }

  if( ( nonce_len > 13 ) || (nonce_len < 7 ) )
  {
    printf("wrong nonce len: %u", nonce_len);
    return NULL;
  }

  if( data_len <= tag_len - nonce_len )
  {
    printf("wrong nonce len: %u", nonce_len);
    return NULL;
  }

  *message_len = data_len - tag_len - nonce_len;
  decoded_message = malloc( *message_len );


  if( decoded_message == NULL )
  {
    return NULL;
  }

  // Generte CTR Encrypted Message
  ctr_buf[0] = 0;
  ctr_buf[0] |= ( ( 15 - nonce_len ) - 1 ) & 7;

  // Fill in nonce
  memcpy(&ctr_buf[1], data, nonce_len );
  for( i = 15; i > nonce_len; i-- )
  {
    ctr_buf[ i ] =  0;
  }

  // Initialize CTR Mode
  aes256_ctr_init( &ctxctr, key, ctr_buf );

  memcpy(ctr_buf, data + nonce_len + *message_len, tag_len );
  for( i = 15; i >= tag_len; i-- )
  {
    ctr_buf[ i ] =  0;
  }

  // Get our authentication value
  aes256_ctr_encrypt( &ctxctr, ctr_buf, 16 );
  memcpy( decoded_message, data + nonce_len, *message_len );
  aes256_ctr_encrypt( &ctxctr, decoded_message, *message_len );

  aes256_ctr_done( &ctxctr );

  // Generate CBC-MAC Tag
  // Flags for B0
  tag_buf[0] = 0;
  tag_buf[0] |= ( ( ( tag_len - 2 ) / 2 ) & 7 ) << 3;
  tag_buf[0] |= ( ( 15 - nonce_len ) - 1 ) & 7;

  // Fill in nonce
  memcpy(&tag_buf[1], data, nonce_len );

  // Encode l(m) in MSB order
  for( i = 15; i > nonce_len; i-- )
  {
    if( ( 15 - i ) >= ( sizeof( *message_len ) ) )
      tag_buf[ i ] = 0;
    else
      tag_buf[ i ] =  ( ( *message_len >> ( 8 * ( 15 - i ) ) ) & 0xff );
  }

  aes256_cbc_init(&ctxcbc, key, NULL );
  aes256_cbc_encrypt(&ctxcbc, ( void * )tag_buf, 16);

  for( i = 0; i < ( ( *message_len + (16 - 1) ) / 16 ); i++)
  {
    cur_len = ( ( *message_len - i * 16 ) >= 16 ) ? 16 : ( *message_len - i * 16 );
    memset((void *)tag_buf, 0, 16);
    memcpy((void *)tag_buf, ( (uint8_t *)decoded_message ) + i * 16, cur_len );

    aes256_cbc_encrypt(&ctxcbc, ( void * )tag_buf, 16);
  }

  aes256_cbc_done(&ctxcbc);

  if( memcmp( tag_buf, ctr_buf, tag_len ) == 0 )
  {
    return decoded_message;
  }
  else
  {
    free( decoded_message );
    return NULL;
  }
}

void aes256_cbc_init(aes256_cbc_context *ctx, const void *key, const void *iv )
{
  if( ctx != NULL )
  {
    if( iv != NULL )
    {
      memcpy((void *)ctx->iv.hwctr,iv,16);
    }
    else
    {
      memset((void *)ctx->iv.hwctr, 0, 16 );
    }

    if( key != NULL )
    {
      memcpy((void *)ctx->key.hwkey,key,32);
    }
  }

  gAES0_enter_cbc_config();

  // 256-bit Key Encryption
  SI32_AES_A_select_key_size_256(SI32_AES_0);
}

void aes256_ecb_init(aes256_ecb_context *ctx, const void *key, const void *iv )
{
  if( ctx != NULL )
  {
    if( iv != NULL )
    {
      memcpy((void *)ctx->iv.hwctr,iv,16);
    }
    else
    {
      memset((void *)ctx->iv.hwctr, 0, 16 );
    }

    if( key != NULL )
    {
      memcpy((void *)ctx->key.hwkey,key,32);
    }
  }

  gAES0_enter_ecb_config();

  // 128-bit Key Encryption
  SI32_AES_A_select_key_size_256(SI32_AES_0);
}

void aes256_cbc_encrypt( aes256_cbc_context *ctx, void *data, size_t len )
{
  uint32_t block_number,i;
  
  SI32_AES_A_select_xor_path_input(SI32_AES_0);
  SI32_AES_A_select_encryption_mode(SI32_AES_0);
  SI32_AES_A_enable_key_capture(SI32_AES_0);   

  if( len % 16 != 0 )
  {
    printf("ERROR: data buffer must be a multiple of block size (16)");
    return;
  }
  
  // CBC encrypt data
  for( block_number = 0; block_number < ( ( len + (16 - 1) ) / 16 ); block_number++)
  {
    SI32_AES_A_write_hardware_counter(SI32_AES_0, ctx->iv );
    SI32_AES_A_write_hardware_key(SI32_AES_0, ctx->key );

    // Write plaintext
    for (i = 0; i < 4; i++)
    {
       SI32_AES_A_write_datafifo (SI32_AES_0, ( ( uint32_t * )data )[ block_number * 4 + i ]);
    }

    // Encrypt
    SI32_AES_A_clear_operation_complete_interrupt (SI32_AES_0);
    SI32_AES_A_start_operation (SI32_AES_0);
    while (!SI32_AES_A_is_operation_complete_interrupt_pending (SI32_AES_0));

    // Read ciphertext
    for (i = 0; i < 4; i++)
    {
       ( ( uint32_t * )data )[ block_number * 4 + i ] = SI32_AES_A_read_datafifo (SI32_AES_0);
       ctx->iv.hwctr[ i ] = ( ( uint32_t * )data )[ block_number * 4 + i ];
    }
  }
}


void aes256_ecb_encrypt( aes256_ecb_context *ctx, void *data, size_t len )
{
  uint32_t block_number,i;
  
  SI32_AES_A_exit_cipher_block_chaining_mode(SI32_AES_0);
  SI32_AES_A_exit_counter_mode(SI32_AES_0);
  SI32_AES_A_select_xor_path_none(SI32_AES_0);
  SI32_AES_A_select_encryption_mode(SI32_AES_0);
  SI32_AES_A_enable_key_capture(SI32_AES_0);   

  if( len % 16 != 0 )
  {
    printf("ERR: len must be multiple 16\n");
    return;
  }
  
  // ECB encrypt data
  for( block_number = 0; block_number < ( ( len + (16 - 1) ) / 16 ); block_number++)
  {
    SI32_AES_A_write_hardware_counter(SI32_AES_0, ctx->iv );
    SI32_AES_A_write_hardware_key(SI32_AES_0, ctx->key );

    // Write plaintext
    for (i = 0; i < 4; i++)
    {
       SI32_AES_A_write_datafifo (SI32_AES_0, ( ( uint32_t * )data )[ block_number * 4 + i ]);
    }

    // Encrypt
    SI32_AES_A_clear_operation_complete_interrupt (SI32_AES_0);
    SI32_AES_A_start_operation (SI32_AES_0);
    while (!SI32_AES_A_is_operation_complete_interrupt_pending (SI32_AES_0));

    // Read ciphertext
    for (i = 0; i < 4; i++)
    {
       ( ( uint32_t * )data )[ block_number * 4 + i ] = SI32_AES_A_read_datafifo (SI32_AES_0);
       ctx->iv.hwctr[ i ] = ( ( uint32_t * )data )[ block_number * 4 + i ];
    }
  }
}

// void aes256_cbc_decrypt( aes256_cbc_context *ctx, void *data, size_t len )
// {
//   uint32_t block_number,i;

//   SI32_AES_A_select_decryption_mode(SI32_AES_0);
//   SI32_AES_A_select_xor_path_output(SI32_AES_0);
//   SI32_AES_A_disable_key_capture (SI32_AES_0);

//   SI32_AES_A_write_hardware_key(SI32_AES_0, ctx->key );

//   if( len % 16 != 0 )
//   {
//     printf("ERROR: data buffer must be a multiple of block size (16)");
//     return;
//   }
  
//   // CBC encrypt data
//   for( block_number = 0; block_number < ( ( len + (16 - 1) ) / 16 ); block_number++)
//   {
//     SI32_AES_A_write_hardware_counter(SI32_AES_0, ctx->iv );
//     // Write plaintext
//     for (i = 0; i < 4; i++)
//     {
//       SI32_AES_A_write_datafifo (SI32_AES_0, ( ( uint32_t * )data )[ block_number * 4 + i ]);
//       ctx->iv.hwctr[ i ] = ( ( uint32_t * )data )[ block_number * 4 + i ];
//     }

//     // Encrypt
//     SI32_AES_A_clear_operation_complete_interrupt (SI32_AES_0);
//     SI32_AES_A_start_operation (SI32_AES_0);
//     while (!SI32_AES_A_is_operation_complete_interrupt_pending (SI32_AES_0));

//     // Read ciphertext
//     for (i = 0; i < 4; i++)
//     {
//        ( ( uint32_t * )data )[ block_number * 4 + i ] = SI32_AES_A_read_datafifo (SI32_AES_0);
//     }
//   }
// }

void aes256_cbc_done( aes256_cbc_context *ctx )
{
    // Clear out state
    memset((void *)ctx->iv.hwctr,0,16);
    memset((void *)ctx->key.hwkey,0,32);
}

void aes256_ecb_done( aes256_ecb_context *ctx )
{
    // Clear out state
    memset((void *)ctx->iv.hwctr,0,16);
    memset((void *)ctx->key.hwkey,0,32);
}

void aes256_ctr_init(aes256_ctr_context *ctx, const void *key, const void *iv )
{
  if( ctx != NULL )
  {
    if( iv != NULL )
    {
      memcpy((void *)ctx->iv.hwctr,iv,16);
    }
    else
    {
      memset((void *)ctx->iv.hwctr, 0, 16 );
    }

    if( key != NULL )
    {
      memcpy((void *)ctx->key.hwkey,key,32);
    }
  }

  gAES0_enter_ctr_config();

  // 256-bit Key Encryption
  SI32_AES_A_select_encryption_mode(SI32_AES_0);
  SI32_AES_A_select_key_size_256(SI32_AES_0);

  SI32_AES_A_write_hardware_key(SI32_AES_0, ctx->key);

  // Set the initial nonce value
  SI32_AES_A_write_hardware_counter(SI32_AES_0, ctx->iv);
}

void aes256_ctr_encrypt( aes256_ctr_context *ctx, void *data, size_t len )
{
  uint32_t block_number,i;
  uint32_t tmp_data[4];
  uint8_t cur_len;

  // CTR encrypt data
  for( block_number = 0; block_number < ( ( len + (16 - 1) ) / 16 ); block_number++)
  {
    // Copy data into temporary vector
    cur_len = ( (len - block_number * 16 ) >= 16 ) ? 16 : (len - block_number * 16 );
    memset((void *)tmp_data, 0, 16);
    memcpy((void *)tmp_data, ( (uint8_t *)data ) + block_number * 16, cur_len );

    // Write plaintext to xor fifo
    for (i = 0; i < 4; i++)
    {
       SI32_AES_A_write_xorfifo (SI32_AES_0, tmp_data[ i ]);
    }

    // Encrypt
    SI32_AES_A_clear_operation_complete_interrupt (SI32_AES_0);
    SI32_AES_A_start_operation (SI32_AES_0);
    while (!SI32_AES_A_is_operation_complete_interrupt_pending (SI32_AES_0));

    // Read ciphertext
    for (i = 0; i < 4; i++)
    {
       tmp_data[ i ] = SI32_AES_A_read_datafifo (SI32_AES_0);
    }

    // Copy data back to our work buffer
    for( i = 0; i < cur_len; i++)
    {
      ( ( uint8_t * )data)[ block_number * 16 + i ] = ( ( uint8_t * )tmp_data )[ i ];
    }
  }
}

void aes256_ctr_done( aes256_ctr_context *ctx )
{
    // Clear out state
    memset((void *)ctx->iv.hwctr,0,16);
    memset((void *)ctx->key.hwkey,0,32);
}

//Lua: param( param name, set value )
static int aes_cbc_encrypt( lua_State *L )
{

  aes256_cbc_context ctx;
  luaL_Buffer b;
  size_t len;
  const uint8_t *data;
  uint8_t *data_enc;
  unsigned char *iv;
  uint8_t key[32];

  luaL_buffinit(L,&b);

  romfs_sec_unlock();
  len = param_get( "~key", "", PARAM_SECURE, (uint8_t *)key, 32);
  romfs_sec_lock();
  if(len != 32)
  {
    logconsole(LOG_LEVEL_BASIC, LOG_MODULE_AES, "T#invalidKey=%d", len);
    return 0;
  }

    // Get IV if passed, otherwise zero
  if( lua_gettop(L) == 2 )
  {
    iv = (unsigned char *) luaL_checklstring(L, 2, &len);
    if(len != 16)
    {
      logconsole(LOG_LEVEL_BASIC, LOG_MODULE_AES, "T#invalidIV=%d", len);
      return 0;
    }
    
    aes256_cbc_init( &ctx, (void *)key, (void *)iv );
  }
  else
  {
    aes256_cbc_init( &ctx, (void *)key, NULL );
  }

  data = (uint8_t *) luaL_checklstring ( L, 1, &len);
  data_enc = malloc(len);
  memcpy(data_enc, data, len);

  aes256_cbc_encrypt(&ctx, data_enc, len);

  luaL_addlstring(&b,( const char * )data_enc,len);
  luaL_pushresult( &b );

  free(data_enc);

  aes256_cbc_done(&ctx);

  return 1;
}

int aes_ecb_encrypt( unsigned char * data, int data_len, unsigned char * iv )
{
  aes256_ecb_context ctx;
  size_t len;
  unsigned char key[32];
  unsigned char * keyptr = key;
  
  romfs_sec_unlock();
  len = param_get( "~key", "", PARAM_SECURE, keyptr, 32);
  romfs_sec_lock();
  if(len != 32)
  {
    logconsole(LOG_LEVEL_BASIC, LOG_MODULE_AES, "T#invalidKey=%d", len);
    return 0;
  }

  // Provide initilization vector, or NULL none provided
  aes256_ecb_init(&ctx, keyptr, iv );
  aes256_ecb_encrypt(&ctx, data, data_len);

  aes256_ecb_done(&ctx);
  return 1;
}


//Lua: param( param name, set value )
static int lua_aes_ecb_encrypt( lua_State *L )
{
  luaL_Buffer b;
  size_t len;
  const uint8_t *data;
  uint8_t *data_enc;
  unsigned char *iv = NULL;

  luaL_buffinit(L,&b);

    // Get IV if passed, otherwise zero
  if( lua_gettop(L) == 2 )
  {
    iv = (unsigned char *) luaL_checklstring(L, 2, &len);
    if(len != 16)
    {
      logconsole(LOG_LEVEL_BASIC, LOG_MODULE_AES, "T#invalidIV=%d", len);
      return 0;
    }
  }

  data = (uint8_t *) luaL_checklstring ( L, 1, &len);
  data_enc = malloc(len);
  memcpy(data_enc, data, len);

  aes_ecb_encrypt( data_enc, len, iv );

//aes.sethexkey("3132333435363738393031323334353631323334353637383930313233343536") 12345678901234561234567890123456
//settings.flash(settings.report_format, 3)
//aes.encryptecb("0987654321098765") //30393837363534333231303938373635
//aes.encryptecb("09876543210987651234567890123456") //3039383736353433323130393837363531323334353637383930313233343536

  luaL_addlstring(&b,( const char * )data_enc,len);
  luaL_pushresult( &b );

  free(data_enc);

  return 1;
}

//Lua: param( param name, set value )
int aes_cbc_decrypt( lua_State *L )
{
  aes256_cbc_context ctx;
  luaL_Buffer b;
  size_t len;
  const uint8_t *data;
  uint8_t *data_enc;
  unsigned char *iv;
  uint8_t key[32];

  luaL_buffinit(L,&b);

  romfs_sec_unlock();
  len = param_get( "~key", "", PARAM_SECURE, (uint8_t *)key, 32);
  romfs_sec_lock();
  if(len != 32)
  {
    logconsole(LOG_LEVEL_BASIC, LOG_MODULE_AES, "T#invalidKey=%d", len);
    return 0;
  }

    // Get IV if passed, otherwise zero
  if( lua_gettop(L) == 2 )
  {
    iv = (unsigned char *) luaL_checklstring(L, 2, &len);
    if(len != 16)
    {
      logconsole(LOG_LEVEL_BASIC, LOG_MODULE_AES, "T#invalidIV=%d", len);
      return 0;
    }
    
    aes256_cbc_init( &ctx, (void *)key, (void *)iv );
  }
  else
  {
    aes256_cbc_init( &ctx, (void *)key, NULL );
  }

  data = (uint8_t *) luaL_checklstring ( L, 1, &len);
  data_enc = malloc(len);
  memcpy(data_enc, data, len);

  aes256_cbc_decrypt(&ctx, data_enc, len);

  luaL_addlstring(&b,( const char * )data_enc,len);
  luaL_pushresult( &b );

  free(data_enc);

  aes256_cbc_done(&ctx);

  return 1;
}

//Lua: param( param name, set value )
static int aes_ctr_encrypt( lua_State *L )
{

  aes256_ctr_context ctx;
  luaL_Buffer b;
  size_t len;
  const uint8_t *data;
  uint8_t *data_enc;
  unsigned char *iv;
  uint8_t key[32];

  luaL_buffinit(L,&b);

  romfs_sec_unlock();
  len = param_get( "~key", "", PARAM_SECURE, key, 32);
  romfs_sec_lock();

  if(len != 32)
  {
    logconsole(LOG_LEVEL_BASIC, LOG_MODULE_AES, "T#invalidKey=%d", len);
    return 0;
  }

  // Get IV if passed, otherwise zero
  if( lua_gettop(L) == 2 )
  {
    iv = (unsigned char *) luaL_checklstring(L, 2, &len);
    if(len != 16)
    {
      logconsole(LOG_LEVEL_BASIC, LOG_MODULE_AES, "T#invalidIV=%d", len);
      return 0;
    }
    aes256_ctr_init( &ctx, (void *)key, (void *)iv );
  }
  else
  {
    aes256_ctr_init( &ctx, (void *)key, NULL );
  }

  data = (uint8_t *) luaL_checklstring ( L, 1, &len);
  data_enc = malloc(len);
  memcpy(data_enc, data, len);

  aes256_ctr_encrypt(&ctx, data_enc, len);

  luaL_addlstring(&b,( const char * )data_enc,len);
  luaL_pushresult( &b );

  free(data_enc);

  aes256_ctr_done(&ctx);

  return 1;
}




static int aes_sethexkey( lua_State *L )
{
  size_t l;
  int i;
  uint8_t key_buf[ 32 ];
  unsigned char *s = (unsigned char *) luaL_checklstring(L, 1, &l);

  if(l != 64 && l != 32 && l != 0)
    return luaL_error( L, "Invalid key len %d\n", l );


  for(i = 0; i<l; i++)
  {
    // Reset 
    if( i % 2 == 0 )
      key_buf[ i / 2 ] = 0;
    else
      key_buf[ i / 2 ] <<= 4;

    if( s[ i ] >= '0' && s[ i ] <= '9' )
      key_buf[ i / 2 ] += s[ i ] - '0';
    else if ( s[ i ] >= 'A' && s[ i ] <= 'Z')
      key_buf[ i / 2 ] += s[ i ] - 'A' + 10;
    else if ( s[ i ] >= 'a' && s[ i ] <= 'z' )
      key_buf[ i / 2 ] += s[ i ] - 'a' + 10;
    else
    {
      return luaL_error( L, "Invalid char in string");
    }
  }

  if( l > 0 && logconsole_nobreak(LOG_LEVEL_BASIC, LOG_MODULE_MAIN, "SHK#key=0x") )
  {
    for( i = 0; i<(l/2); i++)
      logconsole_core(LOG_LEVEL_BASIC, LOG_MODULE_MAIN, "%02X", key_buf[ i ] );

    logconsole_footer(LOG_LEVEL_BASIC);
  }

  romfs_sec_unlock();
  param_set( "~key", "", PARAM_SECURE, key_buf, ( l / 2 ) );
  romfs_sec_lock();
  return 0;
}

int aes_iskeyset( void )
{
  // Check if key is set
  int ret = 0;

  romfs_sec_unlock();

  ret = param_get_string_len("~key", "");
    
  romfs_sec_lock();

  return ret;
}

static int lua_aes_iskeyset( lua_State *L )
{
  lua_pushinteger( L, aes_iskeyset() );
  return 1;
}

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"

// Module function map
const LUA_REG_TYPE aes_map[] =
{
  { LSTRKEY( "encryptecb" ),  LFUNCVAL( lua_aes_ecb_encrypt ) },
//  { LSTRKEY( "encryptcbc" ),  LFUNCVAL( aes_cbc_encrypt ) },
//  { LSTRKEY( "encryptctr" ),  LFUNCVAL( aes_ctr_encrypt ) },
//  { LSTRKEY( "decryptctr" ),  LFUNCVAL( aes_ctr_encrypt ) },
  { LSTRKEY( "encryptccm" ),  LFUNCVAL( aes_ccm_encode_message ) },
  { LSTRKEY( "decryptccm" ),  LFUNCVAL( aes_ccm_decode_message ) },
  { LSTRKEY( "sethexkey" ),  LFUNCVAL( aes_sethexkey ) },
  { LSTRKEY( "iskeyset" ),  LFUNCVAL( lua_aes_iskeyset ) },
  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_aes( lua_State *L )
{
  LREGISTER( L, AUXLIB_AES, aes_map );
}



