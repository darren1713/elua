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
#include "param.h"

#include "SI32_AES_A_Support.h"
#include "SI32_AES_A_Type.h"

#include "auxmods.h"
#include "stacks.h"
#include "type.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    SI32_AES_A_Hardware_Key_Type key;
    SI32_AES_A_Hardware_Counter_Type iv;
} aes256_cbc_context;

typedef struct
{
    SI32_AES_A_Hardware_Key_Type key;
    SI32_AES_A_Hardware_Counter_Type iv;
} aes256_ctr_context;

typedef struct
{
    SI32_AES_A_Hardware_Key_Type key;
    SI32_AES_A_Hardware_Counter_Type iv;
} aes256_ecb_context;

void aes256_cbc_init(aes256_cbc_context *ctx, const void *key, const void *iv );
void aes256_cbc_encrypt( aes256_cbc_context *ctx, void *data, size_t len );
void aes256_cbc_decrypt( aes256_cbc_context *ctx, void *data, size_t len );
void aes256_cbc_done( aes256_cbc_context *ctx );

void aes256_ctr_init(aes256_ctr_context *ctx, const void *key, const void *iv );
void aes256_ctr_encrypt( aes256_ctr_context *ctx, void *data, size_t len );
void aes256_ctr_decrypt( aes256_ctr_context *ctx, void *data, size_t len );
void aes256_ctr_done( aes256_ctr_context *ctx );

void aes256_ecb_init(aes256_ecb_context *ctx, const void *key, const void *iv );
void aes256_ecb_encrypt( aes256_ecb_context *ctx, void *data, size_t len );
void aes256_ecb_decrypt( aes256_ecb_context *ctx, void *data, size_t len );
void aes256_ecb_done( aes256_ecb_context *ctx );

int aes_iskeyset( void );
int aes_ecb_encrypt( unsigned char * data, int data_len, unsigned char * iv );

unsigned char * aes256_ccm_encrypt(const unsigned char *data, size_t data_len, const unsigned char *key, size_t key_len, const unsigned char *nonce, size_t nonce_len, size_t tag_len );
unsigned char * aes256_ccm_decrypt(const unsigned char *data, size_t data_len, const unsigned char *key, size_t key_len, size_t nonce_len, size_t tag_len, size_t *message_len );

//*****************************************************************************
//
// Helper Macros for Ethernet Processing
//
//*****************************************************************************
//
// htonl/ntohl - big endian/little endian byte swapping macros for
// 32-bit (long) values
//
//*****************************************************************************
#ifndef htonl
    #define htonl(a)                    \
        ((((a) >> 24) & 0x000000ff) |   \
         (((a) >>  8) & 0x0000ff00) |   \
         (((a) <<  8) & 0x00ff0000) |   \
         (((a) << 24) & 0xff000000))
#endif

#ifndef ntohl
    #define ntohl(a)    htonl((a))
#endif

//*****************************************************************************
//
// htons/ntohs - big endian/little endian byte swapping macros for
// 16-bit (short) values
//
//*****************************************************************************
#ifndef htons
    #define htons(a)                \
        ((((a) >> 8) & 0x00ff) |    \
         (((a) << 8) & 0xff00))
#endif

#ifndef ntohs
    #define ntohs(a)    htons((a))
#endif



