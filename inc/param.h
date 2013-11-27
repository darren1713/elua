#ifndef __PARAM_H__
#define __PARAM_H__

#include <stdint.h>

#define SETTING_FORMAT "/wo/._%s%s"

enum {
  PARAM_NIL=0,
  PARAM_INTEGER,
  PARAM_BOOLEAN,
  PARAM_STRING,
  PARAM_SECURE,
};

// Generic set & get functions
int32_t set_param( const char * name, const char * prefix, uint8_t type, uint8_t * value, uint32_t length ); // returns bytes written for type (not including header)
int32_t get_param( const char * name, const char * prefix, uint8_t type, uint8_t * value, uint32_t length ); // returns bytes read, <0 = error
int32_t get_param_type( const char * name, const char * prefix); // returns type, <0 = error

// Set & get functions for signed 32-bit integer
int32_t set_param_s32( const char * name, const char * prefix, int32_t value ); // returns bytes written for type (not including header)
int32_t get_param_s32( const char * name, const char * prefix, int32_t *value ); // returns bytes read, <0 = error

// Set & get functions for strings
int32_t set_param_string( const char * name, const char * prefix, uint8_t * value ); // returns bytes written for type (not including header)
int32_t get_param_string_len( const char * name, const char * prefix ); // returns byte length, <0 = error
int32_t get_param_string( const char * name, const char * prefix, uint8_t *value, uint32_t max_len ); // returns bytes read, <0 = error

#endif