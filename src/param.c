#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdint.h>
#include "devman.h"
#include "param.h"


FILE * open_param_file(const char * name, const char * prefix, const char * format, const char * mode)
{
    char fname[ DM_MAX_FNAME_LENGTH + 1 ];
    snprintf( fname, DM_MAX_FNAME_LENGTH + 1, format, prefix, name );
    FILE * ret = fopen(fname, mode);
    return ret;
}
int32_t set_param( const char * name, const char * prefix, uint8_t type, uint8_t * value, uint32_t length )
{
    FILE *fp;
    int n;

    fp = open_param_file(name, prefix, SETTING_FORMAT, "w+");

    // bail if we can't write out setting
    if (fp == NULL)
        return -1;

    n = fputc(type, fp);
    n = fwrite( value, 1, length, fp);
    fclose(fp);
    return n;
}

int32_t get_param( const char * name, const char * prefix, uint8_t type, uint8_t * value, uint32_t length )
{
    FILE *fp;
    int n;

    fp = open_param_file(name, prefix, SETTING_FORMAT, "r");

    // bail if we can't get stored setting
    if (fp == NULL)
    {
        return -1;
    }

    // bail if type isn't what we expected
    if( fgetc( fp ) != type)
    {
        printf("bad type\n");
        fclose(fp);
        return -2;
    }

    n = fread(value, 1, length, fp);
    fclose(fp);
    return n; // return length of item actually read
}

int32_t get_param_type( const char * name, const char * prefix)
{
    FILE *fp;
    uint8_t type;

    fp = open_param_file(name, prefix, SETTING_FORMAT, "r");

    if (fp == NULL)
        return -1;

    type = fgetc( fp );
    fclose( fp );
    return ( int32_t )type;
}

// store 32-bit integer parameter
int32_t set_param_s32( const char * name, const char * prefix, int32_t value )
{
    return set_param( name, prefix, PARAM_INTEGER, ( uint8_t * )&value, 4 );
}

// get 32-bit integer parameter, return number of bytes read
int32_t get_param_s32( const char * name, const char * prefix, int32_t *value )
{
    uint8_t b[4];
    int32_t ret;

    ret = get_param( name, prefix, PARAM_INTEGER, b, 4);

    if( ret < 0 )
        return ret;

    *value = *( int32_t * )b;
    return ret;
}

// store string parameter
int32_t set_param_string( const char * name, const char * prefix, uint8_t * value )
{
    return set_param( name, prefix, PARAM_STRING, value, strlen( ( const char * )value ) );
}

// return string parameter length
int32_t get_param_string_len( const char * name, const char * prefix )
{
    FILE *fp;
    int n = 0;

    fp = open_param_file(name, prefix, SETTING_FORMAT, "r");

    if( fp == NULL )
        return -1;

    while( fgetc( fp ) != EOF )
        n++;

    fclose(fp);
    return n-1; // return position minus header length
}


// get string parameter, return length
int32_t get_param_string( const char * name, const char * prefix, uint8_t *value, uint32_t max_len  )
{
    int32_t len;

    len = get_param_string_len( name, prefix );

    // If an error was encountered, pass it up
    if( len < 0 )
        return len;

    if( len <= max_len )
        len = get_param( name, prefix, PARAM_STRING, value, len);
    else
        return -3;

    // Null terminate string
    value[ len ] = 0;

    return len;
}
