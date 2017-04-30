-- A module to convert an entire directory to a C array, in the "romfs" format

module( ..., package.seeall )
require "pack"
local sf = string.format
local b = require "utils.build"
local utils = b.utils

local _crtline = '  '
local _numdata = 0
local _bytecnt = 0
local maxlen = 30
local _fcnt = 0
local alignment = 4
local outfile

-- Line output function
local function _add_data( data, outfile, moredata )
  if moredata == nil then moredata = true end
  _bytecnt = _bytecnt + 1
  _fcnt = _fcnt + 1
  if moredata then
    _crtline = _crtline .. sf( "0x%02X, ", data )
  else
    _crtline = _crtline .. sf( "0x%02X", data )
  end
  _numdata = _numdata + 1
  if _numdata == 16 or not moredata then
    outfile:write( _crtline .. '\n' )
    _crtline = '  '
    _numdata = 0
  end
end

-- dirname - the directory where the files are located.
-- outname - the name of the C output
-- flist - list of files
-- mode - preprocess the file system:
--   "verbatim" - copy the files directly to the FS as they are
--   "compile" - precompile all files to Lua bytecode and then copy them
--   "compress" - keep the source code, but compress it with LuaSrcDiet
-- compcmd - the command to use for compiling if "mode" is "compile"
-- Returns true for OK, false for error
function mkfs( dirname, outname, flist, mode, compcmd )
  -- Try to create the output files
  local outfname = outname .. ".h"
  outfile = io.open( outfname, "wb" )
  if not outfile then
    print "Unable to create output file"
    return false
  end

  print( sf( "Generating file %s/%s", dirname, outfname ) )

  _crtline = '  '
  _numdata = 0
  _bytecnt = 0

  -- Generate headers
  outfile:write( "// Generated by mkfs.lua\n// DO NOT MODIFY\n\n" )
  outfile:write( sf( "#ifndef __%s_H__\n#define __%s_H__\n\n", outname:upper(), outname:upper() ) )
  
  outfile:write( sf( "const unsigned char %s_fs[] = \n{\n", outname:lower() ) )
  
  -- Process all files
  for _, fname in pairs( flist ) do
    if #fname > maxlen then
      print( sf( "Skipping %s (name longer than %d chars)", fname, maxlen ) )
    else 
      -- Get actual file name
      local realname = dirname .. utils.dir_sep .. fname      
      -- Ensure it actually is a file
      if not utils.is_file( realname ) then
        print( sf( "Skipping %s ... (not found or not a regular file)", fname ) )
      else          
        -- Try to open and read the file
        local crtfile = io.open( realname, "rb" )
        if not crtfile then
          outfile:close()
          os.remove( outfname )
          print( sf( "Unable to read %s", fname ) )
          return false
        end
        -- Do we need to process the file?
        local fextpart, fnamepart = ''
        if mode == "compile" or mode == "compress" then
          fnamepart, fextpart = utils.split_ext( realname )
          local newext = mode == "compress" and ".lua.tmp" or ".lc"
          if fextpart == ".lua" then
            newname = fnamepart .. newext
            if mode == "compress" then
              print( sf( "Compressing %s to %s ...", realname, newname ) )
            else
              print( sf( "Cross compiling %s to %s ...", realname, newname ) )
            end
            if os.execute( sf( compcmd, newname, realname ) ) ~= 0 then
              print "Cross-compilation error, aborting"
              outfile:close()
              crtfile:close()
              return false
            end
            crtfile:close()
            crtfile = io.open( newname, "rb" )
            if not crtfile then
              outfile:close()
              os.remove( outfname )
              print( sf( "Unable to read %s", newname ) )
              return false
            end
            if mode == "compile" then
              fnamepart, fextpart = utils.split_ext( fname )
              fname = fnamepart .. ".lc"
            end
          end
        end
        local filedata = crtfile:read( '*a' )
        crtfile:close()
        if fextpart == ".lua" and mode ~= "verbatim" then
          os.remove( newname )
        end
        -- Write name, size, id, numpars
        _fcnt = 0
        for i = 1, #fname do
          _add_data( fname:byte( i ), outfile )
        end
        _add_data( 0, outfile ) -- ASCIIZ
        local plen = string.pack( "<i", #filedata )
         -- Round to a multiple of 'alignment'
        while _bytecnt % alignment ~= 0 do
          _add_data( 0, outfile )
        end
        -- Write size
        _add_data( plen:byte( 1 ), outfile )
        _add_data( plen:byte( 2 ), outfile )
        _add_data( plen:byte( 3 ), outfile )
        _add_data( plen:byte( 4 ), outfile )
       -- Then write the rest of the file
        for i = 1, #filedata do
          _add_data( filedata:byte( i ), outfile )
        end
        -- Report
        print( sf( "Encoded file %s (%d bytes real size, %d bytes encoded size)", fname, #filedata, _fcnt ) )
      end
    end
  end
    
  -- All done, write the final "0xFF" (terminator)
  _add_data( 0xFF, outfile, false )
  outfile:write( "};\n\n#endif\n" );
  outfile:close()
  print( sf( "Done, total size is %d bytes", _bytecnt ) )
  return true
end

