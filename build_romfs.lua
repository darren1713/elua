#! /usr/bin/env lua

--[[
   build_elua.lua: A build script for eLua written in Lua.

   The command line syntax is the same as for the old scons/SConstruct system.
   See http://www.eluaproject.net/en_building.html

   The only required option is the target board or CPU. e.g.:
     lua build_elua.lua board=MIZAR32

   This script requires some well-known Lua libraries to run.
   To install them on Ubuntu/Debian, go (as root):
       apt-get install luarocks
       luarocks install luafilesystem
       luarocks install lpack
       luarocks install md5
--]]

local args = { ... }
local b = require "utils.build"
local mkfs = require "utils.mkfs"

builder = b.new_builder()
utils = b.utils
sf = string.format

-------------------------------------------------------------------------------

builder:add_option( 'target', 'build "regular" float lua, 32 bit integer-only "lualong" or 64-bit integer only lua "lualonglong"', 'lua', { 'lua', 'lualong', 'lualonglong' } )
builder:add_option( 'romfs', 'ROMFS compilation mode', 'verbatim', { 'verbatim' , 'compress', 'compile', 'compile_raw' } )
builder:add_option( 'crosscompile_integer', 'Target compiler integer mode', 'int 32', { 'int 32' } )
builder:add_option( 'crosscompile_endian', 'Endianness', 'little', { 'little' , 'big' } )
builder:add_option( 'romfs_dir', 'choose ROMFS directory', 'romfs' )
builder:init( args )
builder:set_build_mode( builder.BUILD_DIR_LINEARIZED )

-- Build the 'comp' target which will 'redirect' all the requests
-- for its fields to builder:get_option
comp = {}
setmetatable( comp, { __index = function( t, key ) return builder:get_option( key ) end } )

local function dprint( ... )
  if comp.disp_mode ~= "minimal" then
    print( ... )
  end
end

-- Build the compilation command now
local fscompcmd = ''
if comp.romfs == 'compile' or comp.romfs == 'compile_raw' then
  if comp.target == 'lualonglong' then
    print "Cross-compilation is not yet supported for 64-bit integer-only Lua (lualonglong)."
    os.exit( -1 )
  end
  local suffix = ''
  if utils.is_windows() then
    suffix = '.exe'
  end
  -- First check for luac.cross in the current directory
  if not utils.is_file( "luac.cross" .. suffix ) then
    print "The eLua cross compiler was not found."
    print "Build it by running 'lua cross-lua.lua'"
    os.exit( -1 )
  end
  local cmdpath = { lfs.currentdir(), sf( 'luac.cross%s -ccn %s -cce %s -o %%s -s %%s', suffix, comp.crosscompile_integer:lower(), comp.crosscompile_endian:lower() ) }
  fscompcmd = table.concat( cmdpath, utils.dir_sep )
elseif comp.romfs == 'compress' then
  if comp.target == 'lualong' or comp.target == 'lualonglong' then fscompoptnums = '--noopt-numbers' else fscompoptnums = '--opt-numbers' end
  fscompcmd = 'lua luasrcdiet.lua --quiet --maximum --opt-comments --opt-whitespace --opt-emptylines --opt-eols --opt-strings ' .. fscompoptnums .. ' --opt-locals -o %s %s'
end

-- User report
print ""
print "*********************************"
print "Compiling eLua ..."
print( "ROMFS mode:     ", comp.romfs )
print "*********************************"
print ""


-------------------------------------------------------------------------------
-- Create compiler/linker/assembler command lines and build

-- ROM file system builder

romfs_exclude_patterns = { '%.DS_Store', '%.gitignore' }

function match_pattern_list( item, list )
  for k, v in pairs( list ) do
    if item:find(v) then return true end
  end
end

local function make_romfs( target, deps )
  print "Building ROM file system ..."
  local romdir = builder:get_option( "romfs_dir" )
  local flist = {}
  flist = utils.string_to_table( utils.get_files( romdir, function( fname ) return not match_pattern_list( fname, romfs_exclude_patterns ) end ) )
  flist = utils.linearize_array( flist )
  for k, v in pairs( flist ) do
    flist[ k ] = v:gsub( romdir .. utils.dir_sep, "" )
  end

  print( 'Executing ' .. fscompcmd)
  if not mkfs.mkfs( romdir, "romfiles", flist, comp.romfs, fscompcmd ) then return -1 end
  if utils.is_file( "inc/romfiles.h" ) then
    -- Read both the old and the new file
    local oldfile = io.open( "inc/romfiles.h", "rb" )
    assert( oldfile )
    local newfile = io.open( "romfiles.h", "rb" )
    assert( newfile )
    local olddata, newdata = oldfile:read( "*a" ), newfile:read( "*a" )
    oldfile:close()
    newfile:close()
    -- If content is similar return '1' to builder to indicate that the target didn't really
    -- produce a change even though it ran
    if olddata == newdata then
      os.remove( "romfiles.h" )
      return 1
    end
    os.remove( "inc/romfiles.h" )
  end
  os.rename( "romfiles.h", "inc/romfiles.h" )
  return 0
end

make_romfs(nil,nil)