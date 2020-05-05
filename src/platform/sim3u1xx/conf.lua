-- Configuration file for the sim3u1xx backend

if utils.is_file(sf( "src/platform/%s/FreakUSB/class/CDC/cdc.c", platform )) ~= true then
  print "ERROR: Missing FreakUSB submodule. Run the following commands from the eLua root:"
  print "git submodule init"
  print "git submodule update"
  os.exit( -1 )
end

addi( sf( 'src/platform/%s/si32Hal/SI32_Modules', platform ) )
addi( sf( 'src/platform/%s/si32Hal/sim3u1xx', platform ) )
addi( sf( 'src/platform/%s/FreakUSB/class/CDC', platform ) )
addi( sf( 'src/platform/%s/FreakUSB/usb', platform ) )
addi( sf( 'src/platform/%s/FreakUSB/hw/sim3u1xx', platform ) )
addi( sf( 'src/platform/%s/generated', platform ) )

local fwlib_files = utils.get_files( sf( "src/platform/%s/FreakUSB/usb", platform ), ".*%.c$" )
fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/si32Hal/SI32_Modules", platform ), ".*RTC.*%.c$" )
fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/si32Hal/SI32_Modules", platform ), ".*VREG.*%.c$" )
fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/si32Hal/SI32_Modules", platform ), ".*VMON.*%.c$" )
fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/si32Hal/SI32_Modules", platform ), ".*AES.*%.c$" )
fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/si32Hal/SI32_Modules", platform ), ".*SARADC.*%.c$" )
fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/si32Hal/SI32_Modules", platform ), ".*DMA.*%.c$" )
fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/si32Hal/sim3u1xx", platform ), ".*%.c$" )
fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/FreakUSB/hw/sim3u1xx", platform ), ".*%.c$", 1 )
fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/FreakUSB/class/CDC", platform ), ".*%.c$" )
fwlib_files = fwlib_files .. " " .. utils.get_files( sf( "src/platform/%s/generated", platform ), ".*%.c$" )
specific_files = "platform.c platform_int.c pmu.c aes.c aes_config.c"

-- Choose ldscript according to choice of bootloader
if comp.bootloader == 'none' then
  if comp.large_rram then
    print "Selecting large RRAM region (128 bytes)"
    ldscript = sf( "src/platform/%s/%s_large_rram.ld", platform, comp.cpu:lower() )
  else
    ldscript = sf( "src/platform/%s/%s.ld", platform, comp.cpu:lower() )
  end
else
  print "Compiling for FreakUSB bootloader"
  if comp.large_rram then
    print "Selecting large RRAM region (128 bytes)"
    ldscript = sf( "src/platform/%s/%s_%s_large_rram.ld", platform, comp.cpu:lower(), comp.bootloader )
  else
    ldscript = sf( "src/platform/%s/%s_%s.ld", platform, comp.cpu:lower(), comp.bootloader )
  end
  addm{ "USE_BOOTLOADER" }
end


-- Prepend with path
specific_files = fwlib_files .. " " .. utils.prepend_path( specific_files, sf( "src/platform/%s", platform ) )
specific_files = specific_files .. " src/platform/cortex_utils.s src/platform/arm_cortex_interrupts.c"

addm{ "FOR" .. comp.cpu:upper(), 'gcc', 'CORTEX_M3' }

addm{ "__NEWLIB__" }
addm{ "USE_CDC_CLASS" }

-- Standard GCC flags
addcf{ '-ffunction-sections', '-fdata-sections', '-fno-strict-aliasing', '-Wall', '-Wno-error=maybe-uninitialized', '-Wno-unused-function', '-Wno-unused-variable', '-Wno-unused-parameter', '-Wno-variadic-macros', '-W', '-Wextra', '-Wcast-align', '-Wcast-qual', '-Wstrict-aliasing=2', '-Wframe-larger-than=32768', '-Wno-strict-overflow'  }
addcf{ '-Wsync-nand', '-Wtrampolines', '-Wsign-compare', '-Werror=float-equal', '-Werror=missing-braces', '-Werror=init-self', '-Werror=logical-op' }
addcf{ '-Werror=write-strings', '-Werror=address', '-Werror=array-bounds', '-Werror=char-subscripts', '-Werror=enum-compare', '-Werror=implicit-int', '-Werror=empty-body' }

addcf{ '-Werror=main', '-Werror=aggressive-loop-optimizations', '-Werror=nonnull', '-Werror=parentheses', '-Werror=pointer-sign', '-Werror=return-type' }
addcf{ '-Werror=sequence-point', '-Werror=uninitialized', '-Werror=volatile-register-var', '-Werror=ignored-qualifiers', '-Werror=missing-parameter-type' }
addcf{ '-Werror=old-style-declaration', '-Wodr', '-Wformat-signedness', '-Wsuggest-final-types', '-Wsuggest-final-methods', '-Wno-ignored-attributes' }
addcf{ '-Wno-missing-field-initializers', '-Wshift-overflow=2', '-Wduplicated-cond', '-Wduplicated-branches', '-Werror=restrict', '-Wdouble-promotion', '-Wformat=2' }
addcf{ '-Wno-sign-compare', '-Wno-cast-qual', '-Wno-cast-align', '-Wno-format-nonliteral'}
-- Testing below for backtrace functionality
-- addcf{ '-ffunction-sections', '-fdata-sections', '-fno-strict-aliasing', '-Wall' , '-mtpcs-frame', '-mtpcs-leaf-frame', '-fno-omit-frame-pointer' }
addlf{ '-nostartfiles', '-nostdlib', '-T', ldscript, '-Wl,--gc-sections' }
addaf{ '-x', 'assembler-with-cpp', '-Wall' }
addlib{ 'c','gcc','m' }

local target_flags = { '-mcpu=cortex-m3','-mthumb' }

-- Configure general flags for target
addcf{ target_flags, '-mlittle-endian' }
addlf{ target_flags, '-Wl,-static', sf("-Wl,-Map=%s.map",output) }
addaf{ target_flags }
-- Toolset data
tools.sim3u1xx = {}

-- Array of file names that will be checked against the 'prog' target; their absence will force a rebuild
tools.sim3u1xx.prog_flist = { output .. ".hex", output .. ".bin" }

-- We use 'gcc' as the assembler
toolset.asm = toolset.compile

