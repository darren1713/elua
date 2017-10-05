-- gsatmicro build configuration

addm("PCB_V10")
addm( "MEMBRANE_V1" )

return {
  cpu = 'sim3u167',
  components = {
    sercon = { uart = 255, speed = 115200, buf_size = 128 },
    romfs = true,
    wofs = false,
    niffs = true,
    cdc = { buf_size = 128 },
    advanced_shell = false,
    shell_basic = false,
    shell = false,
    term = { lines = 25, cols = 80 },
    adc = { buf_size = 2 },
    xmodem = false,
    cints = true,
    luaints = true
  },
  config = {
    egc = { mode = "alloc" },
    vtmr = { num = 4, freq = 10 },
  },
  modules = {
    generic = { 'all', "-spi", "-pwm", "-can", "-net",  "-lua_math", "-rpc", "-lua_table" }, --"-lua_string",
  },
}
