-- gsatmicro build configuration

addm("PCB_V8")
addm("LAY_FLAT_COMPASS")
addm("ELUA_BOARD_GSATMICRO_V8")

return {
  cpu = 'sim3u167',
  components = {
    sercon = { uart = 255, speed = 115200, buf_size = 128 },
    romfs = true,
    wofs = true,
    cdc = { buf_size = 128 },
    advanced_shell = false,
    term = { lines = 25, cols = 80 },
    rpc = { uart = 0, speed = 115200 },
    adc = { buf_size = 2 },
    xmodem = false,
    cints = true,
    luaints = true
  },
  config = {
    egc = { mode = "alloc" },
    vtmr = { num = 4, freq = 10 },
    clocks = { external = 8000000, cpu = 168000000 }
  },
  modules = {
    generic = { 'all', "-spi", "-pwm", "-can", "-net" },
  },
}