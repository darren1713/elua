-- LPCXpresso LPC1769 build configuration

return {
  cpu = 'lpc1769',
  components = {
    sercon = { uart = 0, speed = 115200 },
    romfs = true,
    shell = true,
    term = { lines = 25, cols = 80 },
    linenoise = { shell_lines = 10, lua_lines = 50 },
    rpc = { uart = 0, speed = 115200 },
    adc = { buf_size = 4 },
    xmodem = true
  },
  config = {
    egc = { mode = "alloc" },
    ram = { internal_rams = 2 },
    clocks = { external = 12000000, cpu = 120000000 }
  },
  modules = {
    generic = { 'all', "-spi", "-i2c", "-net" },
    platform = 'all',
    platform_name = 'lpcxpresso'
  },
}

