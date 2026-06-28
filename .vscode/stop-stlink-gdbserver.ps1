$ErrorActionPreference = 'SilentlyContinue'

Stop-Process -Name 'ST-LINK_gdbserver' -Force
exit 0
