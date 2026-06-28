$ErrorActionPreference = 'Stop'

# Warm up SWD link before launching gdbserver
& 'C:/Progra~1/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe' -c port=SWD freq=400 mode=HOTPLUG | Out-Null
if ($LASTEXITCODE -ne 0) {
    # Fallback only when hotplug cannot see the target.
    & 'C:/Progra~1/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe' -c port=SWD freq=100 mode=UR reset=HWrst | Out-Null
    if ($LASTEXITCODE -ne 0) {
        Write-Error 'SWD link verify failed (hotplug and under-reset).'
        exit 1
    }
}

$proc = Start-Process -FilePath 'C:/Users/marka/AppData/Local/stm32cube/bundles/stlink-gdbserver/7.13.0+st.3/bin/ST-LINK_gdbserver.exe' -ArgumentList @(
    '-c', 'C:/Projects/KAKUTEH7/.vscode/stlink-gdbserver.cfg'
) -WindowStyle Hidden -PassThru

Start-Sleep -Milliseconds 700
if ($proc.HasExited) {
    Write-Error 'ST-LINK_gdbserver exited during startup. Check build/Debug/stlink-gdbserver.log.'
    exit 1
}

$connected = $false
for ($i = 0; $i -lt 8; $i++) {
    try {
        $tcp = New-Object System.Net.Sockets.TcpClient
        $tcp.Connect('127.0.0.1', 50000)
        $tcp.Close()
        $connected = $true
        break
    } catch {
        Start-Sleep -Milliseconds 150
    }
}

if (-not $connected) {
    Write-Error 'ST-LINK_gdbserver is not listening on 127.0.0.1:50000.'
    exit 1
}

exit 0
