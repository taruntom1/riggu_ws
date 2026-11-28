<#
.SYNOPSIS
  Forward Windows localhost ports to the Pi on the Windows mobile hotspot.

.DESCRIPTION
  WSL mirrored networking does not mirror the mobile-hotspot adapter, so WSL and
  Docker cannot reach 192.168.137.0/24 directly. This adds netsh portproxy rules:
    127.0.0.1:2222  -> <pi>:22    (SSH)
    127.0.0.1:17447 -> <pi>:7447  (rmw_zenoh router)
    127.0.0.1:47620 -> <pi>:47620 (DriveLink agent, for the DriveLink desktop app)
  Non-default local ports avoid clashing with a local zenoh router in WSL.
  Run as Administrator. Re-run if the Pi's hotspot IP changes.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File riggu_portproxy.ps1
  powershell -ExecutionPolicy Bypass -File riggu_portproxy.ps1 -Remove
#>
param(
  [string]$PiHost = "riggu.local",
  [switch]$Remove
)
$ErrorActionPreference = "Stop"

$rules = @(
  @{ Listen = 2222;  Target = 22 },
  @{ Listen = 17447; Target = 7447 },
  @{ Listen = 47620; Target = 47620 }
)

$principal = [Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
  throw "Run this script as Administrator."
}

foreach ($r in $rules) {
  cmd /c "netsh interface portproxy delete v4tov4 listenaddress=127.0.0.1 listenport=$($r.Listen) >nul 2>&1"
}
if ($Remove) { Write-Host "riggu port forwards removed."; return }

$ip = Resolve-DnsName $PiHost -Type A |
  Where-Object { $_.IPAddress } |
  Select-Object -First 1 -ExpandProperty IPAddress
if (-not $ip) { throw "Cannot resolve $PiHost. Is the Pi on the hotspot with avahi-daemon running?" }

# portproxy needs the IP Helper service.
Set-Service iphlpsvc -StartupType Automatic
Start-Service iphlpsvc

foreach ($r in $rules) {
  netsh interface portproxy add v4tov4 listenaddress=127.0.0.1 listenport=$($r.Listen) `
    connectaddress=$ip connectport=$($r.Target) | Out-Null
}
Write-Host "Forwarding to $PiHost ($ip):"
netsh interface portproxy show v4tov4
