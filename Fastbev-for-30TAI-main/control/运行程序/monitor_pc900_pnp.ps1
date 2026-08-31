param(
  [int]$Seconds = 60,
  [int]$IntervalMs = 200
)

$ErrorActionPreference = 'Continue'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$checks = Join-Path $root 'checks'
New-Item -ItemType Directory -Force -Path $checks | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$out = Join-Path $checks "pc900_pnp_monitor_${stamp}.csv"

'timestamp,pc900_ok,usb_ok,hid_ok,realtek_enabled,realtek_speed,pc900_ids' |
  Set-Content -LiteralPath $out -Encoding utf8

$deadline = (Get-Date).AddSeconds($Seconds)
Write-Host 'Monitoring PC900 PnP state. Plug/unplug the Ethernet cable now.'
Write-Host "Output: $out"

while ((Get-Date) -lt $deadline) {
  try {
    $all = Get-CimInstance Win32_PnPEntity
    $ok = $all | Where-Object { $_.ConfigManagerErrorCode -eq 0 }
    $pc900 = @($ok | Where-Object { $_.PNPDeviceID -like '*VID_1345&PID_4004*' })
    $realtek = Get-CimInstance Win32_NetworkAdapter |
      Where-Object { $_.PNPDeviceID -like 'PCI\VEN_10EC&DEV_8168*' } |
      Select-Object -First 1

    $row = [pscustomobject]@{
      timestamp = (Get-Date -Format 'o')
      pc900_ok = $pc900.Count
      usb_ok = @(($ok | Where-Object { $_.PNPDeviceID -like 'USB\*' })).Count
      hid_ok = @(($ok | Where-Object { $_.PNPClass -eq 'HIDClass' })).Count
      realtek_enabled = if ($realtek) { [string]$realtek.NetEnabled } else { '' }
      realtek_speed = if ($realtek) { [string]$realtek.Speed } else { '' }
      pc900_ids = (($pc900 | ForEach-Object { $_.PNPDeviceID }) -join '|')
    }
    $line = '"{0}",{1},{2},{3},"{4}","{5}","{6}"' -f
      $row.timestamp, $row.pc900_ok, $row.usb_ok, $row.hid_ok,
      $row.realtek_enabled, $row.realtek_speed, ($row.pc900_ids -replace '"', '""')
    Add-Content -LiteralPath $out -Encoding utf8 -Value $line
    Write-Host ("{0} PC900={1} USB={2} HID={3} Realtek={4}" -f
      (Get-Date -Format 'HH:mm:ss.fff'), $row.pc900_ok, $row.usb_ok, $row.hid_ok, $row.realtek_enabled)
  } catch {
    $line = '"{0}",error,error,error,"","","{1}"' -f (Get-Date -Format 'o'), ($_.Exception.Message -replace '"', '""')
    Add-Content -LiteralPath $out -Encoding utf8 -Value $line
    Write-Host ("{0} ERROR {1}" -f (Get-Date -Format 'HH:mm:ss.fff'), $_.Exception.Message)
  }
  Start-Sleep -Milliseconds $IntervalMs
}

Write-Host "Saved monitor log:"
Write-Host $out
