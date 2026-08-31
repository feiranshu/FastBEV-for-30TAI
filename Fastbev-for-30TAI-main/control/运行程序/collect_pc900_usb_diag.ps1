param(
  [string]$Label = 'snapshot'
)

$ErrorActionPreference = 'Continue'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$checks = Join-Path $root 'checks'
New-Item -ItemType Directory -Force -Path $checks | Out-Null

$safeLabel = ($Label -replace '[^0-9A-Za-z_\-]', '_')
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$out = Join-Path $checks "pc900_usb_diag_${safeLabel}_${stamp}.txt"

function Add-Line([string]$Text = '') {
  $Text | Out-File -LiteralPath $out -Append -Encoding utf8
}

function Add-Section([string]$Title) {
  Add-Line ''
  Add-Line "===== $Title ====="
}

function Run-Text([string]$Title, [scriptblock]$Block) {
  Add-Section $Title
  try {
    & $Block 2>&1 | Out-String -Width 4096 | Out-File -LiteralPath $out -Append -Encoding utf8
  } catch {
    Add-Line "[ERROR] $($_.Exception.Message)"
  }
}

Set-Content -LiteralPath $out -Encoding utf8 -Value 'PC900 / USB / Ethernet diagnostic snapshot'
Add-Line "label=$Label"
Add-Line "timestamp=$(Get-Date -Format 'o')"
Add-Line "computer=$env:COMPUTERNAME"
Add-Line "user=$env:USERNAME"
Add-Line "is_admin=$(([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator))"

Run-Text 'OS' {
  Get-CimInstance Win32_OperatingSystem |
    Select-Object Caption,Version,BuildNumber,OSArchitecture,LastBootUpTime |
    Format-List
}

Run-Text 'BIOS / Computer' {
  Get-CimInstance Win32_BIOS | Select-Object Manufacturer,SMBIOSBIOSVersion,ReleaseDate | Format-List
  Get-CimInstance Win32_ComputerSystem | Select-Object Manufacturer,Model,SystemType | Format-List
}

Run-Text 'PC900 / HID / Game Controller PnP Entities' {
  Get-CimInstance Win32_PnPEntity |
    Where-Object {
      $_.PNPDeviceID -like '*VID_1345&PID_4004*' -or
      $_.Name -match 'PC900|game|controller|joystick|HID-compliant game controller|USB Input Device'
    } |
    Sort-Object PNPClass,Name,PNPDeviceID |
    Select-Object Status,ConfigManagerErrorCode,PNPClass,Name,Manufacturer,Service,PNPDeviceID |
    Format-List
}

Run-Text 'Currently OK USB/HID Device Counts' {
  $all = Get-CimInstance Win32_PnPEntity
  $ok = $all | Where-Object { $_.ConfigManagerErrorCode -eq 0 }
  [pscustomobject]@{
    TotalOk = @($ok).Count
    UsbOk = @(($ok | Where-Object { $_.PNPDeviceID -like 'USB\*' })).Count
    HidOk = @(($ok | Where-Object { $_.PNPClass -eq 'HIDClass' })).Count
    PC900Ok = @(($ok | Where-Object { $_.PNPDeviceID -like '*VID_1345&PID_4004*' })).Count
    LRCP1080POk = @(($ok | Where-Object { $_.PNPDeviceID -like '*VID_1BCF&PID_2CC8*' })).Count
    CH34xOk = @(($ok | Where-Object { $_.PNPDeviceID -like '*VID_1A86*' -or $_.Name -match 'CH340|CH341' })).Count
  } | Format-List
}

Run-Text 'LRCP / CH340 / USB Cameras' {
  Get-CimInstance Win32_PnPEntity |
    Where-Object {
      $_.PNPDeviceID -like '*VID_1BCF&PID_2CC8*' -or
      $_.PNPDeviceID -like '*VID_1A86*' -or
      $_.Name -match 'LRCP|F1080P|CH340|CH341|UVC|WebCam'
    } |
    Sort-Object PNPClass,Name,PNPDeviceID |
    Select-Object Status,ConfigManagerErrorCode,PNPClass,Name,Manufacturer,Service,PNPDeviceID |
    Format-List
}

Run-Text 'USB Controllers' {
  Get-CimInstance Win32_USBController |
    Select-Object Name,Manufacturer,Status,PNPDeviceID |
    Format-List
}

Run-Text 'USB Hubs' {
  Get-CimInstance Win32_USBHub |
    Select-Object Name,Status,PNPDeviceID |
    Sort-Object Name,PNPDeviceID |
    Format-List
}

Run-Text 'Physical Network Adapters' {
  Get-CimInstance Win32_NetworkAdapter |
    Where-Object { $_.PhysicalAdapter } |
    Sort-Object Name |
    Select-Object Name,NetConnectionID,Manufacturer,ServiceName,AdapterType,MACAddress,NetEnabled,Speed,PNPDeviceID |
    Format-List
}

Run-Text 'Network Adapter Driver Details' {
  Get-CimInstance Win32_PnPSignedDriver |
    Where-Object { $_.DeviceClass -eq 'NET' } |
    Sort-Object DeviceName |
    Select-Object DeviceName,Manufacturer,DriverProviderName,DriverVersion,DriverDate,InfName,DeviceID |
    Format-List
}

Run-Text 'USB / HID / Net Filter Drivers' {
  $classes = @(
    @{Name='HIDClass'; Path='HKLM:\SYSTEM\CurrentControlSet\Control\Class\{745A17A0-74D3-11D0-B6FE-00A0C90F57DA}'},
    @{Name='USB'; Path='HKLM:\SYSTEM\CurrentControlSet\Control\Class\{36FC9E60-C465-11CF-8056-444553540000}'},
    @{Name='NET'; Path='HKLM:\SYSTEM\CurrentControlSet\Control\Class\{4d36e972-e325-11ce-bfc1-08002be10318}'}
  )
  foreach ($class in $classes) {
    "[$($class.Name)] $($class.Path)"
    if (Test-Path $class.Path) {
      Get-ItemProperty -LiteralPath $class.Path |
        Select-Object UpperFilters,LowerFilters,Class,ClassDesc |
        Format-List
    } else {
      'missing'
    }
  }
}

Run-Text 'USB Selective Suspend Power Setting' {
  powercfg /QUERY SCHEME_CURRENT 2a737441-1930-4402-8d77-b2bebba308a3 48e6b7a6-50f5-4782-a5d4-53bb8f07e226
}

Run-Text 'Recent System PnP / USB / HID / Network Events' {
  $since = (Get-Date).AddHours(-2)
  Get-WinEvent -FilterHashtable @{LogName='System'; StartTime=$since} -ErrorAction SilentlyContinue |
    Where-Object {
      $_.ProviderName -match 'PnP|USB|HID|DriverFrameworks|Kernel|NDIS|Net|Tcpip|Dhcp|Realtek|Intel|Rtl|e1d|rt640|rt68'
    } |
    Select-Object TimeCreated,Id,LevelDisplayName,ProviderName,Message |
    Sort-Object TimeCreated |
    Format-List
}

Run-Text 'DriverFrameworks UserMode Events' {
  $since = (Get-Date).AddHours(-2)
  Get-WinEvent -FilterHashtable @{LogName='Microsoft-Windows-DriverFrameworks-UserMode/Operational'; StartTime=$since} -ErrorAction SilentlyContinue |
    Select-Object TimeCreated,Id,LevelDisplayName,ProviderName,Message |
    Sort-Object TimeCreated |
    Format-List
}

Run-Text 'pnputil Connected HIDClass' {
  pnputil /enum-devices /connected /class HIDClass
}

Run-Text 'pnputil Connected USB' {
  pnputil /enum-devices /connected /class USB
}

Run-Text 'pnputil Connected Net' {
  pnputil /enum-devices /connected /class Net
}

Add-Line ''
Add-Line "DONE: $out"
Write-Host 'Saved diagnostic snapshot:'
Write-Host $out
