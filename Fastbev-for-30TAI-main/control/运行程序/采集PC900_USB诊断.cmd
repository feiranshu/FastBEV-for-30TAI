@echo off
setlocal

net session >nul 2>nul
if errorlevel 1 (
  echo Requesting administrator permission for USB/PnP diagnostics...
  powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "Start-Process -FilePath '%~f0' -ArgumentList '%~1' -Verb RunAs"
  exit /b
)

cd /d "%~dp0"
set "LABEL=%~1"
if not defined LABEL set "LABEL=snapshot"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0collect_pc900_usb_diag.ps1" -Label "%LABEL%"
pause
