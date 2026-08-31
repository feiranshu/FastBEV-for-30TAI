@echo off
setlocal

net session >nul 2>nul
if errorlevel 1 (
  echo Requesting administrator permission for USB/PnP monitor...
  powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "Start-Process -FilePath '%~f0' -ArgumentList '%~1','%~2' -Verb RunAs"
  exit /b
)

cd /d "%~dp0"
set "SECONDS=%~1"
set "INTERVAL=%~2"
if not defined SECONDS set "SECONDS=60"
if not defined INTERVAL set "INTERVAL=200"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0monitor_pc900_pnp.ps1" -Seconds %SECONDS% -IntervalMs %INTERVAL%
pause
