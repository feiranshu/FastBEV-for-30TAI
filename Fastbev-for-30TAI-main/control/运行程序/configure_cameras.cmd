@echo off
setlocal
cd /d "%~dp0"
if "%~1"=="" (
  echo Usage: configure_cameras.cmd ^<camera_devices_json^>
  pause
  exit /b 2
)
call ensure_venv.cmd
if errorlevel 1 (
  echo [ERROR] Python environment setup failed.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -u configure_cameras.py "%~1"
pause
