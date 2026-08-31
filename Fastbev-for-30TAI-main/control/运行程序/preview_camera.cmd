@echo off
setlocal
cd /d "%~dp0"
if "%~2"=="" (
  echo Usage: preview_camera.cmd ^<source-index^> ^<camera_devices_json^>
  pause
  exit /b 2
)
call ensure_venv.cmd
if errorlevel 1 (
  echo [ERROR] Python environment setup failed.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -u preview_camera.py "%~2" "%~1"
pause
