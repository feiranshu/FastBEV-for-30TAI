@echo off
setlocal
cd /d "%~dp0"
call ensure_venv.cmd
if errorlevel 1 (
  echo [ERROR] Python environment setup failed.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -u web_remote.py ^
  --execute ^
  --serial-port COM3 ^
  --vehicle-live ^
  --vehicle-edge-host 192.168.125.166 ^
  --vehicle-edge-port 5200 ^
  --vehicle-viewer http://127.0.0.1:8093 ^
  --open-browser
pause
