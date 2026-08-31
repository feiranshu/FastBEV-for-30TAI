@echo off
setlocal
cd /d "%~dp0"
call ensure_venv.cmd
if errorlevel 1 (
  echo [ERROR] Python environment setup failed.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -u check_chassis.py
pause
