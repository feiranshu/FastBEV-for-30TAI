@echo off
setlocal
cd /d "%~dp0"
call ensure_venv.cmd
if errorlevel 1 (
  echo [ERROR] Python environment setup failed.
  pause
  exit /b 1
)
echo INPUT ONLY: no serial port, no cameras, and no vehicle motion.
".venv\Scripts\python.exe" -u test_pc900_input.py
pause
