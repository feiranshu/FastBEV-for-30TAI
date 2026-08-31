@echo off
setlocal
cd /d "%~dp0"
call ensure_venv.cmd
if errorlevel 1 (
  echo [ERROR] Setup failed.
  pause
  exit /b 1
)
echo.
echo Setup complete. Run probe_cameras.cmd next.
pause
