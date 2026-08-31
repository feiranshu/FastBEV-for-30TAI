@echo off
setlocal

net session >nul 2>nul
if errorlevel 1 (
  echo Requesting administrator permission for DirectShow camera access...
  powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
  exit /b
)

reg add "HKCU\Software\Microsoft\Windows\CurrentVersion\CapabilityAccessManager\ConsentStore\webcam" /v Value /t REG_SZ /d Allow /f >nul 2>nul
reg add "HKLM\SOFTWARE\Microsoft\Windows Media Foundation\Platform" /v EnableFrameServerMode /t REG_DWORD /d 0 /f >nul 2>nul
reg add "HKLM\SOFTWARE\WOW6432Node\Microsoft\Windows Media Foundation\Platform" /v EnableFrameServerMode /t REG_DWORD /d 0 /f >nul 2>nul

cd /d "%~dp0"
set "SERIAL_PORT=AUTO"
set "APP_DIR="
for /d %%D in (*) do if exist "%%~fD\web_remote.py" set "APP_DIR=%%~fD"
if not defined APP_DIR (
  echo [ERROR] Cannot find the runtime folder containing web_remote.py.
  pause
  exit /b 2
)
cd /d "%APP_DIR%"
call ensure_venv.cmd
if errorlevel 1 (
  echo [ERROR] Python environment setup failed. Run setup.cmd in the runtime folder and check the error above.
  pause
  exit /b 1
)
echo Starting the local six-camera web dashboard...
".venv\Scripts\python.exe" -u web_remote.py --execute --serial-port "%SERIAL_PORT%" --pc900 --open-browser
pause
