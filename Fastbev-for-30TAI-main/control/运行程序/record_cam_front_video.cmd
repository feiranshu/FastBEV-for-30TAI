@echo off
setlocal

net session >nul 2>nul
if errorlevel 1 (
  echo Requesting administrator permission for DirectShow camera access...
  powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "Start-Process -FilePath '%~f0' -ArgumentList '%~1' -Verb RunAs"
  exit /b
)

reg add "HKCU\Software\Microsoft\Windows\CurrentVersion\CapabilityAccessManager\ConsentStore\webcam" /v Value /t REG_SZ /d Allow /f >nul 2>nul
reg add "HKLM\SOFTWARE\Microsoft\Windows Media Foundation\Platform" /v EnableFrameServerMode /t REG_DWORD /d 0 /f >nul 2>nul
reg add "HKLM\SOFTWARE\WOW6432Node\Microsoft\Windows Media Foundation\Platform" /v EnableFrameServerMode /t REG_DWORD /d 0 /f >nul 2>nul

cd /d "%~dp0"
call ensure_venv.cmd
if errorlevel 1 (
  echo [ERROR] Python environment setup failed.
  pause
  exit /b 1
)

if "%~1"=="" (
  ".venv\Scripts\python.exe" -u record_cam_front_video.py
) else (
  ".venv\Scripts\python.exe" -u record_cam_front_video.py --duration-s "%~1"
)
pause
