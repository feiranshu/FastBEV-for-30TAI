@echo off
setlocal
cd /d "%~dp0"

set "BASE_PYTHON="
if exist "D:\python.exe" set "BASE_PYTHON=D:\python.exe"
if not defined BASE_PYTHON if exist "C:\python\python.exe" set "BASE_PYTHON=C:\python\python.exe"

if exist ".venv\Scripts\python.exe" (
  ".venv\Scripts\python.exe" -c "import sys, serial, pygame; print(sys.executable)" >nul 2>nul
  if not errorlevel 1 exit /b 0
  echo [INFO] Existing Python environment is missing packages or broken; installing requirements.
  ".venv\Scripts\python.exe" -m pip install --no-cache-dir -r requirements.txt
  if not errorlevel 1 exit /b 0
  echo [INFO] Package installation failed; rebuilding .venv with --clear.
)

if defined BASE_PYTHON (
  "%BASE_PYTHON%" -m venv --clear ".venv"
) else (
  py -3 -m venv --clear ".venv"
  if errorlevel 9009 python -m venv --clear ".venv"
)
if errorlevel 1 exit /b 1

".venv\Scripts\python.exe" -m pip install --no-cache-dir --upgrade pip
if errorlevel 1 exit /b 1
".venv\Scripts\python.exe" -m pip install --no-cache-dir -r requirements.txt
if errorlevel 1 exit /b 1

exit /b 0
