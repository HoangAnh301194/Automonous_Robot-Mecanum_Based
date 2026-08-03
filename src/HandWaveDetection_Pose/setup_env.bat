@echo off
setlocal

echo ===================================================
echo   HandWaveDetection_Pose - Windows Environment Setup
echo ===================================================
echo.

cd /d "%~dp0"

:: Check Python
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python is not installed or not added to PATH.
    echo Please install Python 3.10+ from python.org and check "Add Python to PATH".
    pause
    exit /b 1
)

echo [*] Python detected:
python --version

:: Create virtual environment if not exists
if not exist ".venv" (
    echo [*] Creating virtual environment (.venv)
    python -m venv .venv
    if errorlevel 1 (
        echo [ERROR] Failed to create virtual environment.
        pause
        exit /b 1
    )
    echo [OK] Virtual environment created successfully.
) else (
    echo [*] Virtual environment (.venv) already exists.
)

:: Upgrade pip and install requirements
echo [*] Upgrading pip
call .venv\Scripts\python.exe -m pip install --upgrade pip

if exist "requirements-desktop.txt" (
    echo [*] Installing dependencies from requirements-desktop.txt
    call .venv\Scripts\python.exe -m pip install -r requirements-desktop.txt
    goto finish
)

if exist "requirements.txt" (
    echo [*] Installing dependencies from requirements.txt
    call .venv\Scripts\python.exe -m pip install -r requirements.txt
    goto finish
)

:finish
echo.
echo ===================================================
echo [OK] Setup completed successfully!
echo You can run the program using: run.bat
echo ===================================================
echo.
pause
