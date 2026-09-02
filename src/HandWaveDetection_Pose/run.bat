@echo off
setlocal

cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo Virtual environment not found. Running setup_env.bat first
    call setup_env.bat
)

echo [*] Starting HandWaveDetection_Pose
.venv\Scripts\python.exe main.py %*
