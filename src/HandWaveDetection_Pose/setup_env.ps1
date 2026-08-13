# PowerShell Script for Setting Up HandWaveDetection_Pose Environment

$ProjectRoot = $PSScriptRoot
Set-Location $ProjectRoot

Write-Host "===================================================" -ForegroundColor Cyan
Write-Host "  HandWaveDetection_Pose - Windows Setup (PowerShell)" -ForegroundColor Cyan
Write-Host "===================================================" -ForegroundColor Cyan
Write-Host ""

# Check Python
try {
    $pythonVersion = python --version 2>&1
    Write-Host "[*] Python detected: $pythonVersion" -ForegroundColor Green
} catch {
    Write-Host "[ERROR] Python is not installed or not available in PATH." -ForegroundColor Red
    Write-Host "Please install Python 3.10+ and add it to system PATH." -ForegroundColor Red
    Exit 1
}

# Create .venv
if (-not (Test-Path ".venv")) {
    Write-Host "[*] Creating virtual environment (.venv)..." -ForegroundColor Yellow
    python -m venv .venv
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] Failed to create virtual environment." -ForegroundColor Red
        Exit 1
    }
    Write-Host "[OK] Virtual environment created successfully." -ForegroundColor Green
} else {
    Write-Host "[*] Virtual environment (.venv) already exists." -ForegroundColor Yellow
}

# Install dependencies
Write-Host "[*] Upgrading pip..." -ForegroundColor Yellow
& ".\.venv\Scripts\python.exe" -m pip install --upgrade pip

if (Test-Path "requirements-desktop.txt") {
    Write-Host "[*] Installing dependencies from requirements-desktop.txt..." -ForegroundColor Yellow
    & ".\.venv\Scripts\python.exe" -m pip install -r requirements-desktop.txt
} elseif (Test-Path "requirements.txt") {
    Write-Host "[*] Installing dependencies from requirements.txt..." -ForegroundColor Yellow
    & ".\.venv\Scripts\python.exe" -m pip install -r requirements.txt
}

Write-Host ""
Write-Host "===================================================" -ForegroundColor Cyan
Write-Host "[OK] Setup completed successfully!" -ForegroundColor Green
Write-Host "You can run the program using: .\run.ps1" -ForegroundColor Cyan
Write-Host "===================================================" -ForegroundColor Cyan
