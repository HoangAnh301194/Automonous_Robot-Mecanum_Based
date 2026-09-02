# PowerShell Script for Running HandWaveDetection_Pose

$ProjectRoot = $PSScriptRoot
Set-Location $ProjectRoot

if (-not (Test-Path ".\.venv\Scripts\python.exe")) {
    Write-Host "[!] Virtual environment (.venv) not found. Running setup_env.ps1 first..." -ForegroundColor Yellow
    .\setup_env.ps1
}

Write-Host "[*] Starting HandWaveDetection_Pose..." -ForegroundColor Green
& ".\.venv\Scripts\python.exe" main.py @args
