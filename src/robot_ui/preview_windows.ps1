param(
    [ValidateRange(1, 65535)]
    [int]$Port = 5173,
    [switch]$NoBrowser
)

$ErrorActionPreference = "Stop"
$minimumNodeVersion = [version]"20.19.0"
$scriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$frontendDirectory = Join-Path $scriptDirectory "frontend"

if (-not (Get-Command node.exe -ErrorAction SilentlyContinue)) {
    throw "Node.js is missing. Install Node.js 20.19 or newer, then run this script again."
}

if (-not (Get-Command npm.cmd -ErrorAction SilentlyContinue)) {
    throw "npm is missing. Reinstall Node.js with npm enabled."
}

$nodeVersionText = (& node.exe --version).Trim().TrimStart("v").Split("-")[0]
$nodeVersion = [version]$nodeVersionText
if ($nodeVersion -lt $minimumNodeVersion) {
    throw "Node.js $nodeVersion is too old. Install Node.js 20.19 or newer."
}

if (-not (Test-Path -LiteralPath $frontendDirectory -PathType Container)) {
    throw "Frontend directory not found: $frontendDirectory"
}

$previewUrl = "http://127.0.0.1:$Port/"

Write-Host "Robot UI Windows preview"
Write-Host "Local:  $previewUrl"

$lanAddresses = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object {
        $_.IPAddress -notlike "127.*" -and
        $_.IPAddress -notlike "169.254.*"
    } |
    Select-Object -ExpandProperty IPAddress -Unique

foreach ($address in $lanAddresses) {
    Write-Host "LAN:    http://${address}:$Port/"
}

Write-Host "Mode:   frontend demo data; ROS 2 and Jetson are not required."
Write-Host "Stop:   press Ctrl+C"

Push-Location $frontendDirectory
try {
    if (-not (Test-Path -LiteralPath (Join-Path $frontendDirectory "node_modules"))) {
        Write-Host "Installing frontend dependencies..."
        & npm.cmd install
        if ($LASTEXITCODE -ne 0) {
            throw "npm install failed with exit code $LASTEXITCODE."
        }
    }

    if (-not $NoBrowser) {
        $openCommand = "Start-Sleep -Seconds 2; Start-Process '$previewUrl'"
        Start-Process powershell.exe -WindowStyle Hidden -ArgumentList @(
            "-NoProfile",
            "-WindowStyle",
            "Hidden",
            "-Command",
            $openCommand
        ) | Out-Null
    }

    & npm.cmd run dev -- --port $Port
    if ($LASTEXITCODE -ne 0) {
        throw "Vite preview failed with exit code $LASTEXITCODE."
    }
}
finally {
    Pop-Location
}
