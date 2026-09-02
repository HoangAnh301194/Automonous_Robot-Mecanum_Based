#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "==================================================="
echo "  HandWaveDetection_Pose - Linux / Jetson Setup"
echo "==================================================="
echo

# Detect architecture
ARCH="$(uname -m)"
echo "[*] Detected Architecture: $ARCH"

# Find python3
if ! command -v python3 >/dev/null 2>&1; then
    echo "[ERROR] python3 is not installed or not in PATH."
    exit 1
fi

PYTHON_VER="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
echo "[*] Python version: $PYTHON_VER"

# Create .venv if not exists
if [[ ! -d ".venv" ]]; then
    echo "[*] Creating virtual environment (.venv)..."
    python3 -m venv --system-site-packages .venv
    echo "[✔] Virtual environment created."
else
    echo "[*] Virtual environment (.venv) already exists."
fi

# Upgrade pip
.venv/bin/python3 -m pip install --upgrade pip

if [[ "$ARCH" == "aarch64" ]]; then
    echo "---------------------------------------------------"
    echo "[!] NVIDIA Jetson Orin Nano (aarch64) detected!"
    echo "[!] PyTorch and ONNX Runtime must be pre-installed"
    echo "    via NVIDIA JetPack wheels for GPU acceleration."
    echo "---------------------------------------------------"
    if [[ -f "requirements.txt" ]]; then
        echo "[*] Installing base requirements..."
        .venv/bin/python3 -m pip install -r requirements.txt || true
    fi
else
    echo "[*] x86_64 Linux PC detected."
    if [[ -f "requirements-desktop.txt" ]]; then
        echo "[*] Installing desktop dependencies..."
        .venv/bin/python3 -m pip install -r requirements-desktop.txt
    elif [[ -f "requirements.txt" ]]; then
        echo "[*] Installing base requirements..."
        .venv/bin/python3 -m pip install -r requirements.txt
    fi
fi

echo
echo "==================================================="
echo "[✔] Setup completed successfully!"
echo "Run application using: ./run.sh"
echo "==================================================="
