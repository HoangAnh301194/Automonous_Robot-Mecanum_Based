#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ ! -f ".venv/bin/python3" ]]; then
    echo "[!] Virtual environment (.venv) not found. Running ./setup_env.sh..."
    ./setup_env.sh
fi

echo "[*] Starting HandWaveDetection_Pose..."
exec .venv/bin/python3 main.py "$@"
