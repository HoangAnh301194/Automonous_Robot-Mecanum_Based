#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$HOME/.venvs/souta-wakeword"
REQ_FILE="$SCRIPT_DIR/requirements.txt"

echo "=========================================================="
echo " Wake Word Phase 1: Environment Installation"
echo " Target Venv: $VENV_DIR"
echo "=========================================================="

# 1. Check OS and Architecture
OS_NAME="$(uname -s)"
ARCH_NAME="$(uname -m)"
echo "[INFO] OS: $OS_NAME, Architecture: $ARCH_NAME"

# 2. Check Python version (>= 3.10 required)
if ! command -v python3 >/dev/null 2>&1; then
    echo "[FAIL] python3 not found. Please install Python 3.10+."
    exit 1
fi

PY_VER="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
echo "[INFO] System Python: $PY_VER"
if python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)'; then
    echo "[PASS] Python version >= 3.10"
else
    echo "[FAIL] Python 3.10 or higher is required. Found: $PY_VER"
    exit 1
fi

# 3. Check python3-venv support
if ! python3 -m venv --help >/dev/null 2>&1; then
    echo "[FAIL] python3-venv package is missing."
    echo "       Install it using: sudo apt-get update && sudo apt-get install -y python3-venv"
    exit 1
fi

# 4. Create isolated virtual environment
if [ ! -d "$VENV_DIR" ]; then
    echo "[INFO] Creating virtual environment at $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
else
    echo "[INFO] Virtual environment already exists at $VENV_DIR"
fi

# 5. Install Python dependencies
VENV_PYTHON="$VENV_DIR/bin/python"
echo "[INFO] Installing Python dependencies from $REQ_FILE..."
"$VENV_PYTHON" -m pip install --upgrade pip setuptools wheel
"$VENV_PYTHON" -m pip install -r "$REQ_FILE"

# 6. Verify vosk import inside venv
if "$VENV_PYTHON" -c "import vosk" >/dev/null 2>&1; then
    VOSK_VER="$("$VENV_PYTHON" -c "import vosk; print(getattr(vosk, '__version__', 'installed'))")"
    echo "[PASS] vosk imported successfully (version: $VOSK_VER)"
else
    echo "[FAIL] Failed to import vosk in $VENV_PYTHON"
    exit 1
fi

# 7. Check system audio utilities (pulseaudio-utils: parec, pactl)
MISSING_SYS_DEPS=()
if ! command -v parec >/dev/null 2>&1; then
    MISSING_SYS_DEPS+=("parec")
fi
if ! command -v pactl >/dev/null 2>&1; then
    MISSING_SYS_DEPS+=("pactl")
fi

if [ ${#MISSING_SYS_DEPS[@]} -eq 0 ]; then
    echo "[PASS] System audio utilities (parec, pactl) are present."
else
    echo "[WARNING] Missing audio utility: ${MISSING_SYS_DEPS[*]}"
    echo "          Run the following command to install:"
    echo "          sudo apt-get update && sudo apt-get install -y pulseaudio-utils"
fi

echo "=========================================================="
echo " Installation completed successfully!"
echo " Next step: Download models via:"
echo "   $VENV_PYTHON $SCRIPT_DIR/download_models.py"
echo "=========================================================="
