#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PYTHON="$HOME/.venvs/souta-wakeword/bin/python"
DEFAULT_MODELS_DIR="$HOME/vosk-models"

# 1. Verify Virtual Environment
if [ ! -x "$VENV_PYTHON" ]; then
    echo "[ERROR] Python virtual environment not found at $VENV_PYTHON"
    echo "        Please run installation first:"
    echo "        bash $SCRIPT_DIR/install.sh"
    exit 1
fi

# 2. Check Subcommand
COMMAND="$1"
if [ -z "$COMMAND" ]; then
    echo "Usage: $0 {live|record|evaluate} [options]"
    echo ""
    echo "Examples:"
    echo "  $0 live --debug-transcripts --debug-partials"
    echo "  $0 record --output samples/test.wav --duration 4"
    echo "  $0 evaluate --manifest manifest.csv"
    exit 1
fi

# 3. Check Models (for live & evaluate)
if [ "$COMMAND" = "live" ] || [ "$COMMAND" = "evaluate" ]; then
    MODELS_DIR="$DEFAULT_MODELS_DIR"
    # Check if --models-dir was specified in CLI arguments
    PREV=""
    for ARG in "$@"; do
        if [ "$PREV" = "--models-dir" ]; then
            MODELS_DIR="$ARG"
            break
        fi
        PREV="$ARG"
    done

    MODELS_DIR="${MODELS_DIR/#\~/$HOME}"
    if [ ! -d "$MODELS_DIR" ]; then
        echo "[ERROR] Models directory not found: $MODELS_DIR"
        echo "        Please download the models first by running:"
        echo "        $VENV_PYTHON $SCRIPT_DIR/download_models.py --models-dir $MODELS_DIR"
        exit 1
    fi
fi

# 4. Forward execution to probe.py
exec "$VENV_PYTHON" "$SCRIPT_DIR/probe.py" "$@"
