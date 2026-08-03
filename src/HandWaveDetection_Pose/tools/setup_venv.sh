#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
uv_bin="${UV_BIN:-$HOME/.local/bin/uv}"

if [[ ! -x "$uv_bin" ]]; then
    if command -v uv >/dev/null 2>&1; then
        uv_bin="$(command -v uv)"
    else
        python3 -m pip install --user --upgrade uv
    fi
fi

"$uv_bin" python install 3.14.6
if [[ -e "$project_root/.venv" ]]; then
    python3 -c 'import shutil, sys; shutil.rmtree(sys.argv[1])' "$project_root/.venv"
fi
"$uv_bin" venv --python 3.14.6 "$project_root/.venv"
"$uv_bin" pip install --python "$project_root/.venv/bin/python" \
    --link-mode copy \
    --index-url https://download.pytorch.org/whl/cpu \
    'torch==2.13.0+cpu' \
    'torchvision==0.28.0+cpu'
"$uv_bin" pip install --python "$project_root/.venv/bin/python" \
    --link-mode copy \
    -r "$project_root/requirements-desktop.txt"

printf '%s\n' 'Environment ready.'
printf '%s\n' 'Activate with: source tools/activate_venv.sh'
