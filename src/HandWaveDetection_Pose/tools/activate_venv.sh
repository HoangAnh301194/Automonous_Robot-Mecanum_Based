#!/usr/bin/env bash

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_root/.venv/bin/activate"
site_packages="$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
torch_lib="$site_packages/torch/lib"
case ":${LD_LIBRARY_PATH:-}:" in
    *":$torch_lib:"*) ;;
    *) export LD_LIBRARY_PATH="$torch_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
esac
unset project_root site_packages torch_lib
