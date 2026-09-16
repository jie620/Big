#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
/usr/bin/python3 -m venv --system-site-packages --without-pip "$ROOT/.venv-vla"
"$ROOT/.venv-vla/bin/python" -m pip install -r "$ROOT/requirements-vla.txt"
"$ROOT/.venv-vla/bin/python" -c 'import torch; print(torch.__version__, "CUDA:", torch.cuda.is_available())'
