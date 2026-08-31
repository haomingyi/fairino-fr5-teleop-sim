#!/usr/bin/env bash
# Create only the local Python environment required by this portable fork.
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${PYTHON_BIN:-python3}"

"${python_bin}" -m venv "${project_dir}/.venv"
"${project_dir}/.venv/bin/python" -m pip install --upgrade pip
"${project_dir}/.venv/bin/python" -m pip install -r "${project_dir}/requirements.txt"
echo "PASS: local environment ready at ${project_dir}/.venv"
