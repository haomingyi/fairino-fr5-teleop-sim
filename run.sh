#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
export PYTHONPATH="${PWD}/third_party/fairino-python-sdk/linux:${PWD}/scripts${PYTHONPATH:+:$PYTHONPATH}"
exec python3 scripts/fr5_teach_ui.py "$@"
