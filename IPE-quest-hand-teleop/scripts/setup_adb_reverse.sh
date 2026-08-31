#!/usr/bin/env bash
set -euo pipefail

adapter_root="$(cd "$(dirname "$0")/.." && pwd)"
port="${1:-8000}"
bash "$adapter_root/scripts/check_quest.sh"
adb reverse "tcp:$port" "tcp:$port"
adb reverse --list
echo "PASS: Quest localhost:$port -> PC localhost:$port"
