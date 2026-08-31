#!/usr/bin/env bash
set -euo pipefail

# Explicit gate before a command-capable EtherCAT session is opened.
# Author: haoming

requested="${1:-}"
if [[ "${requested}" == YES ]]; then
  echo YES
  exit 0
fi
printf 'Hardware mode sends EtherCAT targets. Type YES to continue: ' >&2
IFS= read -r answer || answer=""
[[ "${answer}" == YES ]] || { echo 'Hardware confirmation cancelled.' >&2; exit 64; }
echo YES
