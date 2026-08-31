#!/usr/bin/env bash
set -euo pipefail

# Interactive side selector shared by simulation and hardware entry points.
# Author: haoming

requested="${1:-}"
case "${requested,,}" in
  left|l) echo left; exit 0 ;;
  right|r) echo right; exit 0 ;;
  both|b) echo both; exit 0 ;;
esac

printf 'Select hand: [l]eft, [r]ight, [b]oth (default: left): ' >&2
IFS= read -r choice || choice=""
case "${choice,,}" in
  ""|left|l) echo left ;;
  right|r) echo right ;;
  both|b) echo both ;;
  *) echo "Invalid hand selection: ${choice}" >&2; exit 64 ;;
esac
