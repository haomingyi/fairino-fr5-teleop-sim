#!/usr/bin/env bash
set -euo pipefail

command -v adb >/dev/null 2>&1 || { echo "FAIL: adb is not installed." >&2; exit 2; }
devices="$(adb devices -l)"
printf '%s\n' "$devices"
if printf '%s\n' "$devices" | grep -q 'no permissions'; then
  echo "FAIL: USB permission denied; install the existing Quest udev rule first." >&2
  exit 5
fi
if printf '%s\n' "$devices" | grep -q 'unauthorized'; then
  echo "FAIL: unlock Quest and accept USB debugging." >&2
  exit 3
fi
if ! printf '%s\n' "$devices" | awk 'NR > 1 && $2 == "device" {found=1} END {exit !found}'; then
  echo "FAIL: no authorized Quest detected." >&2
  exit 4
fi
echo "PASS: authorized Quest detected."
