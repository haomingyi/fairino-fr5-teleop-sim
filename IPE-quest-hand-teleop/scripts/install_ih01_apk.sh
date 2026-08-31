#!/usr/bin/env bash
set -euo pipefail

adapter_root="$(cd "$(dirname "$0")/.." && pwd)"
# Install only the independently built IH01 application.  The upstream APK is
# retained as provenance/reference and is intentionally never selected here.
apk="${HTS_APK:-$adapter_root/hand_tracking_streamer.apk}"
bash "$adapter_root/scripts/check_quest.sh"
test -s "$apk" || {
  echo "FAIL: IH01 APK not found: $apk" >&2
  echo "Run 'make unity-build' first, or set HTS_APK to an IH01 build." >&2
  exit 5
}
adb install -r "$apk"
echo "PASS: installed IPE Quest Hand Teleop APK from $apk"
