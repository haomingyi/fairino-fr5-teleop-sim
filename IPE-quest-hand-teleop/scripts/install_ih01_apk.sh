#!/usr/bin/env bash
set -euo pipefail

adapter_root="$(cd "$(dirname "$0")/.." && pwd)"
# Install only the independently built IH01 application.  The upstream APK is
# retained as provenance/reference and is intentionally never selected here.
apk="${HTS_APK:-$adapter_root/hand_tracking_streamer.apk}"
app_id="${APP_ID:-com.haoming.ipe.handteleop}"
bash "$adapter_root/scripts/check_quest.sh"
if [[ "${FORCE_INSTALL:-0}" != "1" ]] &&
   adb shell pm list packages 2>/dev/null | tr -d '\r' | grep -Fxq "package:${app_id}"; then
  echo "PASS: Quest app ${app_id} already installed; skipped adb install to keep the running app open."
  echo "INFO: use FORCE_INSTALL=1 make install only after rebuilding the APK."
  exit 0
fi
test -s "$apk" || {
  echo "FAIL: IH01 APK not found: $apk" >&2
  echo "Run 'make unity-build' first, or set HTS_APK to an IH01 build." >&2
  exit 5
}
adb install -r "$apk"
echo "PASS: installed IPE Quest Hand Teleop APK from $apk"
