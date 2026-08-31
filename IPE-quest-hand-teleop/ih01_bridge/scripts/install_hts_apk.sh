#!/usr/bin/env bash
set -euo pipefail

adapter_root="$(cd "$(dirname "$0")/.." && pwd)"
apk="${HTS_APK:-$adapter_root/upstream/hand-tracking-streamer/hand_tracking_streamer.apk}"
bash "$adapter_root/scripts/check_quest.sh"
test -s "$apk" || {
  echo "FAIL: HTS APK not found: $apk" >&2
  echo "Set HTS_APK=/absolute/path/hand_tracking_streamer.apk" >&2
  exit 5
}
adb install -r "$apk"
echo "PASS: installed official Hand Tracking Streamer APK from $apk"
