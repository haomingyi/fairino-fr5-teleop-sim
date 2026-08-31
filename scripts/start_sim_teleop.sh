#!/usr/bin/env bash
# Start the already-installed IPE Quest app, wire its TCP stream to the local
# computer, then open the FR5 + IH01 simulation.  No real robot SDK is loaded.
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
quest_dir="${project_dir}/IPE-quest-hand-teleop"
port="${PORT:-8000}"
side="${SIDE:-right}"
app_id="${APP_ID:-com.haoming.ipe.handteleop}"

[[ -d "${quest_dir}" ]] || { echo "FAIL: missing IPE Quest Hand Teleop project" >&2; exit 2; }
case "${side}" in
  right) mapping_hint="right wrist + right fingers" ;;
  left) mapping_hint="left wrist + left fingers" ;;
  both) mapping_hint="right wrist -> FR5, left fingers -> IH01" ;;
  *) echo "FAIL: SIDE must be right, left, or both." >&2; exit 64 ;;
esac

bash "${quest_dir}/scripts/check_quest.sh"
adb reverse "tcp:${port}" "tcp:${port}"
adb shell am force-stop "${app_id}"
adb shell am start -a android.intent.action.MAIN -c android.intent.category.LAUNCHER -p "${app_id}" >/dev/null
echo "PASS: Quest app started; choose TCP Wired / localhost / ${port} and ${mapping_hint}."

exec make -C "${project_dir}" _sim-viewer LISTEN=1 SIDE="${side}" PORT="${port}"
