#!/usr/bin/env bash
# Quest -> IH01-only simulation. This does not load FR5 or any arm SDK.
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
quest_dir="${project_dir}/IPE-quest-hand-teleop"
port="${PORT:-8000}"
side="${SIDE:-right}"
app_id="${APP_ID:-com.haoming.ipe.handteleop}"

if [[ -z "${SIDE:-}" && -t 0 ]]; then
  printf '选择 Quest 映射手 [right=右手, left=左手, both=双手] (默认 right): '
  read -r requested_side
  side="${requested_side:-right}"
fi
case "${side}" in left|right|both) ;; *) echo "FAIL: SIDE must be left, right, or both." >&2; exit 64 ;; esac
bash "${quest_dir}/scripts/check_quest.sh"
adb reverse "tcp:${port}" "tcp:${port}"
adb shell am force-stop "${app_id}"
adb shell monkey -p "${app_id}" -c android.intent.category.LAUNCHER 1 >/dev/null
echo "PASS: Quest app started; IH01-only simulation SIDE=${side}, TCP Wired / localhost / ${port}."
exec make -C "${quest_dir}" sim SIDE="${side}" HOST=127.0.0.1 PORT="${port}"
