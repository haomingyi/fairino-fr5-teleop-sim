#!/usr/bin/env bash
# Prepare the TCP stream and open the FR5 + IH01 simulation.  The Quest app is
# intentionally not launched here; start it manually in the headset so its
# tracking origin and guardian remain under the operator's control.
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
quest_dir="${project_dir}/IPE-quest-hand-teleop"
port="${PORT:-8000}"
side="${SIDE:-right}"

[[ -d "${quest_dir}" ]] || { echo "FAIL: missing IPE Quest Hand Teleop project" >&2; exit 2; }
case "${side}" in
  right) mapping_hint="right wrist + right fingers" ;;
  left) mapping_hint="left wrist + left fingers" ;;
  both) mapping_hint="right wrist -> FR5, left fingers -> IH01" ;;
  *) echo "FAIL: SIDE must be right, left, or both." >&2; exit 64 ;;
esac

bash "${quest_dir}/scripts/check_quest.sh"
adb reverse "tcp:${port}" "tcp:${port}"
echo "PASS: transport ready; manually start Quest app and choose TCP Wired / localhost / ${port} (${mapping_hint})."

exec make -C "${project_dir}" _sim-viewer LISTEN=1 SIDE="${side}" PORT="${port}"
