#!/usr/bin/env bash
# Prepare the TCP stream and open the FR5 + IH01 simulation.  The Quest app is
# intentionally not launched here; start it manually in the headset so its
# tracking origin and guardian remain under the operator's control.
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
quest_dir="${project_dir}/IPE-quest-hand-teleop"
port="${PORT:-8000}"
side="${SIDE:-}"

if [[ -z "${side}" ]]; then
  read -r -p "选择 Quest 映射 [r=右手, l=左手, b=双手] (默认 r): " side
fi
case "${side:-r}" in
  r|right) side="right" ;;
  l|left) side="left" ;;
  b|both) side="both" ;;
esac

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

control_file="$(mktemp "/tmp/fr5-arm-teleop-${UID}-XXXXXX.json")"
panel_pid=""
sim_pid=""
cleanup() {
  [[ -z "${panel_pid}" ]] || kill "${panel_pid}" 2>/dev/null || true
  [[ -z "${sim_pid}" ]] || kill "${sim_pid}" 2>/dev/null || true
  rm -f "${control_file}" "${control_file}.status" "${control_file}.tmp" \
    "${control_file}.status.tmp"
}
trap cleanup EXIT INT TERM

# Start MuJoCo first, then start the panel last so it owns keyboard focus;
# E/Space are intentionally handled by the panel, not by the viewer.
make -C "${project_dir}" _sim-viewer LISTEN=1 SIDE="${side}" PORT="${port}" CONTROL_FILE="${control_file}" &
sim_pid=$!
sleep 0.8
python3 "${project_dir}/scripts/arm_teleop_panel.py" \
  --control-file "${control_file}" --side "${side}" &
panel_pid=$!

# End the session when either the viewer or the panel closes.
set +e
wait -n "${sim_pid}" "${panel_pid}"
session_status=$?
set -e
exit "${session_status}"
