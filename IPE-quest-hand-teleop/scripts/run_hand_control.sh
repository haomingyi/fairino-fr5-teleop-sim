#!/usr/bin/env bash
set -euo pipefail

# Start the portable manual IH01 dashboard and map the selected physical slave(s).
# Author: haoming
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
interface="${IH01_INTERFACE:-enp130s0}"
side="$(bash "${project_dir}/scripts/select_side.sh" "${SIDE:-${IH01_HAND_SIDE:-}}")"
slave="${IH01_SLAVE:-1}"
contact_mode="${IH01_CONTACT_HOLD_MODE:-visual}"
contact_delay_ms="${IH01_CONTACT_HOLD_DELAY_MS:-200}"
left_slave=0; right_slave=0
case "${side}" in
  left) left_slave="${slave}" ;;
  right) right_slave="${slave}" ;;
  both)
    left_slave="${IH01_LEFT_SLAVE:-}"; right_slave="${IH01_RIGHT_SLAVE:-}"
    if [[ -z "${left_slave}" || -z "${right_slave}" ]]; then
      printf 'Left slave [default 1]: ' >&2; IFS= read -r left_slave || left_slave=""
      printf 'Right slave [default 2]: ' >&2; IFS= read -r right_slave || right_slave=""
      left_slave="${left_slave:-1}"; right_slave="${right_slave:-2}"
    fi
    if [[ "${left_slave}" == "${right_slave}" || "${left_slave}" == 0 || "${right_slave}" == 0 ]]; then
      echo 'Both-hand mode needs two different non-zero slave numbers.' >&2; exit 64
    fi ;;
  *) echo "Unsupported hand selection: ${side}" >&2; exit 64 ;;
esac
backend="${project_dir}/build/ih01_hand_control_backend"
log="${IH01_HAND_CONTROL_LOG:-artifacts/ih01-hand-control-$(date +%Y%m%d-%H%M%S).jsonl}"
[[ -x "${backend}" ]] || { echo "Backend not built: ${backend}" >&2; exit 68; }
sudo -v
echo "Selected hand: ${side} (left slave=${left_slave}, right slave=${right_slave})"
echo "Protection: per-channel contact=${contact_mode}, current=1000mA, stall=${contact_delay_ms}ms"
echo "Dashboard: sliders, o=open, c=close, r=clear fault, SPACE=CW0 hold, q/Esc=quit"
PYTHONPATH="${project_dir}/runtime${PYTHONPATH:+:${PYTHONPATH}}" \
  exec "${project_dir}/.venv/bin/python" -m ih01_runtime.hardware_control \
  --interface "${interface}" --left-slave "${left_slave}" --right-slave "${right_slave}" \
  --backend "${backend}" --log "${log}" \
  --contact-hold-mode "${contact_mode}" \
  --contact-hold-delay-ms "${contact_delay_ms}"
