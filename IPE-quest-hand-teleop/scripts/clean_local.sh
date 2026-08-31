#!/usr/bin/env bash
# Generated caches only.  The default is a dry run so source/assets are never
# removed accidentally.  Re-run with HTS_CONFIRM_CLEAN_LOCAL=YES to delete.
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
targets=(
  "${project_dir}/.venv"
  "${project_dir}/build"
  "${project_dir}/artifacts"
  "${project_dir}/hand_tracking_streamer/Library"
  "${project_dir}/hand_tracking_streamer/Temp"
  "${project_dir}/hand_tracking_streamer/Logs"
  "${project_dir}/hand_tracking_streamer/UserSettings"
  "${project_dir}/hand_tracking_streamer/Builds"
)

echo "Generated-only cleanup candidates:"
for target in "${targets[@]}"; do
  [[ -e "${target}" ]] && du -sh "${target}"
done
if [[ "${HTS_CONFIRM_CLEAN_LOCAL:-}" != "YES" ]]; then
  echo "Dry run only. Set HTS_CONFIRM_CLEAN_LOCAL=YES to remove exactly the paths above."
  exit 0
fi
for target in "${targets[@]}"; do
  [[ -e "${target}" ]] && rm -rf -- "${target}"
done
echo "PASS: generated local files removed."
