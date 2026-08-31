#!/usr/bin/env bash
# Build the bundled SOEM backend locally; no parent IH01 repository is needed.
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
soem_root="${project_dir}/third_party/SOEM"
soem_build="${project_dir}/build/soem"
backend="${project_dir}/build/ih01_hand_control_backend"

test -f "${soem_root}/CMakeLists.txt" || { echo "Missing bundled SOEM source." >&2; exit 2; }
# A copied workspace can contain a CMake cache generated in the original
# checkout.  CMake refuses to reuse it because its source/build paths differ.
# Move only this generated directory aside so it remains recoverable, then
# configure a clean cache in the current project.
cache="${soem_build}/CMakeCache.txt"
if test -f "${cache}"; then
  cached_source="$(sed -n 's#^CMAKE_HOME_DIRECTORY:INTERNAL=##p' "${cache}" | head -n 1)"
  if test "${cached_source}" != "${soem_root}"; then
    stale="${soem_build}.stale-$(date +%Y%m%d-%H%M%S)"
    mv "${soem_build}" "${stale}"
    echo "INFO: moved stale CMake cache to ${stale}" >&2
  fi
fi
cmake -S "${soem_root}" -B "${soem_build}" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "${soem_build}" --parallel >/dev/null
soem_library="$(find "${soem_build}" -name libsoem.a -type f -print -quit)"
test -n "${soem_library}" || { echo "SOEM static library was not created." >&2; exit 3; }
mkdir -p "$(dirname "${backend}")"
cc -std=c11 -O2 -Wall -Wextra -Werror \
  -I"${soem_root}/soem" -I"${soem_root}/osal" \
  -I"${soem_root}/osal/linux" -I"${soem_root}/oshw/linux" \
  "${project_dir}/hardware/ethercat/ih01_hand_control_backend.c" \
  "${soem_library}" -lpthread -lrt -o "${backend}"
"${backend}" --dry-run
echo "PASS: built ${backend}"
