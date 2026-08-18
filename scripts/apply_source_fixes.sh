#!/usr/bin/env bash
#
# apply_source_fixes.sh
#
# Copies known-good source files over the pristine sources that `vcs import`
# pulls in for the ABI-pinned third_party packages (ros2_control /
# ros2_controllers). Run this AFTER:
#
#     cd ${ISAAC_ROS_WS}
#     vcs import src < src/flexiv_ros2/flexiv.repos
#
# Why full-file overrides instead of `patch`:
#   * third_party/ros2_control and ros2_controllers are pinned to fixed git
#     tags (ABI-locked to the prebuilt flexiv_rdk v1.8 archive), so these
#     upstream files never change. Copying a vetted file is simpler and more
#     robust to maintain than a context-sensitive diff.
#
# To add another override:
#   1. Drop the corrected file under scripts/overrides/, mirroring its path
#      relative to ${ISAAC_ROS_WS}/src
#      (e.g. scripts/overrides/third_party/<pkg>/path/to/file.cpp).
#   2. Add a `apply_override "<that relative path>"` line in the list below.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERRIDES_DIR="${SCRIPT_DIR}/overrides"

# Resolve the workspace 'src' directory that contains flexiv_ros2.
# scripts/ -> flexiv_ros2/ -> src/
SRC_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

apply_override() {
  # $1: path relative to BOTH scripts/overrides/ and ${SRC_DIR}
  local rel="$1"
  local src="${OVERRIDES_DIR}/${rel}"
  local dst="${SRC_DIR}/${rel}"

  if [[ ! -f "${src}" ]]; then
    echo "ERROR: override source missing: ${src}" >&2
    return 1
  fi
  if [[ ! -f "${dst}" ]]; then
    echo "ERROR: target not found (did you run 'vcs import' first?): ${dst}" >&2
    return 1
  fi

  if cmp -s "${src}" "${dst}"; then
    echo "[skip]  already up to date: ${rel}"
    return 0
  fi

  install -m 0644 "${src}" "${dst}"
  echo "[apply] ${rel}"
}

echo "Applying source overrides into: ${SRC_DIR}"

# --- list of overrides to apply ---
apply_override "third_party/ros2_control/hardware_interface/src/lexical_casts.cpp"
apply_override "third_party/ros2_controllers/joint_trajectory_controller/CMakeLists.txt"
apply_override "third_party/ros2_controllers/joint_trajectory_controller/package.xml"
apply_override "third_party/ros2_controllers/joint_trajectory_controller/include/joint_trajectory_controller/validate_jtc_parameters.hpp"
# apply_override "third_party/<pkg>/path/to/another/file.cpp"

echo "Done."
