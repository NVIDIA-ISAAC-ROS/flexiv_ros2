#!/usr/bin/env bash
#
# Build flexiv_rdk for ROS 2 Jazzy or Lyrical.
#
# Jazzy uses the vendor's Jazzy-compatible archive and ROS-provided
# dependencies. Lyrical uses the generic archive and builds the dependency
# revisions from the RDK manifest into the private install prefix. This keeps
# the RDK's Fast DDS 2 stack separate from Lyrical's Fast DDS 3 packages.

set -euo pipefail

usage() {
    echo "Usage: $0 <install_directory> [num_parallel_jobs]" >&2
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
    usage
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RDK_DIR="$(cd "${SCRIPT_DIR}/../flexiv_hardware/rdk" && pwd)"
INSTALL_DIR="$(realpath -m "$1")"
NUM_JOBS="${2:-$(nproc)}"

if [[ ! "${NUM_JOBS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: num_parallel_jobs must be a positive integer: ${NUM_JOBS}" >&2
    exit 2
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
    echo "ERROR: Source the ROS 2 setup file before running this script." >&2
    exit 1
fi

RDK_CMAKE_ARGS=(
    "-DCMAKE_BUILD_TYPE=Release"
    "-DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}"
    "-DCMAKE_PREFIX_PATH=${INSTALL_DIR}"
)

case "${ROS_DISTRO}" in
    jazzy)
        DEPENDENCY_SCRIPT=build_and_install_dependencies_not_in_ros2.sh
        DEPENDENCY_WORK_DIR="${RDK_DIR}/thirdparty"
        RDK_CMAKE_ARGS+=("-DRDK_SUPPORT_ROS2_JAZZY=ON")
        ;;
    lyrical)
        DEPENDENCY_SCRIPT=build_and_install_dependencies.sh
        INSTALL_KEY="$(printf '%s' "${INSTALL_DIR}" | sha256sum)"
        INSTALL_KEY="${INSTALL_KEY%% *}"
        # Use a clean cache when migrating from the Jazzy dependency build,
        # while retaining incremental builds for the same install prefix.
        DEPENDENCY_WORK_DIR="${RDK_DIR}/build/thirdparty-${INSTALL_KEY:0:16}"
        RDK_CMAKE_ARGS+=("-DRDK_SUPPORT_ROS2_JAZZY=OFF")
        ;;
    *)
        echo "ERROR: Unsupported ROS_DISTRO '${ROS_DISTRO}'. Expected jazzy or lyrical." >&2
        exit 1
        ;;
esac

(
    mkdir -p "${DEPENDENCY_WORK_DIR}"
    cd "${DEPENDENCY_WORK_DIR}"
    bash "${RDK_DIR}/thirdparty/${DEPENDENCY_SCRIPT}" \
        "${INSTALL_DIR}" "${NUM_JOBS}"
)

cmake --fresh -S "${RDK_DIR}" -B "${RDK_DIR}/build" "${RDK_CMAKE_ARGS[@]}"
cmake --build "${RDK_DIR}/build" --parallel "${NUM_JOBS}"
cmake --install "${RDK_DIR}/build"
