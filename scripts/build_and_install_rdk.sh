#!/usr/bin/env bash
#
# Build flexiv_rdk for ROS 2 Jazzy or Lyrical.
#
# Jazzy uses the vendor's Jazzy-compatible archive and ROS-provided
# dependencies. Lyrical uses the generic archive, builds the RDK dependency
# revisions as position-independent static libraries, and bundles them into a
# symbol-hidden shared library. This prevents the RDK's Fast DDS 2 / Fast-CDR 1
# symbols from colliding with ROS 2 Lyrical's Fast-CDR 2 symbols at runtime.

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

INSTALL_KEY="$(printf '%s' "${INSTALL_DIR}" | sha256sum)"
INSTALL_KEY="${INSTALL_KEY%% *}"

case "${ROS_DISTRO}" in
    jazzy)
        RDK_INSTALL_DIR="${INSTALL_DIR}"
        DEPENDENCY_WORK_DIR="${RDK_DIR}/thirdparty"
        (
            mkdir -p "${DEPENDENCY_WORK_DIR}"
            cd "${DEPENDENCY_WORK_DIR}"
            bash "${RDK_DIR}/thirdparty/build_and_install_dependencies_not_in_ros2.sh" \
                "${RDK_INSTALL_DIR}" "${NUM_JOBS}"
        )
        RDK_JAZZY_OPTION=ON
        ;;
    lyrical)
        RDK_INSTALL_DIR="${INSTALL_DIR}/share/flexiv_rdk_lyrical/private"
        DEPENDENCY_WORK_DIR="${RDK_DIR}/build/thirdparty-static-${INSTALL_KEY:0:16}"
        bash "${SCRIPT_DIR}/build_and_install_rdk_dependencies_static.sh" \
            "${RDK_DIR}" "${RDK_INSTALL_DIR}" "${DEPENDENCY_WORK_DIR}" "${NUM_JOBS}"
        RDK_JAZZY_OPTION=OFF
        ;;
    *)
        echo "ERROR: Unsupported ROS_DISTRO '${ROS_DISTRO}'. Expected jazzy or lyrical." >&2
        exit 1
        ;;
esac

RDK_CMAKE_ARGS=(
    "-DCMAKE_BUILD_TYPE=Release"
    "-DCMAKE_INSTALL_PREFIX=${RDK_INSTALL_DIR}"
    "-DCMAKE_PREFIX_PATH=${RDK_INSTALL_DIR}"
    "-DRDK_SUPPORT_ROS2_JAZZY=${RDK_JAZZY_OPTION}"
)

RDK_BUILD_DIR="${RDK_DIR}/build/rdk-${ROS_DISTRO}-${INSTALL_KEY:0:16}"
cmake --fresh -S "${RDK_DIR}" -B "${RDK_BUILD_DIR}" "${RDK_CMAKE_ARGS[@]}"
cmake --build "${RDK_BUILD_DIR}" --parallel "${NUM_JOBS}"
cmake --install "${RDK_BUILD_DIR}"

if [[ "${ROS_DISTRO}" == "lyrical" ]]; then
    WRAPPER_BUILD_DIR="${RDK_DIR}/build/lyrical-wrapper-${INSTALL_KEY:0:16}"
    cmake --fresh \
        -S "${SCRIPT_DIR}/rdk_lyrical_wrapper" \
        -B "${WRAPPER_BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
        -DFLEXIV_RDK_PRIVATE_PREFIX="${RDK_INSTALL_DIR}"
    cmake --build "${WRAPPER_BUILD_DIR}" --parallel "${NUM_JOBS}"
    cmake --install "${WRAPPER_BUILD_DIR}"

    WRAPPER_LIBRARY="${INSTALL_DIR}/lib/libflexiv_rdk_lyrical.so"
    PRIVATE_NEEDED_PATTERN='lib(fastrtps|fastcdr|foonathan_memory|spdlog|RBDyn|SpaceVecAlg|yaml-cpp|tinyxml2|boost)'
    if readelf -d "${WRAPPER_LIBRARY}" \
        | grep -E "Shared library: \[${PRIVATE_NEEDED_PATTERN}"; then
        echo "ERROR: Lyrical RDK wrapper has a dynamic private dependency." >&2
        exit 1
    fi
    if nm -D --defined-only "${WRAPPER_LIBRARY}" | c++filt \
        | awk '/eprosima::|foonathan::/ { found = 1 } END { exit !found }'; then
        echo "ERROR: Lyrical RDK wrapper exports a private DDS symbol." >&2
        exit 1
    fi
    if ! nm -D --defined-only "${WRAPPER_LIBRARY}" | c++filt \
        | awk '/flexiv::rdk::/ { found = 1 } END { exit !found }'; then
        echo "ERROR: Lyrical RDK wrapper does not export the Flexiv RDK API." >&2
        exit 1
    fi
    echo "Verified symbol-isolated Lyrical RDK wrapper: ${WRAPPER_LIBRARY}"
fi
