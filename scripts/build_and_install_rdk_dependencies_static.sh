#!/usr/bin/env bash
#
# Build the dependency revisions expected by flexiv_rdk as PIC static
# libraries. The resulting archives are consumed only by the Lyrical wrapper;
# they are never added to the ROS workspace's CMake or runtime search paths.

set -euo pipefail

usage() {
    echo "Usage: $0 <rdk_directory> <install_directory> <work_directory> [num_parallel_jobs]" >&2
}

if [[ $# -lt 3 || $# -gt 4 ]]; then
    usage
    exit 2
fi

RDK_DIR="$(realpath "$1")"
STATIC_INSTALL_DIR="$(realpath -m "$2")"
STATIC_WORK_DIR="$(realpath -m "$3")"
STATIC_NUM_JOBS="${4:-$(nproc)}"

if [[ ! "${STATIC_NUM_JOBS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: num_parallel_jobs must be a positive integer: ${STATIC_NUM_JOBS}" >&2
    exit 2
fi

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "ERROR: The ROS 2 Lyrical static RDK wrapper currently supports Linux only." >&2
    exit 1
fi

# The vendor scripts consume these exported names.
export SCRIPT_DIR="${RDK_DIR}/thirdparty"
export INSTALL_DIR="${STATIC_INSTALL_DIR}"
export NUM_JOBS="${STATIC_NUM_JOBS}"
export OS_NAME=Linux
export QNX_TARGET=""
export QNX_ARCH=""
export SHARED_CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_C_VISIBILITY_PRESET=hidden \
    -DCMAKE_CXX_VISIBILITY_PRESET=hidden \
    -DCMAKE_VISIBILITY_INLINES_HIDDEN=ON \
    -DCMAKE_PREFIX_PATH=${STATIC_INSTALL_DIR} \
    -DCMAKE_INSTALL_PREFIX=${STATIC_INSTALL_DIR} \
    -DBUILD_TESTING=OFF"

mkdir -p "${STATIC_WORK_DIR}/cloned"
cd "${STATIC_WORK_DIR}/cloned"

DEPENDENCY_SCRIPTS=(
    install_eigen.sh
    install_spdlog.sh
    install_tinyxml2.sh
    install_yaml-cpp.sh
    install_foonathan_memory.sh
    install_Fast-CDR.sh
    install_Fast-DDS.sh
    install_boost.sh
    install_SpaceVecAlg.sh
    install_RBDyn.sh
)

for dependency_script in "${DEPENDENCY_SCRIPTS[@]}"; do
    bash "${SCRIPT_DIR}/scripts/${dependency_script}"
done
