#!/usr/bin/env bash
# Standalone MPPI deployment for an existing Ubuntu 22.04 / ROS 2 Humble setup.
# Usage: bash scripts/install_mppi.sh
# Optional: DEERGO_MPPI_BACKEND=auto|cpu|gpu, DEERGO_WS=/path/to/workspace

log() {
    printf '\n[DEERGO MPPI] %s\n' "$*"
}

warn() {
    printf '\n[WARNING] %s\n' "$*" >&2
}

die() {
    printf '\n[ERROR] %s\n' "$*" >&2
    exit 1
}

deergo_mppi_main() {
    case "${1:-}" in
        -h|--help)
            cat <<'EOF'
Usage: bash scripts/install_mppi.sh

Install MPPI support into an existing DeerGo ROS 2 Humble workspace.
This script does not install ROS 2 Desktop, set up networking, install ZED,
or modify ~/.bashrc.

Environment variables:
  DEERGO_MPPI_BACKEND=auto|cpu|gpu  Default: auto (CUDA with CPU fallback).
  DEERGO_WS=/path/to/workspace     Default: this repository's src/deeergo_ws.

Examples (from the repository root):
  bash scripts/install_mppi.sh
  DEERGO_MPPI_BACKEND=cpu bash scripts/install_mppi.sh
  DEERGO_MPPI_BACKEND=gpu bash scripts/install_mppi.sh
EOF
            return 0
            ;;
        "") ;;
        *) die "Unknown argument: $1. Use --help for installation options." ;;
    esac
    (( $# == 0 )) || die "Unexpected arguments. Use --help for installation options."
    DEERGO_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
    DEERGO_WS="${DEERGO_WS:-${DEERGO_SCRIPT_DIR}/../src/deeergo_ws}"
    DEERGO_MPPI_BACKEND="${DEERGO_MPPI_BACKEND:-auto}"
    [[ "$EUID" -ne 0 ]] || die "Run as a normal user, without sudo."
    [[ -r /etc/os-release ]] || die "Cannot read /etc/os-release."
    # shellcheck disable=SC1091
    source /etc/os-release
    [[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] \
        || die "Ubuntu 22.04 and an existing ROS 2 Humble installation are required."
    [[ -f "${DEERGO_WS}/src/deergo_mqtt_bridge/package.xml" ]] \
        || die "DeerGo workspace not found: ${DEERGO_WS}"
    log "Workspace: ${DEERGO_WS}"
    install_deergo_mppi
    log "Source ${DEERGO_WS}/install/setup.bash before launching."
}

deergo_mppi_cuda_toolkit() {
    local candidate
    for candidate in "${CUDACXX:-}" /usr/local/cuda-12.8/bin/nvcc \
        /usr/local/cuda/bin/nvcc "$(command -v nvcc || true)"; do
        if [[ -n "$candidate" && -x "$candidate" ]]; then
            DEERGO_MPPI_NVCC="$candidate"
            return 0
        fi
    done
    [[ "$(uname -m)" == x86_64 ]] || return 1
    log "Installing CUDA toolkit only; retaining the NVIDIA driver."
    if ! dpkg-query -W -f='${Status}' cuda-keyring 2>/dev/null \
        | grep -q 'install ok installed'; then
        local keyring
        keyring="$(mktemp /tmp/deergo-mppi-cuda.XXXXXX.deb)"
        curl -fL --retry 2 -o "$keyring" \
            https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
            || return 1
        sudo dpkg -i "$keyring" || return 1
        rm -f "$keyring"
    fi
    sudo apt-get update || return 1
    sudo apt-get install -y --no-install-recommends cuda-toolkit-12-8 || return 1
    DEERGO_MPPI_NVCC=/usr/local/cuda-12.8/bin/nvcc
}

deergo_build_mppi_cuda() {
    local revision=b1f53ed348652e4040321ea9b97a8d2808b08a4c
    local source_dir="${DEERGO_WS}/src/mppi_controller_cuda"
    local patch
    local patches=(mppi-cuda-b1f53ed-collision-check.patch
        mppi-cuda-b1f53ed-shared-profile.patch)
    local architecture="$1"
    deergo_mppi_cuda_toolkit || return 1
    if [[ ! -e "$source_dir" ]]; then
        mkdir -p "$source_dir"
        git -C "$source_dir" init || return 1
        git -C "$source_dir" remote add origin \
            https://github.com/datledoan/mppi_controller_cuda.git || return 1
        git -C "$source_dir" fetch --depth 1 origin "$revision" || return 1
        git -C "$source_dir" checkout --detach FETCH_HEAD || return 1
    fi
    [[ "$(git -C "$source_dir" rev-parse HEAD)" == "$revision" ]] || {
        warn "Existing CUDA MPPI source is not the pinned revision; preserved."
        return 1
    }
    for patch in "${patches[@]}"; do
        patch="${DEERGO_SCRIPT_DIR}/patches/${patch}"
        if git -C "$source_dir" apply --reverse --check "$patch" 2>/dev/null; then
            log "CUDA MPPI patch already applied: $(basename "$patch")"
        elif git -C "$source_dir" apply --check "$patch"; then
            git -C "$source_dir" apply "$patch" || return 1
        else
            warn "CUDA MPPI patch conflicts with existing source changes; preserved."
            return 1
        fi
    done
    rm -f "$source_dir/COLCON_IGNORE"
    local toolkit_root batch_size
    toolkit_root="$(dirname "$(dirname "$(readlink -f "$DEERGO_MPPI_NVCC")")")"
    batch_size="$(/usr/bin/python3 -s -c '
import sys, yaml
with open(sys.argv[1]) as stream:
    print(yaml.safe_load(stream)["controller_server"]["ros__parameters"]["FollowPath"]["batch_size"])
' "${DEERGO_WS}/src/deergo_mqtt_bridge/config/nav2_mppi_params.yaml")" || return 1
    [[ "$batch_size" =~ ^[0-9]+$ ]] && (( batch_size >= 32 )) || return 1
    (
        cd "$DEERGO_WS"
        export PATH="${toolkit_root}/bin:$PATH"
        export PYTHONNOUSERSITE=1
        MAKEFLAGS=-j2 colcon build --packages-select mppi_controller_cuda \
            --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release \
            "-DCMAKE_CUDA_COMPILER=${DEERGO_MPPI_NVCC}" \
            "-DCUDAToolkit_ROOT=${toolkit_root}" \
            "-DCMAKE_CUDA_ARCHITECTURES=${architecture}" \
            "-DDEERGO_MPPI_BATCH_SIZE=${batch_size}" \
            -DCMAKE_CUDA_FLAGS=--expt-relaxed-constexpr
    )
}

install_deergo_mppi() {
    case "$DEERGO_MPPI_BACKEND" in
        auto|cpu|gpu) ;;
        *) die "DEERGO_MPPI_BACKEND must be auto, cpu or gpu." ;;
    esac
    [[ -f /opt/ros/humble/setup.bash ]] || die "ROS 2 Humble is required for MPPI."
    export PYTHONNOUSERSITE=1
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    local dependency
    local missing=()
    local dependencies=(ros-humble-nav2-mppi-controller build-essential cmake git curl
        python3-yaml python3-colcon-common-extensions)
    for dependency in "${dependencies[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$dependency" 2>/dev/null \
            | grep -q 'install ok installed'; then
            missing+=("$dependency")
        fi
    done
    if (( ${#missing[@]} > 0 )); then
        sudo apt-get update
        sudo apt-get install -y "${missing[@]}"
    fi
    local probe="${DEERGO_WS}/src/deergo_mqtt_bridge/deergo_mqtt_bridge/mppi_backend.py"
    local report architecture gpu_ready=0
    mkdir -p "${DEERGO_WS}/src"
    if [[ "$DEERGO_MPPI_BACKEND" != cpu ]] \
        && report="$(/usr/bin/python3 -s "$probe" --probe-driver 2>/dev/null)"; then
        architecture="$(/usr/bin/python3 -s -c 'import json,sys; print(json.load(sys.stdin)["architecture"])' <<< "$report")"
        log "Preparing CUDA MPPI for compute capability ${architecture}."
        if deergo_build_mppi_cuda "$architecture"; then
            gpu_ready=1
        fi
    fi
    if [[ "$gpu_ready" == 0 ]]; then
        if [[ "$DEERGO_MPPI_BACKEND" == gpu ]]; then
            die "CUDA MPPI explicitly required but GPU/toolkit/build is unavailable."
        fi
        log "Using official CPU MPPI."
        if [[ -d "${DEERGO_WS}/src/mppi_controller_cuda" ]]; then
            touch "${DEERGO_WS}/src/mppi_controller_cuda/COLCON_IGNORE"
        fi
    fi
    (
        cd "$DEERGO_WS"
        MAKEFLAGS=-j2 colcon build --packages-up-to deergo_mqtt_bridge \
            --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
    )
    set +u
    source "${DEERGO_WS}/install/setup.bash"
    set -u
    ros2 pkg prefix nav2_mppi_controller >/dev/null
    # Remove the obsolete generated install copy; configuration now lives in one YAML.
    rm -f "${DEERGO_WS}/install/deergo_mqtt_bridge/share/deergo_mqtt_bridge/config/nav2_mppi_cuda_overrides.yaml"
    if [[ "$gpu_ready" == 1 ]]; then
        if /usr/bin/python3 -s "$probe" --probe-gpu; then
            log "CUDA MPPI installed; the installed CUDA kernel probe passed."
        elif [[ "$DEERGO_MPPI_BACKEND" == gpu ]]; then
            die "CUDA MPPI build succeeded but its kernel probe failed."
        else
            warn "CUDA kernel probe failed; launch auto mode will use CPU MPPI."
            touch "${DEERGO_WS}/src/mppi_controller_cuda/COLCON_IGNORE"
        fi
    fi
    log "MPPI ready. At launch use mppi_backend:=auto, cpu or gpu."
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    set -Eeuo pipefail
    trap 'printf "\n[ERROR] MPPI installation failed at line %s.\n" "$LINENO" >&2' ERR
    deergo_mppi_main "$@"
fi
