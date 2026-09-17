#!/usr/bin/env bash
#
# DeerGo PC development environment
# Target: Ubuntu 22.04 (Jammy) + ROS 2 Humble
#
# Installs:
#   - ROS 2 Humble Desktop / RViz2
#   - Nav2 + SLAM Toolbox
#   - TF / ROS 2 message packages
#   - ROS 2 build / dependency tools
#   - MQTT client libraries
#   - Network debugging tools
#   - Optional Python data-analysis tools
#
# DeerGo network configuration is handled separately by:
#   scripts/deergo_connect.sh
#
# Usage:
#   chmod +x scripts/install_deergo_pc_humble.sh
#   ./scripts/install_deergo_pc_humble.sh
#
# Optional configuration:
#
#   DEERGO_DOMAIN_ID=13 \
#   DEERGO_RMW=rmw_fastrtps_cpp \
#   DEERGO_WS="$HOME/deergo_ws" \
#   ./scripts/install_deergo_pc_humble.sh
#
# To avoid editing ~/.bashrc:
#
#   SKIP_BASHRC=1 \
#   ./scripts/install_deergo_pc_humble.sh
#

set -Eeuo pipefail


# ================================================================
# Configuration
# ================================================================

readonly ROS_DISTRO="humble"
readonly REQUIRED_UBUNTU_CODENAME="jammy"

DEERGO_DOMAIN_ID="${DEERGO_DOMAIN_ID:-13}"
DEERGO_RMW="${DEERGO_RMW:-rmw_fastrtps_cpp}"
DEERGO_WS="${DEERGO_WS:-$HOME/deergo_ws}"

SKIP_BASHRC="${SKIP_BASHRC:-0}"


# ================================================================
# Logging
# ================================================================

log() {
    printf '\n\033[1;34m[DEERGO ENV]\033[0m %s\n' "$*"
}

warn() {
    printf '\n\033[1;33m[WARNING]\033[0m %s\n' "$*" >&2
}

die() {
    printf '\n\033[1;31m[ERROR]\033[0m %s\n' "$*" >&2
    exit 1
}

on_error() {
    local exit_code=$?

    printf \
        '\n\033[1;31m[ERROR]\033[0m Installation failed at line %s (exit %s).\n' \
        "${BASH_LINENO[0]}" \
        "$exit_code" \
        >&2

    exit "$exit_code"
}

trap on_error ERR


# ================================================================
# Permission check
# ================================================================

if [[ "${EUID}" -eq 0 ]]; then
    die "Do not run this script with sudo. Run it as a normal user; the script invokes sudo when needed."
fi


# ================================================================
# Ubuntu check
# ================================================================

[[ -r /etc/os-release ]] \
    || die "Cannot read /etc/os-release."

# shellcheck disable=SC1091
source /etc/os-release

UBUNTU_CODENAME="${
    UBUNTU_CODENAME:-${VERSION_CODENAME:-}
}"

if [[ "${ID:-}" != "ubuntu" ]]; then
    die "This script only supports Ubuntu. Detected: ${PRETTY_NAME:-unknown}."
fi

if [[ "$UBUNTU_CODENAME" != "$REQUIRED_UBUNTU_CODENAME" ]]; then
    die "Ubuntu 22.04 (jammy) is required for ROS 2 Humble. Detected: ${PRETTY_NAME:-unknown}."
fi


# ================================================================
# RMW check
# ================================================================

case "$DEERGO_RMW" in

    rmw_fastrtps_cpp|rmw_cyclonedds_cpp)
        ;;

    *)
        die "DEERGO_RMW must be rmw_fastrtps_cpp or rmw_cyclonedds_cpp."
        ;;
esac


# ================================================================
# ROS Domain ID check
# ================================================================

if ! [[ "$DEERGO_DOMAIN_ID" =~ ^[0-9]+$ ]] \
    || (( DEERGO_DOMAIN_ID < 0 || DEERGO_DOMAIN_ID > 232 )); then

    die "DEERGO_DOMAIN_ID must be an integer from 0 to 232."
fi


# ================================================================
# Environment summary
# ================================================================

log "Ubuntu check passed: ${PRETTY_NAME}"

log "ROS distribution: ${ROS_DISTRO}"

log "ROS_DOMAIN_ID: ${DEERGO_DOMAIN_ID}"

log "RMW implementation: ${DEERGO_RMW}"

log "Workspace: ${DEERGO_WS}"


# ================================================================
# Basic system dependencies
# ================================================================

log "Installing locale and repository prerequisites..."

sudo apt-get update

sudo apt-get install -y \
    locales \
    software-properties-common \
    curl \
    ca-certificates \
    gnupg \
    lsb-release


# ================================================================
# Locale
# ================================================================

sudo locale-gen en_US en_US.UTF-8

sudo update-locale \
    LC_ALL=en_US.UTF-8 \
    LANG=en_US.UTF-8

export LANG=en_US.UTF-8

sudo add-apt-repository universe -y


# ================================================================
# ROS 2 apt repository
# ================================================================

log "Configuring the official ROS 2 apt repository..."

if dpkg-query \
    -W \
    -f='${Status}' \
    ros2-apt-source \
    2>/dev/null \
    | grep -q "install ok installed"
then

    log "ros2-apt-source is already installed."

elif grep -RqsE \
    'packages\.ros\.org/ros2/ubuntu|packages\.ros\.org/ros2-testing/ubuntu' \
    /etc/apt/sources.list \
    /etc/apt/sources.list.d \
    2>/dev/null
then

    log "An existing ROS 2 apt repository was detected; keeping it unchanged."

else

    RELEASE_JSON="$(
        mktemp \
        /tmp/ros-apt-source-release.XXXXXX.json
    )"

    ROS_APT_SOURCE_DEB="$(
        mktemp \
        /tmp/ros2-apt-source.XXXXXX.deb
    )"

    curl -fsSL \
        -o "$RELEASE_JSON" \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest

    ROS_APT_SOURCE_VERSION="$(
        awk \
            -F'"' \
            '/"tag_name"/ {print $4; exit}' \
            "$RELEASE_JSON"
    )"

    [[ -n "$ROS_APT_SOURCE_VERSION" ]] \
        || die "Could not determine the latest ros2-apt-source release."

    curl -fL \
        -o "$ROS_APT_SOURCE_DEB" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"

    sudo apt-get install -y \
        "$ROS_APT_SOURCE_DEB"

    rm -f \
        "$RELEASE_JSON" \
        "$ROS_APT_SOURCE_DEB"
fi


sudo apt-get update


# ================================================================
# ROS 2 packages
# ================================================================

log "Installing ROS 2 Humble, Nav2 and SLAM packages..."

sudo apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav2-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-std-msgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp


# ================================================================
# DeerGo MQTT / Network dependencies
# ================================================================

log "Installing DeerGo MQTT and network tools..."

sudo apt-get install -y \
    python3-paho-mqtt \
    mosquitto-clients \
    netcat-openbsd


# ================================================================
# Development tools
# ================================================================

log "Installing development tools..."

sudo apt-get install -y \
    build-essential \
    git \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions


# ================================================================
# Data analysis tools
# ================================================================

log "Installing Python data-analysis tools..."

sudo apt-get install -y \
    python3-numpy \
    python3-pandas \
    python3-matplotlib


# ================================================================
# rosdep
# ================================================================

log "Initializing rosdep..."

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then

    sudo rosdep init

else

    log "rosdep is already initialized."

fi

rosdep update


# ================================================================
# Workspace
# ================================================================

log "Preparing DeerGo ROS 2 workspace..."

mkdir -p "${DEERGO_WS}/src"


# ================================================================
# ~/.bashrc
# ================================================================

if [[ "$SKIP_BASHRC" != "1" ]]; then

    log "Writing an idempotent DeerGo environment block to ~/.bashrc..."

    BASHRC="${HOME}/.bashrc"

    START_MARKER="# >>> deergo development environment >>>"

    END_MARKER="# <<< deergo development environment <<<"

    touch "$BASHRC"

    # Remove old managed block if present.
    sed -i \
        "\|${START_MARKER}|,\|${END_MARKER}|d" \
        "$BASHRC"

    cat >> "$BASHRC" <<EOF

${START_MARKER}

source /opt/ros/${ROS_DISTRO}/setup.bash

export ROS_DOMAIN_ID=${DEERGO_DOMAIN_ID}
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=${DEERGO_RMW}

# Source the DeerGo workspace after it has been built.
if [ -f "${DEERGO_WS}/install/setup.bash" ]; then
    source "${DEERGO_WS}/install/setup.bash"
fi

${END_MARKER}
EOF

else

    warn "SKIP_BASHRC=1: ~/.bashrc was not modified."

fi


# ================================================================
# Verification
# ================================================================

log "Verifying ROS 2 installation..."

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

command -v ros2 >/dev/null

ros2 pkg prefix nav2_bringup >/dev/null

ros2 pkg prefix slam_toolbox >/dev/null

ros2 pkg prefix rviz2 >/dev/null

command -v mosquitto_pub >/dev/null

command -v mosquitto_sub >/dev/null

command -v nc >/dev/null


# ================================================================
# Finished
# ================================================================

cat <<EOF


============================================================
DeerGo PC environment installation completed.
============================================================

Open a new terminal, or run:

  source ~/.bashrc


Build the DeerGo workspace:

  cd "${DEERGO_WS}"

  rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y

  colcon build --symlink-install

  source install/setup.bash


Configure DeerGo network and ROS 2 environment:

  cd "${DEERGO_WS}"

  source scripts/deergo_connect.sh


Basic network checks:

  ip addr show enp3s0

  ping -c 4 192.168.158.200

  nc -zv 192.168.158.200 1883


ROS 2 checks:

  ros2 topic list

  ros2 topic hz /scan

  ros2 topic echo /odom --once


DeerGo mapping:

  ros2 launch deergo_mqtt_bridge slam_rviz.launch.py \\
    load_map:=false


DeerGo localization:

  ros2 launch deergo_mqtt_bridge slam_rviz.launch.py \\
    load_map:=true


Important:

  ROS_DOMAIN_ID=${DEERGO_DOMAIN_ID}

  RMW_IMPLEMENTATION=${DEERGO_RMW}

  DeerGo Ethernet/network configuration is handled by:

    scripts/deergo_connect.sh

  Serialized SLAM maps are expected under:

    maps/deergo.data
    maps/deergo.posegraph

============================================================

EOF
