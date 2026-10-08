#!/bin/bash

# ============================================================
# Resolve DeerGo repository / workspace paths
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${REPO_DIR}/src/deeergo_ws"

echo "[DeerGo] Repository: ${REPO_DIR}"
echo "[DeerGo] Workspace : ${WS_DIR}"


# ============================================================
# DeerGo network
# ============================================================

echo "[DeerGo] Configuring enp3s0..."

# Re-sourcing must not remove addresses from an interface already used by DDS.
if ! ip -4 -o addr show dev enp3s0 | awk '{print $4}' | \
    grep -Fxq '192.168.158.100/24'; then
    sudo ip addr add 192.168.158.100/24 dev enp3s0
fi
if ! ip -o link show dev enp3s0 | grep -Eq '<([^>]*,)?UP(,|>)'; then
    sudo ip link set enp3s0 up
fi


# ============================================================
# ROS2 environment
# ============================================================

echo "[DeerGo] Loading ROS2 Humble..."

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=13
export ROS_LOCALHOST_ONLY=0


# ============================================================
# DeerGo workspace
# ============================================================

if [ -f "${WS_DIR}/install/setup.bash" ]; then

    source "${WS_DIR}/install/setup.bash"

    echo "[DeerGo] Workspace environment loaded."

else

    echo "[DeerGo] WARNING: Workspace has not been built."
    echo "[DeerGo] Missing:"
    echo "         ${WS_DIR}/install/setup.bash"
    echo
    echo "[DeerGo] Build it first:"
    echo "         cd ${WS_DIR}"
    echo "         colcon build --symlink-install"

fi


# ============================================================
# Hokuyo LiDAR
# ============================================================

LIDAR_DEVICE="/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00"

# LiDAR TF frame
LIDAR_FRAME="laser"

# +/- 120 degrees
LIDAR_ANGLE_MIN="-2.09439510239"
LIDAR_ANGLE_MAX="2.09439510239"

echo "[DeerGo] Checking Hokuyo LiDAR..."

deergo_scan_is_arriving() {
    # A process or cached topic name does not prove the driver is publishing.
    timeout --signal=INT --kill-after=2s 10s \
        ros2 topic echo /scan sensor_msgs/msg/LaserScan \
        --once --field header --qos-reliability best_effort --no-daemon \
        > /tmp/deergo_scan_check.log 2>&1
}

LIDAR_READY=false

if [ ! -e "${LIDAR_DEVICE}" ]; then

    echo "[DeerGo] WARNING: Hokuyo LiDAR not found:"
    echo "         ${LIDAR_DEVICE}"

else

    echo "[DeerGo] Hokuyo found:"
    echo "         ${LIDAR_DEVICE}"

    # --------------------------------------------------------
    # Prevent duplicate urg_node_driver processes
    # --------------------------------------------------------

    LIDAR_PIDS="$(pgrep -x urg_node_driver || true)"
    LIDAR_START_ALLOWED=true

    if [ -n "${LIDAR_PIDS}" ]; then

        echo "[DeerGo] Driver exists; checking real /scan data (up to 10 seconds)..."

        if deergo_scan_is_arriving; then
            LIDAR_READY=true
            echo "[DeerGo] /scan is receiving LaserScan messages."
        else
            echo "[DeerGo] Driver is alive but /scan is unavailable; restarting this LiDAR."
            for LIDAR_PID in ${LIDAR_PIDS}; do
                # Stop only this user's driver for the configured serial device.
                if [ -O "/proc/${LIDAR_PID}" ] && \
                    tr '\0' '\n' < "/proc/${LIDAR_PID}/cmdline" | \
                    grep -Fxq "serial_port:=${LIDAR_DEVICE}"; then
                    kill -INT "${LIDAR_PID}"
                else
                    echo "[DeerGo] WARNING: Driver ${LIDAR_PID} was not started for this device; leaving it running."
                    LIDAR_START_ALLOWED=false
                fi
            done

            # Give the existing driver time to release its serial device.
            for LIDAR_WAIT in {1..50}; do
                pgrep -x urg_node_driver > /dev/null || break
                sleep 0.1
            done
            if pgrep -x urg_node_driver > /dev/null; then
                echo "[DeerGo] WARNING: Driver still exists; not starting a duplicate."
                LIDAR_START_ALLOWED=false
            fi
        fi

    fi

    if [ "${LIDAR_READY}" = false ] && [ "${LIDAR_START_ALLOWED}" = true ]; then

        echo "[DeerGo] Starting Hokuyo LiDAR..."
        echo "[DeerGo] Frame   : ${LIDAR_FRAME}"
        echo "[DeerGo] Scan FOV: -120 deg ~ +120 deg"

        nohup ros2 run urg_node urg_node_driver \
            --ros-args \
            --params-file /opt/ros/humble/share/urg_node/launch/urg_node_serial.yaml \
            -p serial_port:="${LIDAR_DEVICE}" \
            -p laser_frame_id:="${LIDAR_FRAME}" \
            -p angle_min:="${LIDAR_ANGLE_MIN}" \
            -p angle_max:="${LIDAR_ANGLE_MAX}" \
            > /tmp/deergo_hokuyo.log 2>&1 &

        # ----------------------------------------------------
        # Verify real scan data, including startup/discovery time
        # ----------------------------------------------------

        if deergo_scan_is_arriving; then

            LIDAR_READY=true
            echo "[DeerGo] Hokuyo LiDAR started; /scan is receiving data."
            echo "[DeerGo] Log: /tmp/deergo_hokuyo.log"

        else

            echo "[DeerGo] WARNING: No /scan message arrived within 10 seconds."
            echo "[DeerGo] Check log:"
            echo "         cat /tmp/deergo_hokuyo.log"
            echo "         cat /tmp/deergo_scan_check.log"

        fi

    fi

fi


# ============================================================
# Done
# ============================================================

echo
echo "============================================================"
if [ "${LIDAR_READY}" = true ]; then
    echo "[DeerGo] Ready (/scan verified)"
else
    echo "[DeerGo] WARNING: Environment loaded, but LiDAR is not ready."
fi
echo "============================================================"

if [ "${LIDAR_READY}" != true ]; then
    return 1 2>/dev/null || exit 1
fi

echo
echo "Repository:"
echo "  ${REPO_DIR}"

echo
echo "Workspace:"
echo "  ${WS_DIR}"

echo
echo "ROS2:"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"

echo
echo "Network:"
ip addr show enp3s0 | grep "inet "

echo
echo "LiDAR:"
echo "  device    = ${LIDAR_DEVICE}"
echo "  frame     = ${LIDAR_FRAME}"
echo "  angle_min = -120 deg"
echo "  angle_max = +120 deg"
echo "  raw topic = /scan"

echo
echo "LiDAR log:"
echo "  /tmp/deergo_hokuyo.log"

echo
echo "============================================================"
