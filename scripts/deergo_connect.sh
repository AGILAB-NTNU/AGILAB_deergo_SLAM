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

sudo ip addr flush dev enp3s0
sudo ip addr add 192.168.158.100/24 dev enp3s0
sudo ip link set enp3s0 up


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
LIDAR_FRAME="laser_link"

# +/- 120 degrees
LIDAR_ANGLE_MIN="-2.09439510239"
LIDAR_ANGLE_MAX="2.09439510239"

echo "[DeerGo] Checking Hokuyo LiDAR..."

if [ ! -e "${LIDAR_DEVICE}" ]; then

    echo "[DeerGo] WARNING: Hokuyo LiDAR not found:"
    echo "         ${LIDAR_DEVICE}"

else

    echo "[DeerGo] Hokuyo found:"
    echo "         ${LIDAR_DEVICE}"

    # --------------------------------------------------------
    # Prevent duplicate urg_node_driver processes
    # --------------------------------------------------------

    if pgrep -f "urg_node_driver" > /dev/null; then

        echo "[DeerGo] urg_node_driver is already running."

    else

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

        sleep 2

        # ----------------------------------------------------
        # Verify LiDAR process
        # ----------------------------------------------------

        if pgrep -f "urg_node_driver" > /dev/null; then

            echo "[DeerGo] Hokuyo LiDAR started."
            echo "[DeerGo] Log: /tmp/deergo_hokuyo.log"

        else

            echo "[DeerGo] WARNING: Hokuyo LiDAR failed to start."
            echo "[DeerGo] Check log:"
            echo "         cat /tmp/deergo_hokuyo.log"

        fi

    fi

fi


# ============================================================
# Done
# ============================================================

echo
echo "============================================================"
echo "[DeerGo] Ready"
echo "============================================================"

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