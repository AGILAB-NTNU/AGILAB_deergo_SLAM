#!/bin/bash

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

source ~/ROS2_tb4/deergo_ws/install/setup.bash


# ============================================================
# Done
# ============================================================

echo "[DeerGo] Ready"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "IP:"
ip addr show enp3s0 | grep "inet "
