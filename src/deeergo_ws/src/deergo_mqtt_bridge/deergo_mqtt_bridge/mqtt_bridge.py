#!/usr/bin/env python3

import json
import math

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

BROKER_IP = "192.168.158.200"
BROKER_PORT = 1883

TOPIC_PREFIX = "Puffer/DeerGo_001"
VEL_MQTT_TOPIC = f"{TOPIC_PREFIX}/vel"


class DeerGoMqttBridge(Node):
    def __init__(self):
        super().__init__("deergo_mqtt_bridge")

        # ============================================================
        # ROS2 Subscriber
        #
        # Nav2 /cmd_vel -> MQTT velocity command
        # ============================================================

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.cmd_vel_count = 0

        # ============================================================
        # MQTT
        # ============================================================

        self.mqtt_client = mqtt.Client()

        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect

        self.get_logger().info(f"Connecting MQTT broker: {BROKER_IP}:{BROKER_PORT}")

        try:
            self.mqtt_client.connect(
                BROKER_IP,
                BROKER_PORT,
                keepalive=60,
            )

        except Exception as e:
            self.get_logger().error(f"MQTT connect failed: {e}")
            raise

        self.mqtt_client.loop_start()

        self.get_logger().info("ROS2 /cmd_vel -> MQTT velocity bridge ready")

        self.get_logger().info("MQTT odometry input: DISABLED")

        self.get_logger().info("ROS2 /odom publisher: DISABLED")

        self.get_logger().info("odom -> base_link TF publisher: DISABLED")

    # ================================================================
    # MQTT callbacks
    # ================================================================

    def on_connect(
        self,
        client,
        userdata,
        flags,
        rc,
    ):
        if rc == 0:
            self.get_logger().info("MQTT connected")

        else:
            self.get_logger().error(f"MQTT connection failed: rc={rc}")

    def on_disconnect(
        self,
        client,
        userdata,
        rc,
    ):
        if rc == 0:
            self.get_logger().info("MQTT disconnected normally")

        else:
            self.get_logger().warning(f"MQTT disconnected unexpectedly: rc={rc}")

    # ================================================================
    # ROS2 /cmd_vel -> MQTT vel
    # ================================================================

    def cmd_vel_callback(
        self,
        msg,
    ):
        # ROS2:
        #   linear.x  > 0 -> forward
        #   angular.z > 0 -> CCW
        #
        # DeerGo:
        #   turn_degs > 0 -> CW
        #
        # Therefore angular velocity is inverted.

        vel_ms = msg.linear.x

        turn_degs = -math.degrees(msg.angular.z)

        payload = {
            "vel_ms": f"{vel_ms:.3f}",
            "turn_degs": f"{turn_degs:.3f}",
        }

        payload_json = json.dumps(
            payload,
            separators=(",", ":"),
        )

        result = self.mqtt_client.publish(
            VEL_MQTT_TOPIC,
            payload_json,
            qos=0,
        )

        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(f"MQTT vel publish failed: rc={result.rc}")
            return

        self.cmd_vel_count += 1

        if self.cmd_vel_count % 10 == 0:
            self.get_logger().info(
                f"/cmd_vel -> MQTT | vel={vel_ms:.3f} m/s | turn={turn_degs:.1f} deg/s"
            )

    # ================================================================
    # Shutdown
    # ================================================================

    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = DeerGoMqttBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
