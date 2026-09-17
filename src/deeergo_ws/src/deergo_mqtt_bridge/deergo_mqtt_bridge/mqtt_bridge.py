#!/usr/bin/env python3

import json
import math

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

BROKER_IP = "192.168.158.200"
BROKER_PORT = 1883

TOPIC_PREFIX = "Puffer/DeerGo_001"

MQTT_SUB_TOPIC = f"{TOPIC_PREFIX}/#"
ODOM_MQTT_TOPIC = f"{TOPIC_PREFIX}/odom"
VEL_MQTT_TOPIC = f"{TOPIC_PREFIX}/vel"


class DeerGoMqttBridge(Node):
    def __init__(self):
        super().__init__("deergo_mqtt_bridge")

        # ============================================================
        # ROS2 Publisher
        # ============================================================

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10,
        )

        # ============================================================
        # ROS2 Subscriber
        #
        # Nav2 /cmd_vel -> MQTT vel
        # ============================================================

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        # ============================================================
        # TF
        # ============================================================

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_count = 0
        self.cmd_vel_count = 0

        # ============================================================
        # MQTT
        # ============================================================

        self.mqtt_client = mqtt.Client()

        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
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

        self.get_logger().info("ROS2 /cmd_vel subscriber ready")

    # ================================================================
    # MQTT callbacks
    # ================================================================

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT connected")

            result, _ = client.subscribe(
                MQTT_SUB_TOPIC,
                qos=0,
            )

            if result == mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().info(f"MQTT subscribe: {MQTT_SUB_TOPIC}")

            else:
                self.get_logger().error(f"MQTT subscribe failed: result={result}")

        else:
            self.get_logger().error(f"MQTT connection failed: rc={rc}")

    def on_disconnect(self, client, userdata, rc):
        if rc == 0:
            self.get_logger().info("MQTT disconnected normally")

        else:
            self.get_logger().warning(f"MQTT disconnected unexpectedly: rc={rc}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic

        try:
            payload = msg.payload.decode("utf-8")

        except UnicodeDecodeError:
            self.get_logger().warning(f"[MQTT] Cannot decode payload: {topic}")
            return

        # ------------------------------------------------------------
        # ODOM
        # ------------------------------------------------------------

        if topic == ODOM_MQTT_TOPIC:
            self.handle_odom(payload)
            return

        self.get_logger().debug(f"[MQTT] {topic}: {payload}")

    # ================================================================
    # ROS2 /cmd_vel -> MQTT vel
    # ================================================================

    def cmd_vel_callback(self, msg):
        # ROS2:
        # linear.x  > 0 -> forward
        # angular.z > 0 -> CCW
        #
        # DeerGo:
        # turn_degs > 0 -> CW
        #
        # 因此 angular velocity 必須反號

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
                f"/cmd_vel -> MQTT | "
                f"vel={vel_ms:.3f} m/s | "
                f"turn={turn_degs:.1f} deg/s"
            )

    # ================================================================
    # ODOM MQTT -> ROS2
    # ================================================================

    def handle_odom(self, payload):
        try:
            data = json.loads(payload)

            x = float(data["x_m"])
            y = float(data["y_m"])

            # 實測：
            # CCW -> yaw_deg 增加
            # 符合 ROS yaw 正方向，因此不反號
            yaw_deg = float(data["yaw_deg"])

        except json.JSONDecodeError as e:
            self.get_logger().error(f"ODOM JSON decode error: {e}")
            return

        except KeyError as e:
            self.get_logger().error(f"ODOM missing field: {e}")
            return

        except ValueError as e:
            self.get_logger().error(f"ODOM value error: {e}")
            return

        # ------------------------------------------------------------
        # yaw degree -> quaternion
        # ------------------------------------------------------------

        yaw_rad = math.radians(yaw_deg)

        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        # DeerGo MQTT odom 沒有 timestamp
        # 因此目前使用 PC 收到 MQTT 時的 ROS time
        now = self.get_clock().now().to_msg()

        # ============================================================
        # Publish /odom
        # ============================================================

        odom_msg = Odometry()

        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"

        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom_msg)

        # ============================================================
        # TF: odom -> base_link
        # ============================================================

        tf_msg = TransformStamped()

        tf_msg.header.stamp = now
        tf_msg.header.frame_id = "odom"

        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)

        # ============================================================
        # Debug
        # ============================================================

        self.odom_count += 1

        if self.odom_count % 10 == 0:
            self.get_logger().info(
                f"ROS2 /odom published | "
                f"x={x:.3f}, "
                f"y={y:.3f}, "
                f"yaw={yaw_deg:.1f} deg"
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
