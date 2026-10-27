"""ROS 2 Humble bridge for confirmed WingMan perception and telemetry inputs."""

from __future__ import annotations

import json
import time
from typing import Any

from .contracts import StateStore, vector3

try:
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rtabmap_msgs.msg import RGBDImage
    from std_msgs.msg import String
except ImportError as exc:  # allows pure-Python contract tests without ROS
    rclpy = None
    _ROS_IMPORT_ERROR = exc

if rclpy is not None:
    try:
        from px4_msgs.msg import BatteryStatus, VehicleLocalPosition, VehicleStatus
    except ImportError:
        BatteryStatus = VehicleLocalPosition = VehicleStatus = None
    try:
        from px4_msgs.msg import VehicleLandDetected
    except ImportError:
        VehicleLandDetected = None


def _stamp(msg: Any) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return time.time()
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


if rclpy is not None:
    class WingmanBridge(Node):
        def __init__(self):
            super().__init__("wingman_bridge")
            self.store = StateStore(
                stale_after_s=float(self.declare_parameter(
                    "stale_after_s", 2.0).value))
            self.map_topic = self.declare_parameter(
                "map_topic", "/map").value
            self.odom_topic = self.declare_parameter(
                "odom_topic", "/odom").value
            self.rgbd_topic = self.declare_parameter(
                "rgbd_topic", "/odom_rgbd_image").value
            self.local_position_topic = self.declare_parameter(
                "local_position_topic",
                "/fmu/out/vehicle_local_position_v1").value
            self.vehicle_status_topic = self.declare_parameter(
                "vehicle_status_topic",
                "/fmu/out/vehicle_status_v1").value
            self.battery_topic = self.declare_parameter(
                "battery_topic", "/fmu/out/battery_status_v1").value
            self.landing_topic = self.declare_parameter(
                "landing_topic",
                "/fmu/out/vehicle_land_detected").value
            self.publishers = {
                name: self.create_publisher(String, topic, 10)
                for name, topic in {
                    "dashboard": "/wingman/dashboard_state",
                    "map": "/wingman/map_json",
                    "telemetry": "/wingman/telemetry_json",
                    "rgbd": "/wingman/rgbd_info",
                    "health": "/wingman/health",
                }.items()
            }
            self.create_subscription(
                OccupancyGrid, self.map_topic, self._map_cb, qos_profile_sensor_data)
            self.create_subscription(
                Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data)
            self.create_subscription(
                RGBDImage, self.rgbd_topic, self._rgbd_cb, qos_profile_sensor_data)
            if VehicleLocalPosition is not None:
                self.create_subscription(
                    VehicleLocalPosition, self.local_position_topic,
                    self._local_position_cb, qos_profile_sensor_data)
            if VehicleStatus is not None:
                self.create_subscription(
                    VehicleStatus, self.vehicle_status_topic,
                    self._vehicle_status_cb, qos_profile_sensor_data)
            if BatteryStatus is not None:
                self.create_subscription(
                    BatteryStatus, self.battery_topic,
                    self._battery_cb, qos_profile_sensor_data)
            if VehicleLandDetected is not None:
                self.create_subscription(
                    VehicleLandDetected, self.landing_topic,
                    self._landing_cb, qos_profile_sensor_data)
            self.create_timer(0.5, self._publish_state)
            self.get_logger().warn(
                "Flight command publication is disabled: EGO-Planner and PX4 "
                "command interfaces are not finalized.")

        def _map_cb(self, msg):
            info = msg.info
            value = {
                "schema": "wingman.map.v1",
                "frame_id": msg.header.frame_id,
                "width": int(info.width),
                "height": int(info.height),
                "resolution": float(info.resolution),
                "origin": {
                    "x": float(info.origin.position.x),
                    "y": float(info.origin.position.y),
                },
                "data": [int(v) for v in msg.data],
            }
            self.store.update("map", value, _stamp(msg))

        def _odom_cb(self, msg):
            pose = msg.pose.pose
            value = {
                "schema": "wingman.telemetry.v1",
                "frame_id": msg.header.frame_id,
                "position": vector3(pose.position),
                "orientation": {
                    "x": float(pose.orientation.x),
                    "y": float(pose.orientation.y),
                    "z": float(pose.orientation.z),
                    "w": float(pose.orientation.w),
                },
                "linear_velocity": vector3(msg.twist.twist.linear),
                "angular_velocity": vector3(msg.twist.twist.angular),
                "source": "odometry",
            }
            self.store.update("telemetry", value, _stamp(msg))

        def _rgbd_cb(self, msg):
            rgb = msg.rgb_image
            depth = msg.depth_image
            value = {
                "schema": "wingman.rgbd.v1",
                "rgb": {
                    "frame_id": rgb.header.frame_id,
                    "encoding": rgb.encoding,
                    "width": int(rgb.width),
                    "height": int(rgb.height),
                },
                "depth": {
                    "frame_id": depth.header.frame_id,
                    "encoding": depth.encoding,
                    "width": int(depth.width),
                    "height": int(depth.height),
                },
                "camera_info_available": bool(
                    getattr(msg, "rgb_camera_info", None) is not None),
                "source": "rtabmap_rgbd_image",
            }
            self.store.update("rgbd", value, _stamp(msg))

        def _merge_telemetry(self, values: dict):
            current = dict(self.store.telemetry)
            current.update(values)
            self.store.update("telemetry", current)

        def _local_position_cb(self, msg):
            self._merge_telemetry({
                "px4_position": {
                    "x": getattr(msg, "x", None),
                    "y": getattr(msg, "y", None),
                    "z": getattr(msg, "z", None),
                },
                "px4_velocity": {
                    "x": getattr(msg, "vx", None),
                    "y": getattr(msg, "vy", None),
                    "z": getattr(msg, "vz", None),
                },
                "heading_rad": getattr(msg, "heading", None),
                "localization": {
                    "xy_valid": bool(getattr(msg, "xy_valid", False)),
                    "z_valid": bool(getattr(msg, "z_valid", False)),
                    "heading_valid": bool(
                        getattr(msg, "heading_good_for_control", False)),
                },
                "source": "px4_local_position",
            })

        def _vehicle_status_cb(self, msg):
            self._merge_telemetry({
                "armed": getattr(msg, "arming_state", None) == 2,
                "arming_state": getattr(msg, "arming_state", None),
                "flight_mode": getattr(msg, "nav_state", None),
                "source": "px4_vehicle_status",
            })

        def _battery_cb(self, msg):
            self._merge_telemetry({
                "battery": {
                    "remaining": getattr(msg, "remaining", None),
                    "voltage_v": getattr(msg, "voltage_v", None),
                },
                "source": "px4_battery",
            })

        def _landing_cb(self, msg):
            self._merge_telemetry({
                "landed": bool(getattr(msg, "landed", False)),
                "maybe_landed": bool(getattr(msg, "maybe_landed", False)),
                "source": "px4_landing",
            })

        def _publish(self, key: str, value: dict):
            message = String()
            message.data = json.dumps(value, separators=(",", ":"))
            self.publishers[key].publish(message)

        def _publish_state(self):
            now = time.time()
            self._publish("map", self.store.map_data or {
                "schema": "wingman.map.v1", "available": False})
            self._publish("telemetry", self.store.telemetry or {
                "schema": "wingman.telemetry.v1", "available": False})
            self._publish("rgbd", self.store.rgbd or {
                "schema": "wingman.rgbd.v1", "available": False})
            self._publish("health", self.store.health(now))
            self._publish("dashboard", self.store.dashboard_state(now))


def main(args=None):
    if rclpy is None:
        raise RuntimeError(
            "ROS 2 dependencies are unavailable; source ROS 2 Humble before "
            "running wingman_ros") from _ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = WingmanBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
