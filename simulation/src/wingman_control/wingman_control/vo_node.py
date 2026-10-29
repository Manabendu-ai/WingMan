#!/usr/bin/env python3

# ==============================================================
# vo_node.py — Depth-based Visual Odometry modified for PX4 SITL (ROS 2 Humble)
# ==============================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, Image, CameraInfo
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
    PoseStamped,
    TransformStamped,
)
from nav_msgs.msg import Path, Odometry
from tf2_ros import TransformBroadcaster

# PX4 uORB message imports
from px4_msgs.msg import VehicleVisualOdometry, VehicleAttitude

import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import deque

try:
    import pyrealsense2 as rs

    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False


# ==============================================================
# Quaternion Helpers
# ==============================================================

def rot_to_quat(R):
    """Convert 3x3 rotation matrix -> quaternion (x, y, z, w)."""
    q = np.empty((4,))
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2, 1] - R[1, 2]) * s
        q[1] = (R[0, 2] - R[2, 0]) * s
        q[2] = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q[3] = (R[2, 1] - R[1, 2]) / s
        q[0] = 0.25 * s
        q[1] = (R[0, 1] + R[1, 0]) / s
        q[2] = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q[3] = (R[0, 2] - R[2, 0]) / s
        q[0] = (R[0, 1] + R[1, 0]) / s
        q[1] = 0.25 * s
        q[2] = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q[3] = (R[1, 0] - R[0, 1]) / s
        q[0] = (R[0, 2] + R[2, 0]) / s
        q[1] = (R[1, 2] + R[2, 1]) / s
        q[2] = 0.25 * s
    return q[0], q[1], q[2], q[3]


def quat_to_rot(qx, qy, qz, qw):
    """Convert quaternion -> 3x3 rotation matrix (body -> world)."""
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


# ==============================================================
# IMU Buffer + Stationarity Detection
# ==============================================================

class IMUBuffer:
    def __init__(self):
        self.buffer = deque(maxlen=500)
        self.calibrated = False

    def push(self, msg: Imu):
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        self.buffer.append((accel, gyro))

    def calibrate(self) -> bool:
        if len(self.buffer) < 50:
            return False
        self.calibrated = True
        return True

    def is_stationary(self, window: int = 10) -> bool:
        if len(self.buffer) < window:
            return False

        recent = list(self.buffer)[-window:]
        gyro_norms = [np.linalg.norm(g) for _, g in recent]
        accel_vals = np.array([a for a, _ in recent])

        return (
                max(gyro_norms) < 0.05 and
                np.std(accel_vals, axis=0).mean() < 0.25
        )


# ==============================================================
# MAIN NODE
# ==============================================================

class VONode(Node):
    MAX_SPEED_MS = 0.5  # m/s
    DT_PNP = 0.133  # s
    MAX_DELTA = MAX_SPEED_MS * DT_PNP
    MAX_DELTA_Z = 0.02  # m
    DEPTH_AGE_LIMIT = 0.20  # s
    IMU_STABLE_REQ = 30  # frames
    ORIGIN_TIMEOUT_S = 10.0  # s
    VEL_ALPHA = 0.4
    PATH_MAX_LEN = 5000

    def __init__(self):
        super().__init__('vo_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # ROS 2 & RTAB-Map Publishers
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/vo/pose', 10)
        self.pub_odom = self.create_publisher(Odometry, '/vo/odom', 10)
        self.pub_path = self.create_publisher(Path, '/vo/path', 10)

        # PX4 uORB Visual Odometry Publisher
        self.pub_px4_visual_odom = self.create_publisher(
            VehicleVisualOdometry, '/fmu/in/vehicle_visual_odometry', 10)

        self.bridge = CvBridge()
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        self.imu_buffer = IMUBuffer()

        # Subscribe to PX4 IMU and Fused Vehicle Attitude
        self.create_subscription(Imu, '/fmu/out/vehicle_imu', self.imu_callback, qos)
        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos
        )

        self.pos_world = np.zeros(3)
        self.origin_set = False
        self.origin_pos = np.zeros(3)
        self.imu_stable_count = 0
        self.startup_time = self.get_clock().now()

        self.R_cam_body = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0],
        ], dtype=np.float64)

        self.imu_qx = 0.0
        self.imu_qy = 0.0
        self.imu_qz = 0.0
        self.imu_qw = 1.0

        self.prev_vel_time = None
        self.prev_vel_pos = np.zeros(3)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

        self.frame_count = 0
        self.prev_time = cv2.getTickCount()

        # 15Hz timer loop
        self.timer = self.create_timer(0.065, self.process_frame)
        self.get_logger().info('PX4 Visual Odometry Node (vo_node) initialized.')

    def imu_callback(self, msg: Imu):
        self.imu_buffer.push(msg)

    def attitude_callback(self, msg: VehicleAttitude):
        """Receives PX4 fused attitude orientation quaternions (w, x, y, z)."""
        self.imu_qw = float(msg.q[0])
        self.imu_qx = float(msg.q[1])
        self.imu_qy = float(msg.q[2])
        self.imu_qz = float(msg.q[3])

    def process_frame(self):
        if not self.imu_buffer.calibrated:
            if self.imu_buffer.calibrate():
                self.get_logger().info('IMU calibrated — VO active.')
            return

        self.publish_pose()

    def publish_pose(self):
        stamp = self.get_clock().now().to_msg()
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        if not self.origin_set:
            elapsed = (self.get_clock().now().nanoseconds - self.startup_time.nanoseconds) / 1e9
            if self.imu_buffer.is_stationary():
                self.imu_stable_count += 1
            else:
                self.imu_stable_count = 0

            if self.imu_stable_count >= self.IMU_STABLE_REQ or elapsed > self.ORIGIN_TIMEOUT_S:
                self.origin_pos = self.pos_world.copy()
                self.origin_set = True
                self.get_logger().info('Origin set for PX4 VIO.')
            return

        relative_pos = self.pos_world - self.origin_pos

        # 1. Publish PX4 uORB VehicleVisualOdometry message directly
        px4_msg = VehicleVisualOdometry()
        px4_msg.timestamp = now_us
        px4_msg.timestamp_sample = now_us
        px4_msg.pose_frame = VehicleVisualOdometry.POSE_FRAME_NED

        # ROS ENU -> PX4 NED Frame Conversion
        px4_msg.position = [
            float(relative_pos[0]),  # North = ROS X
            float(-relative_pos[1]),  # East  = -ROS Y
            float(-relative_pos[2])  # Down  = -ROS Z
        ]
        px4_msg.q = [
            float(self.imu_qw),
            float(self.imu_qx),
            float(-self.imu_qy),
            float(-self.imu_qz)
        ]

        px4_msg.velocity_frame = VehicleVisualOdometry.VELOCITY_FRAME_NED
        px4_msg.velocity = [
            float(self.vx),
            float(-self.vy),
            float(-self.vz)
        ]

        # Publish to PX4 DDS agent topic
        self.pub_px4_visual_odom.publish(px4_msg)

        # 2. Publish Standard ROS 2 /vo/odom for RTAB-Map
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = float(relative_pos[0])
        odom_msg.pose.pose.position.y = float(relative_pos[1])
        odom_msg.pose.pose.position.z = float(relative_pos[2])
        odom_msg.pose.pose.orientation.x = self.imu_qx
        odom_msg.pose.pose.orientation.y = self.imu_qy
        odom_msg.pose.pose.orientation.z = self.imu_qz
        odom_msg.pose.pose.orientation.w = self.imu_qw
        self.pub_odom.publish(odom_msg)

        # 3. Broadcast TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = float(relative_pos[0])
        tf_msg.transform.translation.y = float(relative_pos[1])
        tf_msg.transform.translation.z = float(relative_pos[2])
        tf_msg.transform.rotation.x = self.imu_qx
        tf_msg.transform.rotation.y = self.imu_qy
        tf_msg.transform.rotation.z = self.imu_qz
        tf_msg.transform.rotation.w = self.imu_qw
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()