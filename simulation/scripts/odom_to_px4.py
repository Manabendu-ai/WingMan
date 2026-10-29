#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class OdomToPX4(Node):
    def __init__(self):
        super().__init__('odom_to_px4')

        self.publisher = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.reset_counter = 0
        self.get_logger().info(
            'RTAB-Map odometry to PX4 converter started'
        )

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q_ros = msg.pose.pose.orientation

        # ROS ENU position -> PX4 NED position.
        position_ned = [p.y, p.x, -p.z]

        # Convert ROS ENU/FLU orientation to PX4 NED/FRD.
        # This assumes RTAB-Map world is ENU and child frame is FLU.
        yaw = quaternion_to_yaw(q_ros)

        # ENU yaw -> NED yaw, with FRD body convention.
        yaw_ned = math.pi / 2.0 - yaw

        half_yaw = yaw_ned / 2.0
        q_ned_frd = [
            math.cos(half_yaw),
            0.0,
            0.0,
            math.sin(half_yaw)
        ]

        out = VehicleOdometry()
        out.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        out.timestamp_sample = out.timestamp

        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position = [float(v) for v in position_ned]
        out.q = q_ned_frd

        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        out.velocity = [float('nan')] * 3
        out.angular_velocity = [float('nan')] * 3

        out.position_variance = [0.05, 0.05, 0.10]
        out.orientation_variance = [0.05, 0.05, 0.10]
        out.velocity_variance = [float('nan')] * 3

        out.reset_counter = self.reset_counter
        out.quality = 50

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPX4()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
