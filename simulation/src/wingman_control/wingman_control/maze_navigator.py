#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


class MazeNavigator(Node):

    def __init__(self):
        super().__init__("maze_navigator")

        # Navigation Parameters
        self.declare_parameter("forward_speed", 0.25)  # Target forward velocity (m/s)
        self.declare_parameter("turn_speed", 0.4)      # Target yaw rate (rad/s)
        self.declare_parameter("safe_distance", 0.6)   # Minimum obstacle distance trigger (m)
        self.declare_parameter("side_clearance", 0.4)  # Corridor alignment distance (m)

        self.v_forward = float(self.get_parameter("forward_speed").value)
        self.w_turn = float(self.get_parameter("turn_speed").value)
        self.safe_dist = float(self.get_parameter("safe_distance").value)
        self.side_dist = float(self.get_parameter("side_clearance").value)

        # Subscribers & Publishers
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Service Client to trigger automated takeoff
        self.takeoff_cli = self.create_client(Trigger, "/wingman/takeoff")

        self.active = False
        self.get_logger().info("Maze Navigator Initialized. Waiting for scan data...")

    def get_safe_min(self, ranges, center_idx, cone_width, num_points):
        """Safely extract minimum distance within a cone, handling array wrapping."""
        idx_slice = np.arange(center_idx - cone_width, center_idx + cone_width + 1)
        idx_slice = np.mod(idx_slice, num_points)  # Wrap around for 360-degree scans
        return np.min(ranges[idx_slice])

    def scan_cb(self, msg: LaserScan):
        if not self.active:
            return

        ranges = np.array(msg.ranges)
        # Clean invalid ranges (NaN, Inf, or zero/negative)
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges) | (ranges <= 0.05), 10.0, ranges)

        num_points = len(ranges)
        if num_points == 0:
            return

        # Compute indices for front (0 rad), left (+pi/2 rad), right (-pi/2 rad)
        front_idx = int(num_points * (0.0 - msg.angle_min) / (msg.angle_max - msg.angle_min)) % num_points
        left_idx = int(num_points * (math.pi / 2.0 - msg.angle_min) / (msg.angle_max - msg.angle_min)) % num_points
        right_idx = int(num_points * (-math.pi / 2.0 - msg.angle_min) / (msg.angle_max - msg.angle_min)) % num_points

        cone_width = int(num_points * (math.pi / 6.0) / (msg.angle_max - msg.angle_min))

        front_dist = self.get_safe_min(ranges, front_idx, cone_width, num_points)
        left_dist = self.get_safe_min(ranges, left_idx, cone_width, num_points)
        right_dist = self.get_safe_min(ranges, right_idx, cone_width, num_points)

        cmd = Twist()

        if front_dist < self.safe_dist:
            cmd.linear.x = 0.0
            if left_dist > right_dist:
                cmd.angular.z = self.w_turn
                self.get_logger().info("Wall ahead! Turning LEFT", throttle_duration_sec=1.0)
            else:
                cmd.angular.z = -self.w_turn
                self.get_logger().info("Wall ahead! Turning RIGHT", throttle_duration_sec=1.0)
        else:
            cmd.linear.x = self.v_forward
            side_diff = left_dist - right_dist
            cmd.linear.y = float(np.clip(0.15 * side_diff, -0.1, 0.1))
            cmd.angular.z = float(np.clip(0.2 * side_diff, -0.2, 0.2))

        self.cmd_pub.publish(cmd)

    def start_navigation(self):
        self.active = True
        self.get_logger().info("Maze Navigation Active. Streaming /cmd_vel.")


def main(args=None):
    rclpy.init(args=args)
    node = MazeNavigator()

    while not node.takeoff_cli.wait_for_service(timeout_sec=2.0):
        node.get_logger().info("Waiting for /wingman/takeoff service...")

    req = Trigger.Request()
    future = node.takeoff_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    res = future.result()
    if res and res.success:
        node.get_logger().info("Takeoff accepted. Autonomous navigation started!")
        node.start_navigation()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    else:
        node.get_logger().error(f"Takeoff failed: {res.message if res else 'No response'}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()