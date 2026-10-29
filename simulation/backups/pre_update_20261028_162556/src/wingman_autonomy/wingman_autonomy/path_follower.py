#!/usr/bin/env python3
"""Gated path follower: Path+TF+LaserScan -> /cmd_vel. Never takes off or arms."""
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from px4_msgs.msg import VehicleLocalPosition


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GatedPathFollower(Node):
    def __init__(self):
        super().__init__('wingman_vlm_path_follower')
        self.declare_parameter('allow_motion', False)
        self.declare_parameter('path_topic', '/wingman/planner_debug/path')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('max_yaw_rate', 0.15)
        self.declare_parameter('lookahead_m', 0.45)
        self.declare_parameter('goal_tolerance_m', 0.30)
        self.declare_parameter('front_stop_distance_m', 0.80)
        self.declare_parameter('scan_timeout_sec', 0.50)
        self.declare_parameter('path_timeout_sec', 3.0)
        self.declare_parameter('min_hover_height_m', 0.70)
        self.declare_parameter('max_heading_error_rad', 0.65)

        p = lambda key: self.get_parameter(key).value
        self.allow_motion = bool(p('allow_motion'))
        self.path_topic = str(p('path_topic'))
        self.scan_topic = str(p('scan_topic'))
        self.map_frame = str(p('map_frame'))
        self.base_frame = str(p('base_frame'))
        self.max_linear = max(0.03, min(0.20, float(p('max_linear_speed'))))
        self.max_yaw = max(0.05, min(0.20, float(p('max_yaw_rate'))))
        self.lookahead = max(0.20, float(p('lookahead_m')))
        self.goal_tol = max(0.15, float(p('goal_tolerance_m')))
        self.front_stop = max(0.50, float(p('front_stop_distance_m')))
        self.scan_timeout = max(0.2, float(p('scan_timeout_sec')))
        self.path_timeout = max(1.0, float(p('path_timeout_sec')))
        self.min_hover = max(0.5, float(p('min_hover_height_m')))
        self.max_heading_error = max(0.3, float(p('max_heading_error_rad')))

        self.path: Optional[Path] = None
        self.path_received = 0.0
        self.scan: Optional[LaserScan] = None
        self.scan_received = 0.0
        self.local_pos: Optional[VehicleLocalPosition] = None
        self.local_received = 0.0
        self.ground_z_ref: Optional[float] = None
        self.enabled = False
        self.last_block_reason = ''

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Path, self.path_topic, self.path_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(Trigger, '/wingman/path_follower/enable', self.enable_cb)
        self.create_service(Trigger, '/wingman/path_follower/disable', self.disable_cb)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.tick)
        self.get_logger().warning(
            f'Path follower READY BUT DISABLED; allow_motion={self.allow_motion}. '
            'It never initiates takeoff. Enable only after stable hover and current map/TF/scan checks.'
        )

    def path_cb(self, msg):
        self.path = msg
        self.path_received = time.monotonic()

    def scan_cb(self, msg):
        self.scan = msg
        self.scan_received = time.monotonic()

    def local_cb(self, msg):
        self.local_pos = msg
        self.local_received = time.monotonic()
        if self.ground_z_ref is None and getattr(msg, 'z_valid', False):
            self.ground_z_ref = float(msg.z)

    def zero(self):
        self.cmd_pub.publish(Twist())

    def front_distance(self):
        s = self.scan
        if s is None or not s.ranges:
            return None
        vals = []
        for i, value in enumerate(s.ranges):
            angle = s.angle_min + i * s.angle_increment
            if abs(wrap_angle(angle)) > math.radians(20):
                continue
            if math.isfinite(value) and s.range_min <= value <= s.range_max:
                vals.append(float(value))
        return min(vals) if vals else None

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            tr = t.transform.translation
            return float(tr.x), float(tr.y), float(tr.z), yaw_from_quat(t.transform.rotation)
        except Exception:
            return None

    def hover_height(self):
        lp = self.local_pos
        if lp is None or self.ground_z_ref is None or not getattr(lp, 'z_valid', False):
            return None
        # PX4 local frame is NED: z decreases as the vehicle climbs.
        return self.ground_z_ref - float(lp.z)

    def enable_cb(self, _req, res):
        checks = []
        now = time.monotonic()
        if not self.allow_motion:
            checks.append('allow_motion is false; restart node with -p allow_motion:=true only after hover test passes')
        if self.scan is None or now - self.scan_received > self.scan_timeout:
            checks.append('LiDAR scan missing or stale')
        if self.path is None or not self.path.poses or now - self.path_received > self.path_timeout:
            checks.append('planned path missing or stale')
        if self.local_pos is None or now - self.local_received > 0.5:
            checks.append('PX4 local-position data missing or stale')
        elif not (getattr(self.local_pos, 'xy_valid', False) and getattr(self.local_pos, 'z_valid', False)
                  and getattr(self.local_pos, 'heading_good_for_control', False)):
            checks.append('PX4 local-position/heading validity flags are not all true')
        height = self.hover_height()
        if height is None or height < self.min_hover:
            checks.append(f'estimated height above startup reference {height}; must be >= {self.min_hover:.2f} m')
        if self.pose() is None:
            checks.append(f'TF {self.map_frame}->{self.base_frame} unavailable')
        front = self.front_distance()
        if front is None:
            checks.append('no valid forward LiDAR returns')
        elif front < self.front_stop:
            checks.append(f'front obstacle at {front:.2f} m')
        if checks:
            res.success = False
            res.message = 'MOTION NOT ENABLED: ' + '; '.join(checks)
            self.get_logger().error(res.message)
            return res
        self.enabled = True
        self.last_block_reason = ''
        res.success = True
        res.message = 'Path following enabled. Takeoff was not initiated by this node.'
        self.get_logger().warning(res.message)
        return res

    def disable_cb(self, _req, res):
        self.enabled = False
        self.zero()
        res.success = True
        res.message = 'Path follower disabled and zero /cmd_vel published. This is not a landing command.'
        return res

    def tick(self):
        if not self.enabled:
            self.zero()
            return
        now = time.monotonic()
        if self.scan is None or now - self.scan_received > self.scan_timeout:
            self.zero()
            self.last_block_reason = 'stale/missing scan'
            self.get_logger().warning('Motion stopped: stale/missing scan', throttle_duration_sec=2.0)
            return
        if self.path is None or not self.path.poses or now - self.path_received > self.path_timeout:
            self.zero()
            self.last_block_reason = 'no fresh path (planner hold or stale output)'
            self.get_logger().warning('Motion stopped: no fresh planned path', throttle_duration_sec=2.0)
            return
        if self.local_pos is None or now - self.local_received > 0.5:
            self.zero()
            self.get_logger().warning('Motion stopped: PX4 local-position data stale', throttle_duration_sec=2.0)
            return
        if not (getattr(self.local_pos, 'xy_valid', False) and getattr(self.local_pos, 'z_valid', False)
                and getattr(self.local_pos, 'heading_good_for_control', False)):
            self.zero()
            self.get_logger().warning('Motion stopped: PX4 estimator validity lost', throttle_duration_sec=2.0)
            return
        height = self.hover_height()
        if height is None or height < self.min_hover:
            self.zero()
            self.enabled = False
            self.get_logger().error('MOTION DISABLED: estimated hover height fell below interlock threshold')
            return
        front = self.front_distance()
        if front is None or front < self.front_stop:
            self.zero()
            self.get_logger().warning(
                f'Motion stopped by forward obstacle interlock ({front})', throttle_duration_sec=1.0
            )
            return
        current = self.pose()
        if current is None:
            self.zero()
            self.get_logger().warning('Motion stopped: map-to-base TF unavailable', throttle_duration_sec=2.0)
            return
        x, y, _z, yaw = current
        points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in self.path.poses]
        remaining = min(math.hypot(px - x, py - y) for px, py in points) if points else float('inf')
        if remaining <= self.goal_tol and len(points) <= 2:
            self.zero()
            return
        # Choose the first path point at/after the lookahead radius, else the final pose.
        target = points[-1]
        for candidate in points:
            if math.hypot(candidate[0] - x, candidate[1] - y) >= self.lookahead:
                target = candidate
                break
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)
        if distance < 0.05:
            self.zero()
            return
        heading_error = wrap_angle(math.atan2(dy, dx) - yaw)
        cmd = Twist()
        cmd.angular.z = max(-self.max_yaw, min(self.max_yaw, 0.8 * heading_error))
        if abs(heading_error) <= self.max_heading_error and front > self.front_stop:
            cmd.linear.x = min(self.max_linear, 0.35 * distance) * max(0.0, math.cos(heading_error))
        else:
            cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GatedPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
