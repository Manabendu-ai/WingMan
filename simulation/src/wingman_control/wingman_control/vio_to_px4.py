#!/usr/bin/env python3
"""/odom (nav_msgs/Odometry, ROS ENU/FLU) -> /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry, NED/FRD).
Odometry is NOT forwarded when: covariance is bad (> max_cov), ICP reports lost (/odom_info, if available),
the pose is not finite, or the timestamp is a duplicate/backwards. No clock comparison between domains is made;
a wall-clock watchdog only warns when /odom stops arriving. Publishes no TF."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
try:
    from rtabmap_msgs.msg import OdomInfo
except Exception:  # message package not installed: fall back to covariance gating only
    OdomInfo = None

R_NED_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
R_FLU_FRD = np.diag([1., -1., -1.])

def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])

def R_to_quat_wxyz(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q / np.linalg.norm(q)

class VioToPx4(Node):
    def __init__(self):
        super().__init__('vio_to_px4')
        self.declare_parameter('max_cov', 2.0)         # position covariance above this = tracking lost
        self.declare_parameter('jump_thresh', 0.5)     # metres between messages that counts as an odometry reset
        self.declare_parameter('watchdog_s', 1.0)      # warn if no /odom for this long (wall clock)
        self.declare_parameter('use_odom_info', True)  # also gate on /odom_info.lost when available
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos)
        self.create_subscription(Odometry, '/odom', self.cb, 10)
        if OdomInfo is not None and bool(self.get_parameter('use_odom_info').value):
            self.create_subscription(OdomInfo, '/odom_info', self.info_cb, 10)
        self.reset_counter = 1
        self.last_pos = None
        self.last_ts_us = 0
        self.last_rx = None          # wall clock (time.monotonic) of last /odom
        self.info_lost = False
        self.info_rx = 0.0           # wall clock of last /odom_info
        self.sent = 0
        self.gated = 0
        self.create_timer(1.0, self.watchdog)
        self.create_timer(5.0, self.report)
        self.get_logger().info('vio_to_px4 running: /odom -> /fmu/in/vehicle_visual_odometry '
                               f'(odom_info gate: {OdomInfo is not None})')

    def report(self):
        self.get_logger().info(f'forwarded {self.sent}, gated {self.gated} odometry messages '
                               f'(reset_counter={self.reset_counter})')

    def watchdog(self):
        if self.last_rx is None:
            self.get_logger().warn('No /odom received yet', throttle_duration_sec=5.0)
        elif time.monotonic() - self.last_rx > float(self.get_parameter('watchdog_s').value):
            self.get_logger().warn(f'No /odom for {time.monotonic() - self.last_rx:.1f} s', throttle_duration_sec=2.0)

    def info_cb(self, m):
        self.info_lost = bool(getattr(m, 'lost', False))
        self.info_rx = time.monotonic()

    def cb(self, m: Odometry):
        self.last_rx = time.monotonic()
        cov = m.pose.covariance
        p = m.pose.pose.position
        o = m.pose.pose.orientation
        if not all(math.isfinite(v) for v in [p.x, p.y, p.z, o.x, o.y, o.z, o.w]):
            return
        n = math.sqrt(o.x**2 + o.y**2 + o.z**2 + o.w**2)
        if n < 1e-3:
            return
        # Gate 1: covariance (RTAB-Map reports ~9999 when lost)
        if (not math.isfinite(cov[0])) or cov[0] <= 0.0 or cov[0] > float(self.get_parameter('max_cov').value):
            self.gated += 1
            self.get_logger().warn(f'Odometry covariance {cov[0]:.3g} too large - NOT forwarding to PX4',
                                   throttle_duration_sec=2.0)
            return
        # Gate 2: estimator says tracking is lost (only trusted if the message is recent, wall clock)
        if self.info_lost and (time.monotonic() - self.info_rx) < 1.0:
            self.gated += 1
            self.get_logger().warn('ICP reports lost - NOT forwarding to PX4', throttle_duration_sec=2.0)
            return
        # Timestamp straight from the message header (same domain as the sim clock that /odom uses)
        ts_us = int(m.header.stamp.sec * 1e6 + m.header.stamp.nanosec / 1e3)
        if ts_us < self.last_ts_us - 1_000_000:
            self.get_logger().warn('Odometry clock went backwards (sim restarted?) - resetting')
            self.last_ts_us = 0
        if ts_us <= self.last_ts_us:
            return                # duplicate or out-of-order message
        self.last_ts_us = ts_us
        if self.last_pos is not None and math.dist(self.last_pos, (p.x, p.y, p.z)) > float(self.get_parameter('jump_thresh').value):
            self.reset_counter = (self.reset_counter + 1) % 256
            self.get_logger().warn(f'Odometry jump detected -> reset_counter={self.reset_counter}')
        self.last_pos = (p.x, p.y, p.z)
        R_enu_flu = quat_to_R(o.x / n, o.y / n, o.z / n, o.w / n)
        p_ned = R_NED_ENU @ np.array([p.x, p.y, p.z])
        q = R_to_quat_wxyz(R_NED_ENU @ R_enu_flu @ R_FLU_FRD)
        v_flu = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        v_ned = R_NED_ENU @ (R_enu_flu @ v_flu)
        w_frd = R_FLU_FRD @ np.array([m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z])
        out = VehicleOdometry()
        out.timestamp = ts_us
        out.timestamp_sample = ts_us
        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position = [float(v) for v in p_ned]
        out.q = [float(v) for v in q]
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        out.velocity = [float(v) for v in v_ned]
        if hasattr(out, 'angular_velocity'):
            out.angular_velocity = [float(v) for v in w_frd]
        pv_x = max(cov[0], 1e-3)
        pv_y = max(cov[7], 1e-3)
        # High Z variance so EKF2 uses the barometer for altitude; roll/pitch weakly trusted
        out.position_variance = [float(pv_y), float(pv_x), 0.50]      # N (=odom y), E (=odom x), D
        out.orientation_variance = [0.50, 0.50, float(max(cov[35], 0.01))]
        out.velocity_variance = [0.05, 0.05, 0.20]
        out.reset_counter = int(self.reset_counter)
        out.quality = 100
        self.pub.publish(out)
        self.sent += 1

def main():
    rclpy.init()
    node = VioToPx4()
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