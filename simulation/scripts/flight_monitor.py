#!/usr/bin/env python3
"""Run DURING a flight. Logs localization health twice a second to the terminal and ~/ws_wingman/flight_log.txt:
  ODOM  position, covariance, speed          ICP  lost flag + every icp_* field the installed OdomInfo has
  PX4   validity flags, z, speed, data age   CMD vs MEAS body velocity (FLU) and DIR_MISMATCH if they disagree
  scan_min  nearest valid lidar return
  python3 ~/ws_wingman/scripts/flight_monitor.py --ros-args -p use_sim_time:=true"""
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import VehicleLocalPosition

try:
    from rtabmap_msgs.msg import OdomInfo
except Exception:
    OdomInfo = None


def fmt(v):
    if isinstance(v, float):
        return f'{v:.3g}'
    return str(v)


class Monitor(Node):
    def __init__(self):
        super().__init__('flight_monitor')
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.odom = self.lp = self.info = None
        self.cmd = Twist()
        self.odom_t = self.lp_t = 0.0
        self.cmd_t = -1e9
        self.scan_min = None
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_lp, q)
        self.create_subscription(LaserScan, '/scan', self.on_scan, q)
        if OdomInfo is not None:
            self.create_subscription(OdomInfo, '/odom_info', self.on_info, 10)
        self.path = os.path.expanduser('~/ws_wingman/flight_log.txt')
        self.f = open(self.path, 'a')
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f'flight_monitor running -> {self.path} (OdomInfo available: {OdomInfo is not None})')

    def destroy_node(self):
        try:
            self.f.close()
        except Exception:
            pass
        return super().destroy_node()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        self.odom = m; self.odom_t = self.now()

    def on_cmd(self, m):
        self.cmd = m; self.cmd_t = self.now()

    def on_lp(self, m):
        self.lp = m; self.lp_t = self.now()

    def on_info(self, m):
        self.info = m

    def on_scan(self, m):
        r = [x for x in m.ranges if math.isfinite(x) and x > 0.05]
        self.scan_min = min(r) if r else None

    def info_text(self):
        # Introspect the installed message instead of guessing field names
        fields = self.info.get_fields_and_field_types()
        keys = [k for k in fields if k == 'lost' or k.startswith('icp_') or k in ('matches', 'inliers', 'features')]
        return 'ICP ' + ' '.join(f'{k}={fmt(getattr(self.info, k))}' for k in keys)

    def tick(self):
        t = self.now()
        p = []
        mx = my = mw = 0.0
        if self.odom is not None:
            o = self.odom
            mx, my, mw = o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z
            p.append(f'ODOM x={o.pose.pose.position.x:.2f} y={o.pose.pose.position.y:.2f} '
                     f'cov={o.pose.covariance[0]:.1e} age={t - self.odom_t:.2f}s speed={math.hypot(mx, my):.2f}')
        else:
            p.append('ODOM none')
        if self.info is not None:
            p.append(self.info_text())
        if self.lp is not None:
            l = self.lp
            p.append(f'PX4 xy={int(l.xy_valid)} vxy={int(l.v_xy_valid)} z_ok={int(l.z_valid)} vz_ok={int(l.v_z_valid)} '
                     f'head_ok={int(l.heading_good_for_control)} z={l.z:.2f} heading={l.heading:.2f} '
                     f'speed={math.hypot(l.vx, l.vy):.2f} age={t - self.lp_t:.2f}s')
        else:
            p.append('PX4 none')

        if (t - self.cmd_t) < 0.5 and self.odom is not None:
            c = self.cmd
            cx, cy, cw = c.linear.x, c.linear.y, c.angular.z
            bad = []
            if abs(cx) > 0.05 and abs(mx) > 0.03 and cx * mx < 0:
                bad.append('X')
            if abs(cy) > 0.05 and abs(my) > 0.03 and cy * my < 0:
                bad.append('Y')
            if abs(cw) > 0.1 and abs(mw) > 0.05 and cw * mw < 0:
                bad.append('YAW')
            p.append(f'CMD=({cx:.2f},{cy:.2f},{cw:.2f}) MEAS=({mx:.2f},{my:.2f},{mw:.2f}) '
                     f'DIR_MISMATCH={",".join(bad) or "none"}')
        p.append(f'scan_min={None if self.scan_min is None else round(self.scan_min, 2)}')
        line = f'{t:9.1f} | ' + ' | '.join(p)
        self.get_logger().info(line)
        self.f.write(line + '\n')
        self.f.flush()


def main():
    rclpy.init()
    node = Monitor()
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
