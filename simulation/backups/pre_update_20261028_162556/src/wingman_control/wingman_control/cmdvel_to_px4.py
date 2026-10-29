#!/usr/bin/env python3
"""Guarded Nav2 velocity bridge for PX4 Offboard control.

ROS /cmd_vel is body-frame FLU; PX4 TrajectorySetpoint velocity is local NED.
The node intentionally publishes nothing in idle. Takeoff is refused unless
both odometry tracking and PX4 local state are fresh and valid.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import OdomInfo
from std_srvs.srv import Trigger
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition

NAN = float('nan')


class CmdVelToPx4(Node):
    def __init__(self):
        super().__init__('cmdvel_to_px4')
        self.declare_parameter('takeoff_alt', 1.0)
        self.declare_parameter('climb_speed', 0.35)
        self.declare_parameter('odom_timeout_sec', 0.25)
        self.declare_parameter('odom_info_timeout_sec', 0.5)
        self.declare_parameter('min_icp_correspondences', 25)
        self.declare_parameter('min_icp_inlier_ratio', 0.20)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('climb_timeout_sec', 15.0)
        self.alt = float(self.get_parameter('takeoff_alt').value)
        self.climb = float(self.get_parameter('climb_speed').value)

        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.p_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.p_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.p_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_lpos, px4_qos)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(OdomInfo, '/odom_info', self.on_odom_info, 10)
        self.create_service(Trigger, '/wingman/takeoff', self.srv_takeoff)
        self.create_service(Trigger, '/wingman/land', self.srv_land)

        self.state = 'idle'  # idle -> arming -> climbing -> flying; fault -> land manually
        self.state_t = self.now_s()
        self.tick = 0
        self.cmd = Twist()
        self.cmd_t = -1e9
        self.odom_t = -1e9
        self.odom_info = None
        self.odom_info_t = -1e9
        self.yaw_enu = 0.0
        self.lpos = None
        self.lpos_t = -1e9
        self.create_timer(0.05, self.loop)  # 20 Hz
        self.get_logger().info('Guarded cmdvel_to_px4 ready; idle until /wingman/takeoff is accepted')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def set_state(self, state):
        self.state = state
        self.state_t = self.now_s()
        self.get_logger().info(f'state -> {state}')

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_t = self.now_s()

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        vals = [q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in vals) or sum(v*v for v in vals) < 1e-8:
            return
        self.yaw_enu = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        self.odom_t = self.now_s()

    def on_odom_info(self, msg):
        self.odom_info = msg
        self.odom_info_t = self.now_s()

    def on_lpos(self, msg):
        self.lpos = msg
        self.lpos_t = self.now_s()

    def odom_healthy(self):
        if self.now_s() - self.odom_t > float(self.get_parameter('odom_timeout_sec').value):
            return False, 'no fresh /odom'
        if self.odom_info is None or self.now_s() - self.odom_info_t > float(self.get_parameter('odom_info_timeout_sec').value):
            return False, 'no fresh /odom_info'
        info = self.odom_info
        if info.lost:
            return False, 'ICP reports lost tracking'
        if info.icp_correspondences < int(self.get_parameter('min_icp_correspondences').value):
            return False, f'too few ICP correspondences ({info.icp_correspondences})'
        if info.icp_inliers_ratio < float(self.get_parameter('min_icp_inlier_ratio').value):
            return False, f'ICP inlier ratio too low ({info.icp_inliers_ratio:.2f})'
        return True, 'ok'

    def px4_state_healthy(self):
        if self.lpos is None or self.now_s() - self.lpos_t > 0.5:
            return False, 'no fresh PX4 local-position message'
        p = self.lpos
        if not (p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid):
            return False, f'PX4 estimate invalid (xy={p.xy_valid}, z={p.z_valid}, vxy={p.v_xy_valid}, vz={p.v_z_valid})'
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, p.vx, p.vy, p.vz)):
            return False, 'PX4 local-position values are non-finite'
        return True, 'ok'

    def srv_takeoff(self, request, response):
        odom_ok, odom_reason = self.odom_healthy()
        if not odom_ok:
            response.success = False
            response.message = f'Refusing takeoff: {odom_reason}'
            self.get_logger().error(response.message)
            return response
        px4_ok, px4_reason = self.px4_state_healthy()
        if not px4_ok:
            response.success = False
            response.message = f'Refusing takeoff: {px4_reason}'
            self.get_logger().error(response.message)
            return response
        if self.state not in ('idle', 'fault'):
            response.success = False
            response.message = f'Cannot take off while state={self.state}'
            return response
        self.tick = 0
        self.set_state('arming')
        response.success = True
        response.message = 'preflight gates passed; Offboard prestream started'
        return response

    def srv_land(self, request, response):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.set_state('landing')
        response.success = True
        response.message = 'land command sent; confirm landing in PX4/QGroundControl'
        return response

    def send_cmd(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.us()
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.p_cmd.publish(msg)

    def publish_velocity_setpoint(self, vn, ve, vd, yawspeed):
        # A single, consistent control mode: velocity on all 3 axes.
        mode = OffboardControlMode()
        mode.timestamp = self.us()
        mode.position = False
        mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.p_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.timestamp = self.us()
        sp.position = [NAN, NAN, NAN]
        sp.velocity = [float(vn), float(ve), float(vd)]
        sp.acceleration = [NAN, NAN, NAN]
        sp.yaw = NAN
        sp.yawspeed = float(yawspeed)
        self.p_sp.publish(sp)

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def loop(self):
        if self.state in ('idle', 'landing'):
            return  # no Offboard stream while deliberately idle
        now = self.now_s()
        self.tick += 1
        odom_ok, odom_reason = self.odom_healthy()
        px4_ok, px4_reason = self.px4_state_healthy()
        if not odom_ok or not px4_ok:
            # Don't keep navigating on stale localization. Stream zero velocity
            # while active; do not pretend the estimate is still trustworthy.
            self.publish_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            if self.state != 'fault':
                self.get_logger().error(f'Localization gate failed: {odom_reason}; {px4_reason}')
                self.set_state('fault')
            return

        if self.state == 'arming':
            self.publish_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            if self.tick == 20:  # 1 second prestream at 20 Hz
                self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self.tick == 25:
                self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if self.tick > 30:
                self.set_state('climbing')
            return

        if self.state == 'climbing':
            target_z = -self.alt  # NED: upward altitude is negative z
            error = target_z - self.lpos.z
            if abs(error) < 0.10:
                self.set_state('flying')
                return
            if now - self.state_t > float(self.get_parameter('climb_timeout_sec').value):
                self.get_logger().error('Takeoff altitude not reached before timeout. Holding zero velocity; inspect PX4 before continuing.')
                self.set_state('fault')
                self.publish_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
                return
            vd = -self.clamp(abs(self.climb), 0.1, 0.5) if error < 0 else self.clamp(abs(self.climb), 0.1, 0.5)
            self.publish_velocity_setpoint(0.0, 0.0, vd, 0.0)
            return

        if self.state == 'fault':
            self.publish_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            return

        # flying: only accept fresh Nav2 commands and keep altitude by a bounded
        # vertical-velocity correction. cmd_vel x/y are body FLU; output is NED.
        fresh_cmd = now - self.cmd_t < float(self.get_parameter('cmd_timeout_sec').value)
        vx = self.cmd.linear.x if fresh_cmd else 0.0
        vy = self.cmd.linear.y if fresh_cmd else 0.0
        wz = self.cmd.angular.z if fresh_cmd else 0.0
        c, s = math.cos(self.yaw_enu), math.sin(self.yaw_enu)
        v_enu_x = c * vx - s * vy
        v_enu_y = s * vx + c * vy
        vn, ve = v_enu_y, v_enu_x
        target_z = -self.alt
        vd = self.clamp(0.8 * (target_z - self.lpos.z), -0.35, 0.35)
        # If estimator/input health fails, loop above enters fault before this.
        self.publish_velocity_setpoint(vn, ve, vd, -wz)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPx4()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
