#!/usr/bin/env python3

import math
import asyncio
import threading
import base64
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, LaserScan
from std_srvs.srv import Trigger

from px4_msgs.msg import (
    VehicleCommand,
    VehicleCommandAck,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleLocalPosition,
)

from std_msgs.msg import Float32, String

try:                                    # optional live-telemetry website; the controller works without it
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    import uvicorn
    HAVE_WEB = True
except Exception:                       # fastapi/uvicorn not installed
    FastAPI = WebSocket = WebSocketDisconnect = uvicorn = None
    HAVE_WEB = False

NAN = float("nan")

# FastAPI App setup for WebSocket streaming
app = FastAPI() if HAVE_WEB else None

class ConnectionManager:
    def __init__(self):
        self.active_connections: list = []

    async def connect(self, websocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception:
                pass

manager = ConnectionManager()
ros_node_global = None


def body_to_ned(vx_forward, vy_left, heading, lateral_sign=1.0):
    """ROS body velocity (FLU) -> NED (north, east) for a vehicle with NED heading `heading` (rad, clockwise from N)."""
    vy = lateral_sign * vy_left
    north = math.cos(heading) * vx_forward + math.sin(heading) * vy
    east = math.sin(heading) * vx_forward - math.cos(heading) * vy
    return north, east


class CmdVelToPX4(Node):

    def __init__(self):
        super().__init__("cmdvel_to_px4")

        # ---------------- Parameters ----------------
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("camera_compressed_topic", "/gimbal/camera/compressed")
        self.declare_parameter("vmax", 1.2)             # m/s hard cap on commanded horizontal speed
        self.declare_parameter("climb_rate", 0.5)          # m/s, used for the takeoff climb
        self.declare_parameter("target_alt", 1.0)          # flight height above the takeoff point [m]
        self.declare_parameter("allow_takeoff", True)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("local_position_timeout", 0.5)
        self.declare_parameter("takeoff_timeout", 30.0)    # Increased timeout for safety
        self.declare_parameter("position_tolerance", 0.15)
        self.declare_parameter("max_yaw_rate", 1.2)
        self.declare_parameter("invalid_frames", 25)       # 25 ticks at 20 Hz = 1.25 s grace
        self.declare_parameter("lateral_sign", 1.0)        # +1: ROS y=left is sent as left. -1 flips it.
        self.declare_parameter("yaw_sign", 1.0)            # +1: ROS CCW+ yaw rate is sent as CCW. -1 flips it.
        self.declare_parameter("accel_limit", 2.0)         # m/s^2 horizontal speed ramp-up
        self.declare_parameter("decel_limit", 3.5)         # m/s^2 horizontal braking
        # --- rangefinder (downward) height hold: the barometer drifts in Gazebo, so height is closed on the real
        #     height above the floor instead. Falls back to the old barometer z-hold when the range is missing.
        self.declare_parameter("range_topic", "/wingman/range")   # sensor_msgs/LaserScan from the down-facing gz lidar
        self.declare_parameter("use_range_hold", True)
        self.declare_parameter("range_timeout", 0.5)       # s without a valid range -> fall back to barometer hold
        self.declare_parameter("alt_kp", 1.2)              # (m/s) per metre of height error
        self.declare_parameter("alt_vmax", 0.4)            # m/s cap on the vertical correction
        self.declare_parameter("min_safe_alt", 0.25)       # below this height horizontal motion is frozen
        self.declare_parameter("abort_alt", 0.10)          # below this for abort_time -> land
        self.declare_parameter("abort_time", 3.0)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.local_position_topic = str(self.get_parameter("local_position_topic").value)
        self.camera_compressed_topic = str(self.get_parameter("camera_compressed_topic").value)
        self.vmax = float(self.get_parameter("vmax").value)
        self.climb_rate = abs(float(self.get_parameter("climb_rate").value))
        self.target_alt = float(self.get_parameter("target_alt").value)
        self.allow_takeoff = bool(self.get_parameter("allow_takeoff").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.local_position_timeout = float(self.get_parameter("local_position_timeout").value)
        self.takeoff_timeout = float(self.get_parameter("takeoff_timeout").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.invalid_frames = int(self.get_parameter("invalid_frames").value)
        self.lateral_sign = float(self.get_parameter("lateral_sign").value)
        self.yaw_sign = float(self.get_parameter("yaw_sign").value)
        self.accel_limit = float(self.get_parameter("accel_limit").value)
        self.decel_limit = float(self.get_parameter("decel_limit").value)
        self.range_topic = str(self.get_parameter("range_topic").value)
        self.use_range_hold = bool(self.get_parameter("use_range_hold").value)
        self.range_timeout = float(self.get_parameter("range_timeout").value)
        self.alt_kp = float(self.get_parameter("alt_kp").value)
        self.alt_vmax = float(self.get_parameter("alt_vmax").value)
        self.min_safe_alt = float(self.get_parameter("min_safe_alt").value)
        self.abort_alt = float(self.get_parameter("abort_alt").value)
        self.abort_time = float(self.get_parameter("abort_time").value)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        # ---------------- Subscribers ----------------
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(VehicleLocalPosition, self.local_position_topic, self.local_pos_cb, sensor_qos)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self.ack_cb, sensor_qos)
        self.create_subscription(CompressedImage, self.camera_compressed_topic, self.image_cb, sensor_qos)
        self.create_subscription(LaserScan, self.range_topic, self.range_cb, sensor_qos)
        # mission altitude (metres above the floor), e.g. 0.75 from "explore the maze ... at 0.75 m"
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Float32, "/wingman/target_alt", self.target_alt_cb, latched)

        # ---------------- Publishers ----------------
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", reliable_qos)
        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", sensor_qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", sensor_qos)

        # ---------------- Services ----------------
        self.create_service(Trigger, "/wingman/takeoff", self.takeoff_cb)
        self.create_service(Trigger, "/wingman/land", self.land_cb)
        self.create_service(Trigger, "/wingman/check_readiness", self.readiness_cb)
        self.state_pub = self.create_publisher(String, "/wingman/flight_state", 10)
        self.create_timer(0.2, self.publish_flight_state)

        # ---------------- State ----------------
        self.state = "idle"
        self.state_t = self.now_s()

        self.cmd = Twist()
        self.cmd_t = 0.0

        self.odom_msg: Optional[Odometry] = None
        self.odom_t = 0.0

        self.local_pos: Optional[VehicleLocalPosition] = None
        self.local_pos_t = 0.0

        self.latest_image_b64 = ""

        self.ack_by_command = {}
        self.pending_command = None

        self.start_z = None
        self.target_z = None
        self.hold_z = None

        self.invalid_count = 0
        self.offboard_setpoint_count = 0
        self.last_log_t = 0.0
        self.v_ned = [0.0, 0.0]

        self.range_raw = None          # latest valid down-range [m]
        self.range_t = 0.0
        self.range_f = None            # low-passed down-range [m]
        self.range_ground = None       # down-range while sitting on the ground (offset of the sensor above the floor)
        self.low_since = None          # time the height first dropped below abort_alt

        self.create_timer(0.05, self.loop)

        self.get_logger().warning(
            "Controller initialized in safety-guarded mode. "
            f"odom_topic={self.odom_topic}, allow_takeoff={self.allow_takeoff}, target_alt={self.target_alt}m, "
            f"climb_rate={self.climb_rate}m/s, lateral_sign={self.lateral_sign}, yaw_sign={self.yaw_sign}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def set_state(self, state: str):
        self.get_logger().info(f"State: {self.state} -> {state}")
        self.state = state
        self.state_t = self.now_s()

    def cmd_cb(self, msg: Twist):
        self.cmd = msg
        self.cmd_t = self.now_s()

    def odom_cb(self, msg: Odometry):
        self.odom_msg = msg
        self.odom_t = self.now_s()

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.local_pos = msg
        self.local_pos_t = self.now_s()

    def image_cb(self, msg: CompressedImage):
        img_encoded = base64.b64encode(msg.data).decode('utf-8')
        self.latest_image_b64 = f"data:image/jpeg;base64,{img_encoded}"

    def range_cb(self, msg: LaserScan):
        vals = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if not vals:
            return
        r = min(vals)
        self.range_raw = r
        self.range_t = self.now_s()
        self.range_f = r if self.range_f is None else 0.7 * self.range_f + 0.3 * r

    def target_alt_cb(self, msg: Float32):
        v = float(msg.data)
        if not math.isfinite(v):
            return
        v = max(0.4, min(2.0, v))
        if abs(v - self.target_alt) < 1e-3:
            return
        self.get_logger().info(f"Target altitude {self.target_alt:.2f} m -> {v:.2f} m")
        self.target_alt = v
        if self.start_z is not None and self.state in ("prestream", "offboard_request", "arming", "climbing", "flying"):
            self.target_z = self.start_z - v          # barometer-based fallback target follows too
            if self.state == "flying" and self.agl() is None:
                self.hold_z = self.target_z

    def agl(self):
        """Height above the floor [m] from the down-facing rangefinder, or None when unavailable / not calibrated."""
        if not self.use_range_hold or self.range_f is None or self.range_ground is None:
            return None
        if self.now_s() - self.range_t > self.range_timeout:
            return None
        return max(0.0, self.range_f - self.range_ground)

    def ack_cb(self, msg: VehicleCommandAck):
        self.ack_by_command[msg.command] = msg.result
        if self.pending_command == msg.command:
            if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info(f"PX4 accepted command {msg.command}")
            else:
                self.get_logger().error(f"PX4 rejected command {msg.command}, result={msg.result}")
            self.pending_command = None

    def readiness_errors(self, strict_heading: bool = True):
        errors = []
        now = self.now_s()

        if self.odom_msg is None:
            errors.append(f"No odometry received on {self.odom_topic}")
        elif now - self.odom_t > self.odom_timeout:
            errors.append(f"Visual odometry is stale ({now - self.odom_t:.2f}s)")

        if self.local_pos is None:
            errors.append(f"No PX4 local position on {self.local_position_topic}")
        else:
            if now - self.local_pos_t > self.local_position_timeout:
                errors.append("PX4 local-position data is stale")
            if not self.local_pos.xy_valid:
                errors.append("PX4 local XY position is invalid")
            if not self.local_pos.z_valid:
                errors.append("PX4 local Z position is invalid")
            if not self.local_pos.v_xy_valid:
                errors.append("PX4 horizontal velocity is invalid")
            if not self.local_pos.v_z_valid:
                errors.append("PX4 vertical velocity is invalid")
            if strict_heading and not self.local_pos.heading_good_for_control:
                errors.append("PX4 heading is not valid for control")
        return errors

    def publish_flight_state(self):
        alt = None
        if self.local_pos is not None and self.start_z is not None:
            alt = round(float(self.start_z) - float(self.local_pos.z), 2)
        a = self.agl()
        if a is not None:
            alt = round(a, 2)
        ready = len(self.readiness_errors(strict_heading=True)) == 0
        m = String()
        m.data = json.dumps({"state": self.state, "alt_m": alt, "ready": ready,
                             "target_alt_m": round(self.target_alt, 2),
                             "range_ok": self.agl() is not None,
                             "baro_alt_m": (round(float(self.start_z) - float(self.local_pos.z), 2)
                                            if (self.local_pos is not None and self.start_z is not None) else None)})
        self.state_pub.publish(m)

    def readiness_cb(self, request, response):
        errors = self.readiness_errors(strict_heading=True)
        response.success = len(errors) == 0
        response.message = "READY: input checks passed" if not errors else "NOT READY: " + "; ".join(errors)
        return response

    def send_cmd(self, command: int, p1: float = 0.0, p2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.now_s() * 1e6)
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pending_command = command
        self.cmd_pub.publish(msg)

    def publish_setpoint(self, pos, vel, yaw, yawspeed=0.0):
        ts = int(self.now_s() * 1e6)

        has_pos_xy = math.isfinite(pos[0]) and math.isfinite(pos[1])
        has_pos_z = math.isfinite(pos[2])
        has_vel_xy = math.isfinite(vel[0]) and math.isfinite(vel[1])
        has_vel_z = math.isfinite(vel[2])

        mode = OffboardControlMode()
        mode.timestamp = ts
        mode.position = has_pos_xy or has_pos_z
        mode.velocity = has_vel_xy or has_vel_z
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.offboard_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float(v) if math.isfinite(v) else NAN for v in pos]
        sp.velocity = [float(v) if math.isfinite(v) else NAN for v in vel]
        sp.acceleration = [NAN, NAN, NAN]
        sp.jerk = [NAN, NAN, NAN]
        sp.yaw = float(yaw) if math.isfinite(yaw) else NAN
        sp.yawspeed = float(yawspeed) if math.isfinite(yawspeed) else NAN
        self.traj_pub.publish(sp)
        self.offboard_setpoint_count += 1

    def takeoff_cb(self, request, response):
        if not self.allow_takeoff:
            response.success = False
            response.message = ("Takeoff disabled: set allow_takeoff:=true only after localization "
                                "and PX4 integration are validated.")
            return response

        if self.state != "idle":
            response.success = False
            response.message = f"Controller is in state {self.state}"
            return response

        errors = self.readiness_errors(strict_heading=True)
        if errors:
            response.success = False
            response.message = "NOT READY: " + "; ".join(errors)
            return response

        self.pending_command = None
        self.ack_by_command.clear()
        self.start_z = float(self.local_pos.z)
        self.target_z = self.start_z - self.target_alt
        self.hold_z = self.target_z
        self.invalid_count = 0
        self.offboard_setpoint_count = 0
        self.range_ground = self.range_f if self.range_f is not None else None   # on the ground right now
        self.low_since = None
        if self.use_range_hold:
            if self.range_ground is None:
                self.get_logger().warning(f"No rangefinder data on {self.range_topic}: height hold will use the "
                                          "(drifting) barometer. Start the range bridge (see mission.launch.py).")
            else:
                self.get_logger().info(f"Rangefinder ground reference = {self.range_ground:.3f} m")

        self.set_state("prestream")
        response.success = True
        response.message = "Prestream started; validating offboard mode and arming."
        return response

    def land_cb(self, request, response):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.set_state("landing")
        self.pending_command = None
        self.ack_by_command.clear()
        self.hold_z = None
        response.success = True
        response.message = "Landing command sent to PX4"
        return response

    def loop(self):
        now = self.now_s()

        if now - self.last_log_t > 2.0:
            self.last_log_t = now
            if self.state == "idle":
                errors = self.readiness_errors(strict_heading=True)
                if errors:
                    self.get_logger().warning("NOT READY: " + "; ".join(errors))
            elif self.state in ("climbing", "flying") and self.local_pos is not None:
                self.get_logger().info(
                    f"{self.state}: z={float(self.local_pos.z):.2f} (target {self.target_z:.2f}) "
                    f"heading={float(self.local_pos.heading):.2f} v=({float(self.local_pos.vx):.2f},"
                    f"{float(self.local_pos.vy):.2f}) agl={self.agl()} range_f={self.range_f} "
                    f"ground_ref={self.range_ground} baro_alt={float(self.start_z) - float(self.local_pos.z):.2f}")
                a = self.agl()
                if a is not None and abs(a - (float(self.start_z) - float(self.local_pos.z))) > 0.5:
                    self.get_logger().warning(f"Rangefinder height {a:.2f} m and barometer height "
                                              f"{float(self.start_z) - float(self.local_pos.z):.2f} m disagree by > 0.5 m")

        if self.state in ("idle", "error"):
            return

        # 0. Landing
        if self.state == "landing":
            lp = self.local_pos
            if lp is not None:
                self.publish_setpoint([NAN, NAN, NAN], [0.0, 0.0, 0.4], float(lp.heading))
                ground_z = self.start_z - 0.08 if self.start_z is not None else -0.10
                if abs(float(lp.vz)) < 0.05 and float(lp.z) >= ground_z:
                    self.get_logger().info("Touchdown detected. Disarming vehicle.")
                    self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                    self.set_state("idle")
            return

        # Debounced mid-flight safety check
        errors = self.readiness_errors(strict_heading=False)
        if errors:
            self.invalid_count += 1
            self.get_logger().warning(f"Readiness degraded ({self.invalid_count}/{self.invalid_frames}): "
                                      + "; ".join(errors), throttle_duration_sec=1.0)
            if self.invalid_count >= self.invalid_frames:
                self.get_logger().error("Readiness lost in active state! Triggering emergency land: "
                                        + "; ".join(errors))
                self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.set_state("error")
                return
        else:
            self.invalid_count = 0

        lp = self.local_pos
        if lp is None:
            return

        # 1. Prestream setpoints
        if self.state == "prestream":
            self.publish_setpoint([NAN, NAN, float(lp.z)], [0.0, 0.0, 0.0], float(lp.heading))
            if now - self.state_t >= 1.5 and self.offboard_setpoint_count >= 20:
                self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.set_state("offboard_request")
            return

        # 2. Confirm offboard mode
        if self.state == "offboard_request":
            self.publish_setpoint([NAN, NAN, float(lp.z)], [0.0, 0.0, 0.0], float(lp.heading))
            if self.pending_command is None:
                if self.ack_by_command.get(VehicleCommand.VEHICLE_CMD_DO_SET_MODE) == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                    self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    self.set_state("arming")
                else:
                    self.get_logger().error("Offboard mode switch was rejected by PX4.")
                    self.set_state("error")
            elif now - self.state_t > 3.0:
                self.get_logger().error("Offboard mode acknowledgement timeout.")
                self.set_state("error")
            return

        # 3. Confirm arming
        if self.state == "arming":
            self.publish_setpoint([NAN, NAN, float(lp.z)], [0.0, 0.0, 0.0], float(lp.heading))
            if self.pending_command is None:
                if self.ack_by_command.get(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM) == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                    self.start_z = float(lp.z)
                    self.target_z = self.start_z - self.target_alt
                    self.hold_z = self.target_z
                    if self.range_f is not None:
                        self.range_ground = self.range_f
                    elif self.use_range_hold and self.range_ground is None:
                        # the sensor reads nothing on the ground (floor closer than its min range): assume the
                        # reading in the air IS the height above the floor, instead of silently using the barometer
                        self.range_ground = 0.0
                        self.get_logger().warning("No valid rangefinder reading on the ground (floor closer than the "
                                                  "sensor's min range?). Assuming ground reference 0.0 m.")
                    self.low_since = None
                    self.get_logger().info(f"Armed. Ground z={self.start_z:.2f}, target z={self.target_z:.2f}, "
                                           f"range ground ref={self.range_ground}")
                    self.set_state("climbing")
                else:
                    self.get_logger().error("Arm command was rejected by PX4.")
                    self.set_state("error")
            elif now - self.state_t > 3.0:
                self.get_logger().error("Arming command acknowledgement timeout.")
                self.set_state("error")
            return

        # 4. Rate-limited climb
        if self.state == "climbing":
            if now - self.state_t > self.takeoff_timeout:
                self.get_logger().error("Takeoff altitude target timeout reached!")
                self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.set_state("error")
                return

            # Explicit negative Z velocity for upward climb in NED frame
            self.publish_setpoint([NAN, NAN, NAN], [0.0, 0.0, -self.climb_rate], float(lp.heading))

            a = self.agl()
            reached = (a >= self.target_alt - self.position_tolerance) if a is not None \
                else (float(lp.z) <= self.target_z + self.position_tolerance)
            if reached:
                self.hold_z = self.target_z
                self.get_logger().info(f"Target altitude reached (z={float(lp.z):.2f}). Locking hover at {self.hold_z:.2f}.")
                self.v_ned = [float(lp.vx), float(lp.vy)]
                self.set_state("flying")
            return

        # 5. Flying: Nav2 velocity commands
        if self.state == "flying":
            if now - self.cmd_t > 0.5:
                vx_body = vy_body = yaw_rate = 0.0
            else:
                vx_body = max(-self.vmax, min(self.vmax, float(self.cmd.linear.x)))
                vy_body = max(-self.vmax, min(self.vmax, float(self.cmd.linear.y)))
                yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, float(self.cmd.angular.z)))

            # ---- height: close the loop on the real height above the floor when the rangefinder is alive
            a = self.agl()
            vz_cmd = None
            if a is not None:
                vz_cmd = max(-self.alt_vmax, min(self.alt_vmax, self.alt_kp * (a - self.target_alt)))  # NED: + is down
                self.hold_z = float(lp.z) - (self.target_alt - a)     # keeps the barometer fallback consistent
                if a < self.abort_alt:
                    self.low_since = self.low_since if self.low_since is not None else now
                    if now - self.low_since > self.abort_time:
                        self.get_logger().error(f"Height {a:.2f} m for {self.abort_time:.0f}s - landing.")
                        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                        self.set_state("landing")
                        return
                else:
                    self.low_since = None
                if a < self.min_safe_alt:       # too low: no horizontal motion, just climb back up
                    vx_body = vy_body = 0.0
                    self.get_logger().warning(f"Too low ({a:.2f} m): horizontal motion frozen, climbing",
                                              throttle_duration_sec=1.0)

            north, east = body_to_ned(vx_body, vy_body, float(lp.heading), self.lateral_sign)
            # slew-limit the horizontal velocity setpoint (smooth starts, firm stops)
            dt = 0.05
            cur_speed = math.hypot(*self.v_ned)
            tgt_speed = math.hypot(north, east)
            lim = (self.accel_limit if tgt_speed > cur_speed else self.decel_limit) * dt
            dn, de = north - self.v_ned[0], east - self.v_ned[1]
            dmag = math.hypot(dn, de)
            if dmag > lim > 0.0:
                dn, de = dn * lim / dmag, de * lim / dmag
            self.v_ned = [self.v_ned[0] + dn, self.v_ned[1] + de]
            if vz_cmd is not None:
                self.publish_setpoint([NAN, NAN, NAN], [self.v_ned[0], self.v_ned[1], vz_cmd], NAN,
                                      -self.yaw_sign * yaw_rate)
            else:
                self.publish_setpoint([NAN, NAN, float(self.hold_z)], [self.v_ned[0], self.v_ned[1], NAN], NAN,
                                      -self.yaw_sign * yaw_rate)


# FastAPI WebSocket Endpoints (only when fastapi/uvicorn are installed)
if HAVE_WEB:
    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await manager.connect(websocket)
        try:
            while True:
                data = await websocket.receive_text()
        except WebSocketDisconnect:
            manager.disconnect(websocket)


async def broadcast_loop():
    """Streams live telemetry and camera feed to connected website clients at ~20 FPS."""
    while True:
        if manager.active_connections and ros_node_global:
            odom_data = {"x": 0.0, "y": 0.0, "z": 0.0}
            if ros_node_global.local_pos is not None:
                odom_data = {
                    "x": float(ros_node_global.local_pos.x),
                    "y": float(ros_node_global.local_pos.y),
                    "z": float(ros_node_global.local_pos.z)
                }

            payload = {
                "type": "drone_telemetry",
                "state": ros_node_global.state,
                "odom": odom_data,
                "image": ros_node_global.latest_image_b64
            }
            await manager.broadcast(json.dumps(payload))
        await asyncio.sleep(0.05)


if HAVE_WEB:
    @app.on_event("startup")
    async def startup_event():
        asyncio.create_task(broadcast_loop())


def main(args=None):
    global ros_node_global
    rclpy.init(args=args)
    ros_node_global = CmdVelToPX4()

    # Start FastAPI server in a background daemon thread (optional)
    if HAVE_WEB:
        fastapi_thread = threading.Thread(
            target=lambda: uvicorn.run(app, host="0.0.0.0", port=8765, log_level="warning"),
            daemon=True
        )
        fastapi_thread.start()

    try:
        rclpy.spin(ros_node_global)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node_global.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()