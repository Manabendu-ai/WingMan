#!/usr/bin/env python3

import math
import asyncio
import threading
import base64
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger

from px4_msgs.msg import (
    VehicleCommand,
    VehicleCommandAck,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleLocalPosition,
)

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn

NAN = float("nan")

# FastAPI App setup for WebSocket streaming
app = FastAPI()

class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
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
        self.declare_parameter("vmax", 0.3)
        self.declare_parameter("climb_rate", 0.3)          # m/s, used for the takeoff climb
        self.declare_parameter("target_alt", 0.5)
        self.declare_parameter("allow_takeoff", False)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("local_position_timeout", 0.5)
        self.declare_parameter("takeoff_timeout", 30.0)    # Increased timeout for safety
        self.declare_parameter("position_tolerance", 0.15)
        self.declare_parameter("max_yaw_rate", 0.5)
        self.declare_parameter("invalid_frames", 25)       # 25 ticks at 20 Hz = 1.25 s grace
        self.declare_parameter("lateral_sign", 1.0)        # +1: ROS y=left is sent as left. -1 flips it.
        self.declare_parameter("yaw_sign", 1.0)            # +1: ROS CCW+ yaw rate is sent as CCW. -1 flips it.

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

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        # ---------------- Subscribers ----------------
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(VehicleLocalPosition, self.local_position_topic, self.local_pos_cb, sensor_qos)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self.ack_cb, sensor_qos)
        self.create_subscription(CompressedImage, self.camera_compressed_topic, self.image_cb, sensor_qos)

        # ---------------- Publishers ----------------
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", reliable_qos)
        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", sensor_qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", sensor_qos)

        # ---------------- Services ----------------
        self.create_service(Trigger, "/wingman/takeoff", self.takeoff_cb)
        self.create_service(Trigger, "/wingman/land", self.land_cb)
        self.create_service(Trigger, "/wingman/check_readiness", self.readiness_cb)

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
                    f"{float(self.local_pos.vy):.2f})")

        if self.state in ("idle", "error"):
            return

        # 0. Landing
        if self.state == "landing":
            lp = self.local_pos
            if lp is not None:
                self.publish_setpoint([NAN, NAN, NAN], [0.0, 0.0, 0.2], float(lp.heading))
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
                    self.get_logger().info(f"Armed. Ground z={self.start_z:.2f}, target z={self.target_z:.2f}")
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

            if float(lp.z) <= self.target_z + self.position_tolerance:
                self.hold_z = self.target_z
                self.get_logger().info(f"Target altitude reached (z={float(lp.z):.2f}). Locking hover at {self.hold_z:.2f}.")
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

            north, east = body_to_ned(vx_body, vy_body, float(lp.heading), self.lateral_sign)
            self.publish_setpoint([NAN, NAN, float(self.hold_z)], [north, east, NAN], NAN,
                                  -self.yaw_sign * yaw_rate)


# FastAPI WebSocket Endpoints
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


@app.on_event("startup")
async def startup_event():
    asyncio.create_task(broadcast_loop())


def main(args=None):
    global ros_node_global
    rclpy.init(args=args)
    ros_node_global = CmdVelToPX4()

    # Start FastAPI server in a background daemon thread
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