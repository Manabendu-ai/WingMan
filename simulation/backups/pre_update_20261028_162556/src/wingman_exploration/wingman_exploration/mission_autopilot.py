#!/usr/bin/env python3
"""WingMan mission autopilot: ONE node that actually flies the mission.

takeoff (auto) -> explore frontiers / find target / go to point -> return -> land
It publishes /cmd_vel for wingman_control/cmdvel_to_px4 (which owns arming, offboard mode,
the 1 m altitude hold and landing). The VLM is NOT in the flight loop: flight uses LiDAR + map only;
the VLM is asked to confirm a target when the mission has one.

Give a mission at any time (free text or JSON):
  ros2 topic pub --once /wingman/mission_command std_msgs/msg/String "{data: 'find the person and come back'}"
  or: ros2 run wingman_vlm prompt_cli
Status:  ros2 topic echo /wingman/mission_status
"""
import json
import math
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from wingman_exploration.explorer_core import (Explorer, PathTracker, build_view, scan_clearances)
from wingman_exploration.mission_parser import parse_mission

MOTION = ("EXPLORE", "RETURN", "GOTO")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionAutopilot(Node):
    def __init__(self):
        super().__init__("wingman_mission_autopilot")
        d = self.declare_parameter
        d("map_topic", "/map")
        d("scan_topic", "/scan")
        d("map_frame", "map")
        d("base_frame", "base_link")
        d("cruise_speed", 0.8)            # m/s while exploring / returning
        d("search_speed", 0.6)            # m/s while a VLM target search is active (more frames per metre)
        d("max_yaw_rate", 1.0)
        d("inflation_m", 0.45)
        d("plan_resolution_m", 0.10)
        d("plan_period_s", 1.0)
        d("use_explore_box", True)        # box (in the START frame) that frontiers must lie in
        d("explore_box", [1.5, 32.9, -15.8, 15.8])   # xmin, xmax, ymin, ymax [m]; maze interior for maze_exploration.sdf
        d("auto_takeoff", True)
        d("takeoff_timeout_s", 90.0)
        d("mission_timeout_s", 1500.0)
        d("vlm_conf_min", 0.5)
        d("vlm_confirm_needed", 2)
        d("vlm_direction_bonus_m", 1.0)   # soft hint only; 0 disables
        d("empty_cycles_to_finish", 3)
        g = lambda k: self.get_parameter(k).value
        self.map_frame, self.base_frame = str(g("map_frame")), str(g("base_frame"))
        self.cruise, self.search_speed = float(g("cruise_speed")), float(g("search_speed"))
        self.inflation, self.plan_res = float(g("inflation_m")), float(g("plan_resolution_m"))
        self.plan_period = float(g("plan_period_s"))
        self.use_box, self.box = bool(g("use_explore_box")), [float(v) for v in g("explore_box")]
        self.auto_takeoff, self.takeoff_timeout = bool(g("auto_takeoff")), float(g("takeoff_timeout_s"))
        self.mission_timeout = float(g("mission_timeout_s"))
        self.conf_min, self.confirm_needed = float(g("vlm_conf_min")), int(g("vlm_confirm_needed"))
        self.empty_needed = int(g("empty_cycles_to_finish"))

        self.lock = threading.Lock()
        self.explorer = Explorer(vlm_bonus=float(g("vlm_direction_bonus_m")))
        self.tracker = PathTracker(cruise=self.cruise, max_yaw=float(g("max_yaw_rate")))
        self.events = deque()           # control -> planner thread messages
        self._new_plan = None
        self.plan = None
        self.state, self.mission = "IDLE", None
        self.mission_t0 = 0.0
        self.map_msg, self.map_stamp = None, 0
        self.pose = None                # (x, y, yaw) in map frame, updated by control loop
        self.scan = None
        self.scan_t = 0.0
        self.flight = {"state": "unknown"}
        self.flight_t = 0.0
        self.entrance = None            # (x, y, yaw) in map frame, recorded at takeoff
        self.bounds = None
        self.empty_cycles = 0
        self.found = []
        self.vlm_ready = False
        self.vlm_dir, self.vlm_dir_t = "unknown", 0.0
        self.takeoff_deadline = 0.0
        self.takeoff_pending = False
        self.takeoff_next = 0.0
        self.land_requested = False
        self.last_plan_info = ""
        self.confirm = None             # dict while CONFIRM
        self.prev_state = "EXPLORE"
        self.suppress = []              # [(x, y, until_wall)] ignore detections near here
        self.hist = deque()
        self.odo = 0.0
        self.empty_since = 0.0
        self.goal_announced = None
        self.last_query_pub = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, str(g("map_topic")), self.on_map, map_qos)
        self.create_subscription(LaserScan, str(g("scan_topic")), self.on_scan, sensor_qos)
        self.create_subscription(String, "/wingman/mission_command", self.on_command, 10)
        self.create_subscription(String, "/wingman/flight_state", self.on_flight, 10)
        self.create_subscription(String, "/wingman/vlm/observation", self.on_obs, 10)
        self.create_subscription(String, "/wingman/vlm/status", self.on_vlm_status, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/wingman/mission_status", 10)
        self.query_pub = self.create_publisher(String, "/wingman/vlm_query", 10)
        self.path_pub = self.create_publisher(Path, "/wingman/planner_debug/path", 10)
        self.marker_pub = self.create_publisher(Marker, "/wingman/planner_debug/goal_marker", 10)
        self.takeoff_cli = self.create_client(Trigger, "/wingman/takeoff")
        self.land_cli = self.create_client(Trigger, "/wingman/land")
        self.create_service(Trigger, "/wingman/mission/abort", self.srv_abort)
        self.create_service(Trigger, "/wingman/mission/land", self.srv_land)
        self.create_service(Trigger, "/wingman/custom_planner/start", self.srv_start)   # backwards compatible
        self.create_service(Trigger, "/wingman/custom_planner/stop", self.srv_abort)
        self.create_timer(0.05, self.control)
        self.create_timer(0.5, self.publish_status)
        self._run = True
        threading.Thread(target=self.planner_loop, daemon=True).start()
        self.get_logger().info("Mission autopilot ready. Send a mission, e.g.: ros2 topic pub --once "
                               "/wingman/mission_command std_msgs/msg/String \"{data: 'explore the maze'}\"")

    # ------------------------------------------------------------------ inputs
    def on_map(self, msg):
        self.map_msg, self.map_stamp = msg, self.map_stamp + 1

    def on_scan(self, msg):
        self.scan, self.scan_t = msg, time.monotonic()

    def on_flight(self, msg):
        try:
            self.flight = json.loads(msg.data)
        except Exception:
            self.flight = {"state": msg.data}
        self.flight_t = time.monotonic()

    def on_vlm_status(self, msg):
        try:
            self.vlm_ready = bool(json.loads(msg.data).get("ready"))
        except Exception:
            pass

    def fstate(self):
        if time.monotonic() - self.flight_t > 2.0:
            return "unknown"
        return str(self.flight.get("state", "unknown"))

    def on_command(self, msg):
        m, err = parse_mission(msg.data)
        if m is None:
            self.get_logger().error(f"Mission rejected: {err}")
            self.say(f"REJECTED: {err}")
            return
        self.start_mission(m)

    def srv_start(self, _req, res):
        self.start_mission(parse_mission("explore the maze")[0])
        res.success, res.message = True, "explore mission started"
        return res

    def srv_abort(self, _req, res):
        self.set_state("HOVER")
        self.clear_plan()
        self.query("")
        res.success, res.message = True, "mission aborted; hovering (use /wingman/mission/land to land)"
        return res

    def srv_land(self, _req, res):
        self.begin_landing()
        res.success, res.message = True, "landing"
        return res

    # ------------------------------------------------------------------ helpers
    def say(self, text):
        self.last_say = text
        self.get_logger().info(text)

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f"state {self.state} -> {s}")
            self.state = s
            self.hist.clear()

    def query(self, target):
        m = String()
        m.data = target
        self.query_pub.publish(m)
        self.last_query_pub = time.monotonic()

    def clear_plan(self):
        with self.lock:
            self.plan, self._new_plan = None, None
        self.tracker.set_path([])
        self.tracker.reset_cmd()

    def zero(self):
        self.cmd_pub.publish(Twist())

    def start_mission(self, m):
        self.say(f"MISSION: {m['type']} target={m['target']!r} then={m['then']} ('{m['text']}')")
        self.mission, self.mission_t0 = m, time.monotonic()
        self.found, self.empty_cycles, self.confirm = [], 0, None
        self.clear_plan()
        self.land_requested = False
        self.events.append(("reset",))
        t = m["type"]
        self.query(m["target"] if t == "find" else "")
        if t == "land":
            self.begin_landing()
            return
        if t == "hover":
            self.set_state("HOVER")
            return
        if self.fstate() != "flying":
            if not self.auto_takeoff:
                self.say("not flying and auto_takeoff=false: call /wingman/takeoff first")
                self.set_state("IDLE")
                return
            self.takeoff_deadline = time.monotonic() + self.takeoff_timeout
            self.takeoff_pending, self.takeoff_next = False, 0.0
            self.set_state("TAKEOFF")
        else:
            if self.entrance is None and self.pose is not None:
                self.entrance = self.pose
            self.begin_motion()

    def begin_motion(self):
        t = self.mission["type"]
        if self.pose is not None and self.entrance is None:
            self.entrance = self.pose
        if self.use_box and self.entrance is not None:
            x0, y0, yaw0 = self.entrance
            self.bounds = (x0, y0, yaw0, *self.box)
        else:
            self.bounds = None
        if t in ("explore", "find"):
            self.set_state("EXPLORE")
        elif t == "goto":
            self.set_state("GOTO")
        elif t == "return":
            self.set_state("RETURN")
        else:   # takeoff only
            self.set_state("HOVER")

    def begin_landing(self):
        self.clear_plan()
        self.query("")
        self.land_requested = False
        self.set_state("LANDING")

    def finish(self, why, then):
        self.say(f"{why} -> {then}")
        if then == "return":
            self.clear_plan()
            self.set_state("RETURN")
        elif then == "land":
            self.begin_landing()
        else:
            self.clear_plan()
            self.query("")
            self.set_state("HOVER")

    def after_goal(self, found_target):
        m = self.mission
        then = m["then"]
        if m["type"] == "find" and not found_target:
            self.finish("target NOT found after exploring everything reachable", "return")
        elif m["type"] == "explore":
            self.finish("exploration complete", "return" if then == "continue" else then)
        else:
            self.finish("search complete" if then == "continue" else "target found",
                        "return" if then == "continue" else then)

    # ------------------------------------------------------------------ VLM
    def on_obs(self, msg):
        try:
            o = json.loads(msg.data)
        except Exception:
            return
        if not o.get("ok"):
            self.get_logger().warning(f"VLM error: {o.get('error')}", throttle_duration_sec=5.0)
            return
        m = self.mission
        if m is None or m["type"] != "find" or "seen" not in o:
            return
        if o.get("dir") in ("left", "forward", "right"):
            self.vlm_dir, self.vlm_dir_t = o["dir"], time.monotonic()
        positive = bool(o.get("seen")) and float(o.get("conf", 0)) >= self.conf_min
        ft = float(o.get("frame_time", 0))
        if o.get("target") != m["target"]:
            return
        with self.lock:
            if self.state in MOTION and positive and self.state == "EXPLORE":
                if self.pose is not None and any(math.hypot(self.pose[0] - sx, self.pose[1] - sy) < 3.0
                                                 and time.time() < until for sx, sy, until in self.suppress):
                    return
                if ft < self.mission_t0_wall():
                    return
                self.confirm = {"t0": time.time(), "ok": 1, "bad": 0, "what": o.get("what", "")}
                self.prev_state = self.state
                self.state = "CONFIRM"
                self.get_logger().info(f"VLM candidate: {o.get('what')} (conf {o.get('conf')}); stopping to confirm")
            elif self.state == "CONFIRM" and self.confirm and ft > self.confirm["t0"] + 0.4:
                self.confirm["ok" if positive else "bad"] += 1
                self.confirm["t0_last"] = ft

    def mission_t0_wall(self):
        return time.time() - (time.monotonic() - self.mission_t0)

    def confirm_tick(self):
        c = self.confirm
        if c is None:
            self.state = "EXPLORE"
            return
        if c["ok"] >= self.confirm_needed:
            x, y = (self.pose[0], self.pose[1]) if self.pose else (0.0, 0.0)
            self.found.append({"x": round(x, 2), "y": round(y, 2), "what": c["what"], "t": round(time.monotonic() - self.mission_t0, 1)})
            self.say(f"*** TARGET FOUND: {self.mission['target']} near map ({x:.1f}, {y:.1f}) - {c['what']} ***")
            self.suppress.append((x, y, time.time() + 600))
            self.confirm = None
            if self.mission["then"] == "continue":
                self.set_state("EXPLORE")
            else:
                self.after_goal(True)
        elif c["bad"] >= 1 or time.time() - c["t0"] > 25.0:
            self.say("VLM could not confirm the candidate - resuming search")
            if self.pose:
                self.suppress.append((self.pose[0], self.pose[1], time.time() + 20))
            self.confirm = None
            self.set_state(self.prev_state)

    # ------------------------------------------------------------------ planner thread
    def planner_loop(self):
        last_stamp, last_t, last_state = -1, 0.0, None
        while self._run and rclpy.ok():
            time.sleep(0.1)
            while self.events:
                ev = self.events.popleft()
                if ev[0] == "reached":
                    self.explorer.note_goal_reached(ev[1], time.monotonic())
                elif ev[0] == "reset":
                    self.explorer.black.clear()
                    self.explorer.pending.clear()
                    self.explorer.prev_goal = None
                elif ev[0] == "black":
                    self.explorer.blacklist(ev[1], ev[2], time.monotonic(), ev[3])
            st, pose, msg, mission, bounds = self.state, self.pose, self.map_msg, self.mission, self.bounds
            if st not in MOTION or pose is None or msg is None or mission is None:
                last_state = None
                continue
            now = time.monotonic()
            if st == last_state and self.map_stamp == last_stamp and now - last_t < self.plan_period * 1.5:
                continue
            if now - last_t < 0.4:
                continue
            last_state, last_stamp, last_t = st, self.map_stamp, now
            try:
                view = build_view(np.asarray(msg.data, dtype=np.int8), msg.info.width, msg.info.height,
                                  msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y,
                                  plan_res=self.plan_res, inflation=self.inflation, bounds=bounds)
                if view.known_cells < 200:
                    continue
                rxy = (pose[0], pose[1])
                if st == "EXPLORE":
                    vdir = self.vlm_dir if time.monotonic() - self.vlm_dir_t < 8.0 else "unknown"
                    plan = self.explorer.plan_explore(view, rxy, pose[2], now, vdir)
                    if plan.kind == "none" and plan.info == "no reachable frontier":
                        if self.empty_cycles == 0:
                            self.empty_since = now
                        self.empty_cycles += 1
                        # finish only after several consecutive empty plans spanning >= 4 s (map had time to update)
                        if self.empty_cycles >= self.empty_needed and now - self.empty_since >= 4.0:
                            self.empty_cycles = 0
                            self.after_goal_flag = True
                        self.last_plan_info = f"no frontier ({self.empty_cycles}/{self.empty_needed})"
                        continue
                    if plan.kind != "none":
                        self.empty_cycles = 0
                elif st == "RETURN":
                    plan = self.explorer.plan_to(view, rxy, (self.entrance[0], self.entrance[1]))
                else:
                    plan = self.explorer.plan_to(view, rxy, tuple(mission["xy"]))
                self.last_plan_info = plan.info
                if plan.kind != "none":
                    with self.lock:
                        self._new_plan = plan
                else:
                    self.get_logger().warning(f"planner: {plan.info}", throttle_duration_sec=3.0)
            except Exception as exc:
                self.get_logger().error(f"planner error: {exc}", throttle_duration_sec=3.0)

    after_goal_flag = False
    events_state = ""

    # ------------------------------------------------------------------ control loop (20 Hz)
    def lookup_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            tr = t.transform.translation
            return float(tr.x), float(tr.y), yaw_of(t.transform.rotation)
        except Exception:
            return None

    def control(self):
        now = time.monotonic()
        pose = self.lookup_pose()
        if pose is not None:
            self.pose = pose
        if self.mission and now - self.mission_t0 > self.mission_timeout and self.state in ("EXPLORE", "GOTO", "CONFIRM"):
            self.confirm = None
            self.finish("mission timeout", "return")
        st = self.state
        if self.after_goal_flag and st == "EXPLORE":
            self.after_goal_flag = False
            self.after_goal(bool(self.found))
            return
        if st == "IDLE":
            return
        if st == "TAKEOFF":
            self.takeoff_tick(now)
            return
        if st == "LANDING":
            self.landing_tick()
            return
        if st == "CONFIRM":
            self.zero()
            self.tracker.reset_cmd()
            self.confirm_tick()
            return
        if st == "HOVER":
            self.zero()
            self.tracker.reset_cmd()
            return
        # ---- motion states
        if self.mission["type"] == "find" and now - self.last_query_pub > 3.0:
            self.query(self.mission["target"])
        if pose is None or self.scan is None or now - self.scan_t > 0.5 or self.fstate() != "flying":
            self.zero()
            self.tracker.reset_cmd()
            if self.fstate() != "flying" and self.fstate() != "unknown":
                self.get_logger().warning(f"flight state is '{self.fstate()}', not flying - motion paused", throttle_duration_sec=3.0)
            return
        with self.lock:
            if self._new_plan is not None:
                self.plan, self._new_plan = self._new_plan, None
                self.tracker.set_path(self.plan.waypoints, final=(st != "EXPLORE"))
                self.publish_path(self.plan, pose)
        plan = self.plan
        if plan is None:
            self.zero()
            return
        s = self.scan
        f, l, r, fl, fr = scan_clearances(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
        speed = self.search_speed if (self.mission["type"] == "find") else self.cruise
        vx, vy, wz, remaining = self.tracker.command(pose[0], pose[1], pose[2], 0.05, f, l, r, fl, fr, cruise=speed)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(cmd)
        gx, gy = plan.goal_xy
        dist_goal = math.hypot(gx - pose[0], gy - pose[1])
        if st == "EXPLORE":
            if dist_goal < 0.6 and self.goal_announced != plan.goal_xy:
                self.goal_announced = plan.goal_xy
                self.events.append(("reached", plan.goal_xy))
        elif dist_goal < 0.5 or remaining < 0.25:
            self.arrived()
            return
        self.stall_check(now, pose, plan, st)

    def stall_check(self, now, pose, plan, st):
        """Stalled = the drone has travelled < 0.4 m in total over 10 s (path length, not net displacement,
        so a legitimate detour into a dead end and back is NOT a stall)."""
        if self.hist:
            self.odo += math.hypot(pose[0] - self.hist[-1][1], pose[1] - self.hist[-1][2])
        self.hist.append((now, pose[0], pose[1], self.odo))
        while len(self.hist) > 1 and now - self.hist[1][0] >= 10.0:
            self.hist.popleft()
        if now - self.hist[0][0] >= 10.0 and self.odo - self.hist[0][3] < 0.4:
            self.get_logger().warning(f"stalled for 10 s near {plan.goal_xy}; giving up on that goal for 60 s")
            if st == "EXPLORE":
                self.events.append(("black", plan.goal_xy, 1.0, 60.0))
            self.clear_plan()
            self.hist.clear()

    def arrived(self):
        self.zero()
        self.clear_plan()
        m = self.mission
        if self.state == "RETURN":
            self.say("back at the entrance")
            self.begin_landing()
        elif self.state == "GOTO":
            self.say("arrived at the requested point")
            then = m["then"]
            self.finish("goto done", "return" if then == "return" else "land" if then == "land" else "hover")

    def takeoff_tick(self, now):
        self.zero()
        fs = self.fstate()
        if fs == "flying":
            self.say("airborne at the target altitude - starting mission")
            self.begin_motion()
            return
        if now > self.takeoff_deadline:
            self.say("TAKEOFF FAILED (timeout). Check /wingman/flight_state and the cmdvel_to_px4 log; mission cancelled")
            self.set_state("IDLE")
            return
        if fs == "unknown":
            self.get_logger().warning("no /wingman/flight_state yet - is the UPDATED cmdvel_to_px4 running?",
                                      throttle_duration_sec=3.0)
        if fs == "idle" and not self.takeoff_pending and now >= self.takeoff_next:
            if self.pose is not None:
                self.entrance = self.pose
            if not self.takeoff_cli.service_is_ready():
                self.get_logger().warning("/wingman/takeoff not available - is cmdvel_to_px4 running?", throttle_duration_sec=3.0)
                self.takeoff_next = now + 2.0
                return
            self.takeoff_pending = True
            fut = self.takeoff_cli.call_async(Trigger.Request())
            fut.add_done_callback(self.takeoff_done)
        elif fs == "error":
            self.say("cmdvel_to_px4 is in 'error' state; call /wingman/land or restart it")
            self.takeoff_deadline = min(self.takeoff_deadline, now + 5.0)

    def takeoff_done(self, fut):
        try:
            r = fut.result()
            if not r.success:
                self.get_logger().warning(f"takeoff refused: {r.message}  (retrying)", throttle_duration_sec=3.0)
                self.takeoff_next = time.monotonic() + 2.0
            else:
                self.get_logger().info(f"takeoff accepted: {r.message}")
                self.takeoff_next = time.monotonic() + 60.0
        except Exception as exc:
            self.get_logger().error(f"takeoff call failed: {exc}")
            self.takeoff_next = time.monotonic() + 2.0
        self.takeoff_pending = False

    def landing_tick(self):
        self.zero()
        if not self.land_requested:
            if self.fstate() == "idle":
                self.say("already on the ground - mission finished")
                self.set_state("IDLE")
                return
            if self.land_cli.service_is_ready():
                self.land_cli.call_async(Trigger.Request())
                self.land_requested = True
                self.say("landing...")
        elif self.fstate() == "idle":
            self.say("landed - MISSION COMPLETE" + (f"; targets: {self.found}" if self.found else ""))
            self.set_state("IDLE")

    # ------------------------------------------------------------------ outputs
    def publish_path(self, plan, pose):
        p = Path()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        for x, y in plan.waypoints:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), 1.0
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.path_pub.publish(p)
        mk = Marker()
        mk.header = p.header
        mk.ns, mk.id, mk.type, mk.action = "wingman_goal", 0, Marker.SPHERE, Marker.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(plan.goal_xy[0]), float(plan.goal_xy[1]), 1.0
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.4
        mk.color.g, mk.color.a = 1.0, 0.9
        self.marker_pub.publish(mk)

    def publish_status(self):
        p = self.pose
        s = {"state": self.state, "flight": self.fstate(), "alt_m": self.flight.get("alt_m"),
             "mission": {k: self.mission[k] for k in ("type", "target", "then")} if self.mission else None,
             "pose": [round(p[0], 2), round(p[1], 2), round(math.degrees(p[2]))] if p else None,
             "goal": [round(v, 2) for v in self.plan.goal_xy] if self.plan else None,
             "planner": self.last_plan_info, "vlm_ready": self.vlm_ready, "found": self.found,
             "elapsed_s": round(time.monotonic() - self.mission_t0) if self.mission else 0,
             "last": getattr(self, "last_say", "")}
        m = String()
        m.data = json.dumps(s)
        self.status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MissionAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._run = False
        try:
            node.zero()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
