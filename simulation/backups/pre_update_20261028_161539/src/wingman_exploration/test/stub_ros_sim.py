#!/usr/bin/env python3
"""Integration test WITHOUT ROS: runs the real mission_autopilot.py against stub rclpy + the maze simulator.
Fake PX4 bridge (takeoff/land services, flight_state) and a fake VLM (sees the target when it has line of sight).

  python3 test/stub_ros_sim.py --world ../../simulation/worlds/maze_exploration.sdf --mission "find the person and come back" --target 12 -10
"""
import argparse
import importlib
import json
import math
import sys
import threading
import time as _real_time
import types

import cv2
import numpy as np

sys.path.insert(0, ".")
from wingman_exploration.offline_sim import RES, lidar_update, load_world  # noqa: E402


# ----------------------------------------------------------------------------- stub ROS
class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class V3:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class Twist(Msg):
    def __init__(self):
        self.linear, self.angular = V3(), V3()


class String(Msg):
    def __init__(self, data=""):
        self.data = data


class Header(Msg):
    def __init__(self):
        self.frame_id, self.stamp = "", None


class Pose(Msg):
    def __init__(self):
        self.position, self.orientation = V3(), Msg(x=0.0, y=0.0, z=0.0, w=1.0)


class PoseStamped(Msg):
    def __init__(self):
        self.header, self.pose = Header(), Pose()


class Path(Msg):
    def __init__(self):
        self.header, self.poses = Header(), []


class Marker(Msg):
    SPHERE, ADD = 2, 0

    def __init__(self):
        self.header, self.pose = Header(), Pose()
        self.scale, self.color = V3(), Msg(r=0, g=0, b=0, a=0)


class Trigger:
    class Request:
        pass

    class Response:
        def __init__(self):
            self.success, self.message = False, ""


class Bus:
    def __init__(self):
        self.subs, self.services = {}, {}
        self.lock = threading.RLock()

    def publish(self, topic, msg):
        for cb in list(self.subs.get(topic, [])):
            cb(msg)


BUS = Bus()


class Logger:
    def __init__(self, name):
        self.name = name

    def _p(self, lvl, m, throttle_duration_sec=None, **_):
        if throttle_duration_sec is not None:
            key = (lvl, m[:40])
            now = _real_time.time()
            if now - Logger._last.get(key, 0) < throttle_duration_sec:
                return
            Logger._last[key] = now
        print(f"[{SIM.t:7.1f}] {lvl}: {m}", flush=True)

    _last = {}
    info = lambda self, m, **k: self._p("INFO", m, **k)
    warning = lambda self, m, **k: self._p("WARN", m, **k)
    error = lambda self, m, **k: self._p("ERROR", m, **k)


class Param:
    def __init__(self, v):
        self.value = v


class Future:
    def __init__(self):
        self._r, self._cbs = None, []

    def set(self, r):
        self._r = r
        for cb in self._cbs:
            cb(self)

    def result(self):
        return self._r

    def add_done_callback(self, cb):
        self._cbs.append(cb)


class Client:
    def __init__(self, name):
        self.name = name

    def service_is_ready(self):
        return self.name in BUS.services

    def call_async(self, req):
        f = Future()
        res = Trigger.Response()
        BUS.services[self.name](req, res)
        f.set(res)
        return f


class Pub:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, m):
        BUS.publish(self.topic, m)


class Node:
    def __init__(self, name):
        self.params, self.timers = {}, []

    def declare_parameter(self, k, v):
        self.params[k] = Param(v)

    def get_parameter(self, k):
        return self.params[k]

    def create_subscription(self, typ, topic, cb, qos):
        BUS.subs.setdefault(topic, []).append(cb)

    def create_publisher(self, typ, topic, qos):
        return Pub(topic)

    def create_timer(self, period, cb):
        self.timers.append([period, cb, 0.0])

    def create_service(self, typ, name, cb):
        BUS.services[name] = cb

    def create_client(self, typ, name):
        return Client(name)

    def get_logger(self):
        return Logger("n")

    def get_clock(self):
        return Msg(now=lambda: Msg(to_msg=lambda: None))

    def destroy_node(self):
        pass


def install_stubs():
    def mod(name, **kw):
        m = types.ModuleType(name)
        m.__dict__.update(kw)
        sys.modules[name] = m
        return m
    Enum = lambda **kw: type("E", (), kw)
    rclpy = mod("rclpy", ok=lambda: True, init=lambda args=None: None, shutdown=lambda: None, spin=lambda n: None)
    rclpy.time = mod("rclpy.time", Time=lambda: None)
    rclpy.node = mod("rclpy.node", Node=Node)
    rclpy.qos = mod("rclpy.qos", QoSProfile=lambda **k: None, ReliabilityPolicy=Enum(RELIABLE=1, BEST_EFFORT=2),
                    DurabilityPolicy=Enum(TRANSIENT_LOCAL=1), HistoryPolicy=Enum(KEEP_LAST=1))
    mod("geometry_msgs"); mod("geometry_msgs.msg", Twist=Twist, PoseStamped=PoseStamped)
    mod("nav_msgs"); mod("nav_msgs.msg", OccupancyGrid=Msg, Path=Path)
    mod("sensor_msgs"); mod("sensor_msgs.msg", LaserScan=Msg, Image=Msg)
    mod("std_msgs"); mod("std_msgs.msg", String=String)
    mod("std_srvs"); mod("std_srvs.srv", Trigger=Trigger)
    mod("visualization_msgs"); mod("visualization_msgs.msg", Marker=Marker)

    class Buffer:
        def lookup_transform(self, a, b, t):
            return Msg(transform=Msg(translation=Msg(x=SIM.x, y=SIM.y, z=1.0),
                                     rotation=Msg(x=0.0, y=0.0, z=math.sin(SIM.yaw / 2), w=math.cos(SIM.yaw / 2))))
    mod("tf2_ros", Buffer=Buffer, TransformListener=lambda b, n: None)


class Clock:     # replaces the `time` module inside mission_autopilot
    def time(self):
        return SIM.t

    def monotonic(self):
        return SIM.t

    def sleep(self, s):
        _real_time.sleep(s)


# ----------------------------------------------------------------------------- simulator
class Sim:
    pass


SIM = Sim()
SIM.t = 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--mission", default="explore the maze")
    ap.add_argument("--target", type=float, nargs=2, default=None, help="world x y of the 'person'")
    ap.add_argument("--rt", type=float, default=2.0, help="max speed vs real time (planner thread runs in real time)")
    ap.add_argument("--debug", type=float, default=0.0, help="print a trace line every N sim seconds")
    ap.add_argument("--max-time", type=float, default=1500.0)
    ap.add_argument("--start", type=float, nargs=3, default=[0.0, -17.2, math.pi / 2])
    a = ap.parse_args()

    install_stubs()
    ma = importlib.import_module("wingman_exploration.mission_autopilot")
    ma.time = Clock()
    gt, ox = load_world(a.world)
    gtdist = cv2.distanceTransform((~gt).astype(np.uint8) * 255, cv2.DIST_L2, 3) * RES
    known = np.full(gt.shape, -1, np.int8)
    SIM.x, SIM.y, SIM.yaw = a.start
    node = ma.MissionAutopilot()
    node.params["explore_box"] = Param([1.5, 32.9, -15.8, 15.8])

    # fake cmdvel_to_px4
    fp = {"state": "idle", "t": 0.0, "alt": 0.0}

    def takeoff(_r, res):
        if fp["state"] != "idle":
            res.success, res.message = False, f"state {fp['state']}"
        else:
            fp.update(state="climbing", t=SIM.t)
            res.success, res.message = True, "ok"
        return res

    def land(_r, res):
        fp.update(state="landing", t=SIM.t)
        res.success, res.message = True, "landing"
        return res
    BUS.services["/wingman/takeoff"], BUS.services["/wingman/land"] = takeoff, land
    cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "t": -9}

    def on_cmd(m):
        cmd.update(vx=m.linear.x, vy=m.linear.y, wz=m.angular.z, t=SIM.t)
    BUS.subs.setdefault("/cmd_vel", []).append(on_cmd)
    query = {"target": ""}
    BUS.subs.setdefault("/wingman/vlm_query", []).append(lambda m: query.update(target=m.data))
    status = {}
    BUS.subs.setdefault("/wingman/mission_status", []).append(lambda m: status.update(json.loads(m.data)))

    dt = 0.05
    vbx = vby = 0.0
    last_map, last_vlm, last_fs = -9, -9, -9
    pending_obs = None
    dist, coll, min_d = 0.0, 0, 9.0
    BUS.publish("/wingman/vlm/status", String(json.dumps({"ready": True})))
    node.say = node.say    # noqa
    started = False
    wall0 = _real_time.time()
    airborne_max = 0.0
    while SIM.t < a.max_time:
        loop_start = _real_time.time()
        rng = lidar_update(gt, known, ox, SIM.x, SIM.y, SIM.yaw)
        scan = Msg(ranges=list(rng), angle_min=-math.pi, angle_increment=2 * math.pi / 360, range_min=0.1, range_max=10.0)
        BUS.publish("/scan", scan)
        if SIM.t - last_map >= 1.0:
            last_map = SIM.t
            info = Msg(width=known.shape[1], height=known.shape[0], resolution=RES,
                       origin=Msg(position=Msg(x=ox, y=ox), orientation=Msg(x=0, y=0, z=0, w=1)))
            BUS.publish("/map", Msg(info=info, data=known.ravel().copy(), header=Msg(frame_id="map")))
        # fake PX4 bridge
        if fp["state"] == "climbing":
            fp["alt"] = min(1.0, (SIM.t - fp["t"]) * 0.5)
            if fp["alt"] >= 1.0:
                fp["state"] = "flying"
        elif fp["state"] == "landing":
            fp["alt"] = max(0.0, 1.0 - (SIM.t - fp["t"]) * 0.4)
            if fp["alt"] <= 0.0:
                fp["state"] = "idle"
        if SIM.t - last_fs >= 0.2:
            last_fs = SIM.t
            BUS.publish("/wingman/flight_state", String(json.dumps({"state": fp["state"], "alt_m": fp["alt"], "ready": True})))
        airborne_max = max(airborne_max, fp["alt"])
        if fp["state"] == "flying" and SIM.t - cmd["t"] < 0.5:
            tx, ty, wz = cmd["vx"], cmd["vy"], cmd["wz"]
        else:
            tx = ty = wz = 0.0
        vbx += (tx - vbx) * min(1.0, dt / 0.25)
        vby += (ty - vby) * min(1.0, dt / 0.25)
        SIM.yaw += wz * dt
        nx = SIM.x + (math.cos(SIM.yaw) * vbx - math.sin(SIM.yaw) * vby) * dt
        ny = SIM.y + (math.sin(SIM.yaw) * vbx + math.cos(SIM.yaw) * vby) * dt
        wd = float(gtdist[int((ny - ox) / RES), int((nx - ox) / RES)])
        if fp["alt"] > 0.5:
            min_d = min(min_d, wd)
            coll += wd < 0.25
        dist += math.hypot(nx - SIM.x, ny - SIM.y)
        SIM.x, SIM.y = nx, ny
        # fake VLM: ~2.5 s latency, sees the person within 5 m with line of sight
        if query["target"] and a.target and SIM.t - last_vlm >= 2.5 and pending_obs is None:
            last_vlm = SIM.t
            d = math.hypot(a.target[0] - SIM.x, a.target[1] - SIM.y)
            ang = math.atan2(a.target[1] - SIM.y, a.target[0] - SIM.x) - SIM.yaw
            clear = True
            for k in range(1, int(d / 0.05)):
                px, py = SIM.x + (a.target[0] - SIM.x) * k * 0.05 / d, SIM.y + (a.target[1] - SIM.y) * k * 0.05 / d
                if gt[int((py - ox) / RES), int((px - ox) / RES)]:
                    clear = False
                    break
            seen = d < 5.0 and clear and abs(math.atan2(math.sin(ang), math.cos(ang))) < math.radians(35)
            pending_obs = (SIM.t + 2.5, {"ok": True, "target": query["target"], "seen": bool(seen), "conf": 0.85 if seen else 0.1,
                                         "dir": "forward", "what": "person in corridor" if seen else "empty corridor",
                                         "frame_time": SIM.t, "latency_s": 2.5})
        if pending_obs and SIM.t >= pending_obs[0]:
            BUS.publish("/wingman/vlm/observation", String(json.dumps(pending_obs[1])))
            pending_obs = None
        # mission
        if not started and SIM.t > 1.0:
            started = True
            BUS.publish("/wingman/mission_command", String(a.mission))
        for tm in node.timers:
            if SIM.t - tm[2] >= tm[0] - 1e-9:
                tm[2] = SIM.t
                tm[1]()
        if a.debug and abs((SIM.t % a.debug)) < dt:
            pl = node.plan
            print(f"   trace t={SIM.t:6.1f} st={node.state:7} pos=({SIM.x:6.2f},{SIM.y:6.2f}) yaw={math.degrees(SIM.yaw):5.0f} "
                  f"cmd=({cmd['vx']:.2f},{cmd['vy']:.2f},{cmd['wz']:.2f}) goal={pl.goal_xy if pl else None} "
                  f"wps={len(pl.waypoints) if pl else 0} info={node.last_plan_info} black={len(node.explorer.black)}", flush=True)
        SIM.t += dt
        if started and node.state == "IDLE" and fp["state"] == "idle" and SIM.t > 20 and airborne_max > 0.9:
            break
        if a.rt > 0:
            sleep = dt / a.rt - (_real_time.time() - loop_start)
            if sleep > 0:
                _real_time.sleep(sleep)
    node._run = False
    free_gt = np.zeros_like(gt)
    i0, i1 = int((-16 - ox) / RES), int((16 - ox) / RES)
    free_gt[i0:i1, i0:i1] = True
    free_gt &= ~gt
    cov = float(((known == 0) & free_gt).sum()) / float(free_gt.sum())
    print(f"\nRESULT sim_time={SIM.t:.0f}s wall={_real_time.time()-wall0:.0f}s flown={dist:.0f}m coverage={cov*100:.1f}% "
          f"collision_ticks={coll} min_wall_dist={min_d:.2f}m final_node_state={node.state} flight={fp['state']} "
          f"home_dist={math.hypot(SIM.x-a.start[0], SIM.y-a.start[1]):.2f}m found={node.found}")


if __name__ == "__main__":
    main()
