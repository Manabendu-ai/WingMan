#!/usr/bin/env python3
"""Website <-> drone gateway (runs next to rosbridge_websocket).

IN   /wingman/llm_command  (std_msgs/String)  JSON (or plain text) from the website / its LLM
       {"id":"42","type":"explore","then":"return","altitude":0.75}
       {"id":"43","prompt":"explore the maze and return home while maintaining a height of 0.75 m"}
     It is validated, clamped (altitude 0.4-2.0 m) and forwarded to /wingman/mission_command.
OUT  /wingman/llm_command_ack (std_msgs/String) JSON {"id","accepted","error","mission"}  (one per command)
OUT  /wingman/telemetry       (std_msgs/String) JSON, 2 Hz: flight state, real height from the rangefinder,
                                                target height, mission status. Small, so it is cheap to stream.
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from wingman_exploration.mission_parser import normalize_command


class WebGateway(Node):
    def __init__(self):
        super().__init__("wingman_web_gateway")
        self.declare_parameter("telemetry_hz", 2.0)
        self.declare_parameter("dedupe_s", 5.0)
        self.dedupe_s = float(self.get_parameter("dedupe_s").value)
        self.flight, self.flight_t = {}, 0.0
        self.mission, self.mission_t = {}, 0.0
        self.recent = {}                      # id -> wall time (duplicate suppression, e.g. a retried websocket send)
        self.mission_pub = self.create_publisher(String, "/wingman/mission_command", 10)
        self.ack_pub = self.create_publisher(String, "/wingman/llm_command_ack", 10)
        self.tel_pub = self.create_publisher(String, "/wingman/telemetry", 10)
        self.create_subscription(String, "/wingman/llm_command", self.on_command, 10)
        self.create_subscription(String, "/wingman/flight_state", self.on_flight, 10)
        self.create_subscription(String, "/wingman/mission_status", self.on_mission, 10)
        self.create_timer(1.0 / max(0.2, float(self.get_parameter("telemetry_hz").value)), self.publish_telemetry)
        self.get_logger().info("web gateway ready: /wingman/llm_command -> /wingman/mission_command, "
                               "telemetry on /wingman/telemetry")

    def on_flight(self, msg):
        try:
            self.flight, self.flight_t = json.loads(msg.data), time.monotonic()
        except Exception:
            pass

    def on_mission(self, msg):
        try:
            self.mission, self.mission_t = json.loads(msg.data), time.monotonic()
        except Exception:
            pass

    def ack(self, cid, ok, err="", mission=None):
        m = String()
        m.data = json.dumps({"id": cid, "accepted": ok, "error": err, "mission": mission})
        self.ack_pub.publish(m)

    def on_command(self, msg):
        cid = None
        try:
            if msg.data.lstrip().startswith("{"):
                cid = json.loads(msg.data).get("id")
        except Exception:
            pass
        now = time.monotonic()
        self.recent = {k: t for k, t in self.recent.items() if now - t < self.dedupe_s}
        if cid is not None and str(cid) in self.recent:
            self.get_logger().info(f"duplicate command id {cid} ignored")
            return
        canon, mission, err = normalize_command(msg.data)
        if canon is None:
            self.get_logger().warning(f"command rejected: {err}")
            self.ack(cid, False, err)
            return
        if cid is not None:
            self.recent[str(cid)] = now
        out = String()
        out.data = canon
        self.mission_pub.publish(out)
        self.get_logger().info(f"command accepted: {canon}")
        self.ack(cid, True, "", json.loads(canon))

    def publish_telemetry(self):
        now = time.monotonic()
        f, m = self.flight, self.mission
        t = {"t": round(time.time(), 2),
             "flight_state": f.get("state", "unknown") if now - self.flight_t < 2.0 else "unknown",
             "height_m": f.get("alt_m"),               # from the downward rangefinder when range_ok
             "target_height_m": f.get("target_alt_m"),
             "range_ok": f.get("range_ok"),
             "baro_height_m": f.get("baro_alt_m"),
             "ready": f.get("ready"),
             "mission_state": m.get("state"),
             "mission": m.get("mission"),
             "pose": m.get("pose"), "goal": m.get("goal"),
             "found": m.get("found"), "elapsed_s": m.get("elapsed_s"),
             "last": m.get("last"),
             "stale": {"flight": now - self.flight_t > 2.0, "mission": now - self.mission_t > 2.0}}
        out = String()
        out.data = json.dumps(t)
        self.tel_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WebGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
