#!/usr/bin/env python3
"""Serial VLM mission-decision loop. Publishes decisions, never flight commands."""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


VALID_ACTIONS = {"explore", "return_to_entrance", "hold"}
VALID_DIRECTIONS = {"left", "forward", "right", "unknown"}


class VlmMissionManager(Node):
    def __init__(self):
        super().__init__("wingman_vlm_mission_manager")
        self.declare_parameter("request_interval_sec", 15.0)
        self.declare_parameter("request_timeout_sec", 120.0)
        self.declare_parameter("min_confidence", 0.45)
        self.declare_parameter("mission_id", "wingman_maze_search_v1")
        self.request_interval = max(5.0, float(self.get_parameter("request_interval_sec").value))
        self.request_timeout = max(30.0, float(self.get_parameter("request_timeout_sec").value))
        self.min_confidence = min(1.0, max(0.0, float(self.get_parameter("min_confidence").value)))
        self.mission_id = str(self.get_parameter("mission_id").value)

        self.prompt_pub = self.create_publisher(String, "/wingman/vlm_prompt", 10)
        self.mission_pub = self.create_publisher(String, "/wingman/mission_json", 10)
        self.decision_pub = self.create_publisher(String, "/wingman/vlm/mission_decision", 10)
        self.observation_sub = self.create_subscription(
            String, "/wingman/vlm/observation", self.on_observation, 10
        )
        self.pending = False
        self.request_started = 0.0
        self.next_request_at = time.monotonic() + 2.0
        self.last_action = "hold"
        self.last_direction = "unknown"
        self.timer = self.create_timer(1.0, self.tick)
        self.publish_decision("hold", "unknown", None, None,
                              "Initial state: waiting for first valid VLM observation")
        self.get_logger().info(
            "Mission manager ready. It requests one VLM inference at a time and publishes "
            "/wingman/vlm/mission_decision. It never commands /cmd_vel or takeoff."
        )

    def publish_decision(self, action, direction, person_detected, confidence, reason):
        if action not in VALID_ACTIONS:
            action = "hold"
        if direction not in VALID_DIRECTIONS:
            direction = "unknown"
        data = {
            "mission_id": self.mission_id,
            "mission_action": action,
            "preferred_direction": direction,
            "person_detected": person_detected if person_detected in (True, False, "unknown") else "unknown",
            "confidence": confidence if isinstance(confidence, (int, float)) and 0 <= confidence <= 1 else None,
            "reason": str(reason)[:500],
            "stamp_unix_sec": time.time(),
            "source": "gemma3:4b_vlm_mission_manager",
            "direct_flight_command": False,
        }
        msg = String()
        msg.data = json.dumps(data, separators=(",", ":"))
        self.decision_pub.publish(msg)
        self.last_action = action
        self.last_direction = direction
        self.get_logger().info(
            f"VLM mission decision: action={action}, preferred_direction={direction}, "
            f"person_detected={data['person_detected']}; {reason}"
        )

    def tick(self):
        now = time.monotonic()
        if self.pending:
            if now - self.request_started > self.request_timeout:
                self.pending = False
                self.next_request_at = now + self.request_interval
                self.publish_decision("hold", "unknown", "unknown", None,
                                      "VLM response timed out; fail-closed to hold")
            return
        if now < self.next_request_at:
            return

        mission = {
            "mission_id": self.mission_id,
            "objective": "Explore all reachable maze corridors, identify and report people if visibly present, "
                          "avoid obstacles, and return to the recorded entrance when exploration is exhausted.",
            "decision_policy": "Choose only a high-level mission action and an optional visually supported branch direction.",
            "actions": ["explore", "return_to_entrance", "hold"],
            "directions": ["left", "forward", "right", "unknown"],
            "constraints": [
                "Never invent coordinates or distances.",
                "Unknown direction is preferred to guessing.",
                "A person detection is not a location estimate; do not infer a route to the person.",
                "Never output velocity, attitude, motor, MAVLink, PX4, or takeoff commands."
            ]
        }
        mission_msg = String()
        mission_msg.data = json.dumps(mission, separators=(",", ":"))
        prompt_msg = String()
        prompt_msg.data = (
            "Analyze the newest camera frame and map context for the current maze mission. "
            "Return the exact required JSON schema, including mission_action and preferred_direction. "
            "Choose mission_action='explore' if the visible passage is navigable and there is no strong reason to stop. "
            "Choose preferred_direction only when a left, forward, or right opening is visually supported; otherwise unknown. "
            "Choose hold if image quality or evidence is too poor to advise. Never guess distances or coordinates. "
            "A person_detected result is visual only and does not locate the person in the map. "
            "Do not output or imply direct flight-control commands."
        )
        self.mission_pub.publish(mission_msg)
        self.prompt_pub.publish(prompt_msg)
        self.pending = True
        self.request_started = now
        self.next_request_at = now + self.request_interval
        self.get_logger().info("Requested VLM decision from latest available camera/map/odometry context")

    @staticmethod
    def _normalise_observation(obj):
        action = obj.get("mission_action")
        direction = obj.get("preferred_direction", "unknown")
        recommendation = str(obj.get("recommended_next_step", "")).lower()
        inferred = False
        if action not in VALID_ACTIONS:
            # Conservative compatibility fallback for models that omit the new key.
            if "return to entrance" in recommendation or "return_to_entrance" in recommendation:
                action = "return_to_entrance"
                inferred = True
            elif obj.get("ok") is True and obj.get("scene") is not None:
                action = "explore"
                inferred = True
            else:
                action = "hold"
        if direction not in VALID_DIRECTIONS:
            direction = "unknown"
        pd = obj.get("person_detected", "unknown")
        if pd not in (True, False, "unknown"):
            pd = "unknown"
        confidence = obj.get("confidence")
        if not isinstance(confidence, (int, float)) or not 0 <= confidence <= 1:
            confidence = None
        return action, direction, pd, confidence, inferred

    def on_observation(self, msg):
        try:
            obj = json.loads(msg.data)
            if not isinstance(obj, dict):
                raise ValueError("observation is not a JSON object")
        except Exception as exc:
            self.pending = False
            self.next_request_at = time.monotonic() + self.request_interval
            self.publish_decision("hold", "unknown", "unknown", None,
                                  f"Invalid VLM observation JSON: {exc}")
            return

        self.pending = False
        self.next_request_at = time.monotonic() + self.request_interval
        if not obj.get("ok", False):
            self.publish_decision("hold", "unknown", "unknown", None,
                                  "VLM inference failed; fail-closed to hold: " + str(obj.get("error", "unknown error")))
            return

        action, direction, person, confidence, inferred = self._normalise_observation(obj)
        if confidence is not None and confidence < self.min_confidence:
            action = "hold"
        reason = str(obj.get("visual_summary", obj.get("recommended_next_step", "VLM observation accepted")))
        if inferred:
            reason = "Action inferred from older VLM schema. " + reason
        if person is True:
            reason += " Person appears visible; image alone does not provide a map location."
        self.publish_decision(action, direction, person, confidence, reason)


def main(args=None):
    rclpy.init(args=args)
    node = VlmMissionManager()
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
