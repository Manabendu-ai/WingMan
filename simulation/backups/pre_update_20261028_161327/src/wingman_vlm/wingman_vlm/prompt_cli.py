#!/usr/bin/env python3
"""Interactive terminal for sending prompts and mission JSON to WingMan VLM."""
import json
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class PromptCLI(Node):
    def __init__(self):
        super().__init__("wingman_vlm_prompt_cli")
        self.prompt_pub = self.create_publisher(String, "/wingman/vlm_prompt", 10)
        self.mission_pub = self.create_publisher(String, "/wingman/mission_json", 10)
        self.output_sub = self.create_subscription(String, "/wingman/vlm/observation", self.on_result, 10)
        self.get_logger().info("Interactive prompt terminal ready")

    def on_result(self, msg):
        try:
            data = json.loads(msg.data)
            print("\n--- WingMan VLM result ---")
            print(json.dumps(data, indent=2, ensure_ascii=False))
            print("--------------------------\n", flush=True)
        except Exception:
            print(f"\nVLM result: {msg.data}\n", flush=True)

    def send_prompt(self, text):
        msg = String()
        msg.data = text
        self.prompt_pub.publish(msg)
        print("Prompt sent to /wingman/vlm_prompt", flush=True)

    def send_mission(self, text):
        try:
            parsed = json.loads(text)
            if not isinstance(parsed, dict):
                raise ValueError("mission must be a JSON object")
        except Exception as exc:
            print(f"Invalid JSON: {exc}", flush=True)
            return
        msg = String()
        msg.data = json.dumps(parsed, ensure_ascii=False)
        self.mission_pub.publish(msg)
        print("Mission JSON published to /wingman/mission_json", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = PromptCLI()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("\nWingMan VLM prompt terminal")
    print("Type a visual question and press Enter to analyze the latest camera frame.")
    print("Use :json {\"mission_id\":\"maze_001\",\"command\":\"explore\"} to submit mission JSON.")
    print("Use :help for examples; :quit to exit. VLM output is advisory-only.\n")
    try:
        while rclpy.ok():
            try:
                line = input("WingMan prompt> ").strip()
            except EOFError:
                break
            if not line:
                continue
            if line == ":quit":
                break
            if line == ":help":
                print("Examples:\n  Describe the visible corridor and openings.\n  Is a person clearly visible? State uncertainty.\n  Identify visible obstacles; do not guess distances.\n  :json {\"mission_id\":\"maze_001\",\"command\":\"explore\",\"objective\":\"map maze and find people\"}\n  :quit\n", flush=True)
                continue
            if line.startswith(":json "):
                node.send_mission(line[6:].strip())
                continue
            node.send_prompt(line)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()