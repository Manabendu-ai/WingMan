#!/usr/bin/env python3
"""WingMan mission console: type a mission in plain English, the drone does it.

  explore the maze                       find the person and come back
  find a red box and stay there          find the person and keep exploring
  go to 4 -10   (map metres)             return home
  land            stop (hover)           :ask is there a door on the left?   (one-off VLM question)
  :status                                :quit
"""
import json
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class Console(Node):
    def __init__(self):
        super().__init__("wingman_console")
        self.cmd = self.create_publisher(String, "/wingman/mission_command", 10)
        self.ask = self.create_publisher(String, "/wingman/vlm_prompt", 10)
        self.create_subscription(String, "/wingman/mission_status", self.on_status, 10)
        self.create_subscription(String, "/wingman/vlm/observation", self.on_vlm, 10)
        self.status, self.last_say = {}, ""

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            return
        say = self.status.get("last", "")
        if say and say != self.last_say:
            self.last_say = say
            print(f"\n[drone] {say}\nmission> ", end="", flush=True)

    def on_vlm(self, msg):
        try:
            o = json.loads(msg.data)
        except Exception:
            return
        if "answer" in o:
            print(f"\n[vlm {o.get('latency_s')}s] {o['answer']}\nmission> ", end="", flush=True)
        elif not o.get("ok"):
            print(f"\n[vlm error] {o.get('error')}\nmission> ", end="", flush=True)

    def send(self, text):
        m = String()
        m.data = text
        self.cmd.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = Console()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    print(__doc__)
    try:
        while rclpy.ok():
            try:
                line = input("mission> ").strip()
            except EOFError:
                break
            if not line:
                continue
            if line in (":quit", ":q"):
                break
            if line == ":status":
                print(json.dumps(node.status, indent=2))
            elif line.startswith(":ask "):
                m = String()
                m.data = line[5:]
                node.ask.publish(m)
            else:
                node.send(line)
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
