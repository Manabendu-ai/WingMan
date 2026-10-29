#!/usr/bin/env python3
"""WingMan VLM node (Ollama) - fast, non-blocking, advisory only.

Why the old version was slow
  * every request sent 2 images (camera + map render) + ~700 tokens of JSON context
  * 160 generated tokens of free-form JSON at ~10 tok/s on a 4B model
  * the model was cold-loaded on the first request (57 s in your benchmark)
What this version does
  * warms the model at start-up (status ready=false -> true) so the first real call is fast
  * ONE image, ~60-token prompt, schema-constrained reply of ~20 tokens  -> ~2-4 s instead of 15-120 s
  * runs continuously ONLY while a target query is set (/wingman/vlm_query); idle otherwise
  * never gates flight: the autopilot flies on LiDAR/map and uses this node only to confirm targets
Topics
  in : camera_topic (Image), /wingman/vlm_query (String target text, "" = stop),
       /wingman/vlm_prompt (String free-form question, answered once)
  out: /wingman/vlm/observation (String JSON), /wingman/vlm/status (String JSON, 1 Hz)
"""
import base64
import json
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

DEFAULT_CAMERA = "/world/maze_exploration/model/x500_gps_denied_0/link/camera_link/sensor/IMX214/image"

TARGET_SCHEMA = {
    "type": "object",
    "properties": {
        "seen": {"type": "boolean"},
        "conf": {"type": "number"},
        "dir": {"type": "string", "enum": ["left", "forward", "right", "none"]},
        "what": {"type": "string"},
    },
    "required": ["seen", "conf", "dir", "what"],
}
ANSWER_SCHEMA = {"type": "object", "properties": {"answer": {"type": "string"}}, "required": ["answer"]}


class WingmanVLM(Node):
    def __init__(self):
        super().__init__("wingman_vlm")
        d = self.declare_parameter
        d("model", "gemma3:4b")
        d("ollama_url", "http://127.0.0.1:11434/api/chat")
        d("camera_topic", DEFAULT_CAMERA)
        d("query_topic", "/wingman/vlm_query")
        d("prompt_topic", "/wingman/vlm_prompt")
        d("output_topic", "/wingman/vlm/observation")
        d("status_topic", "/wingman/vlm/status")
        d("max_image_width", 640)
        d("jpeg_quality", 80)
        d("max_camera_age_sec", 1.5)
        d("http_timeout_sec", 60.0)
        d("num_predict", 48)
        d("num_ctx", 2048)
        d("keep_alive", "24h")
        d("warmup", True)
        g = lambda k: self.get_parameter(k).value
        self.model, self.url = str(g("model")), str(g("ollama_url"))
        self.camera_topic = str(g("camera_topic"))
        self.max_w, self.q = int(g("max_image_width")), int(g("jpeg_quality"))
        self.max_age = float(g("max_camera_age_sec"))
        self.timeout = float(g("http_timeout_sec"))
        self.num_predict, self.num_ctx = int(g("num_predict")), int(g("num_ctx"))
        self.keep_alive = str(g("keep_alive"))

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None           # latest Image msg
        self.frame_wall = 0.0       # time.time() when received
        self.target = ""            # active target query
        self.prompts = []           # queued free-form prompts
        self.ready = False
        self.busy = False
        self.last_latency = None
        self.last_frame_used = 0.0
        self.count = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, self.camera_topic, self.on_image, qos)
        self.create_subscription(String, str(g("query_topic")), self.on_query, 10)
        self.create_subscription(String, str(g("prompt_topic")), self.on_prompt, 10)
        self.out = self.create_publisher(String, str(g("output_topic")), 10)
        self.status_pub = self.create_publisher(String, str(g("status_topic")), 10)
        self.create_timer(1.0, self.publish_status)
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(f"VLM node up: model={self.model} camera={self.camera_topic} (warming up...)")

    # ------------------------------------------------------------------ io
    def on_image(self, msg):
        self.frame, self.frame_wall = msg, time.time()      # no encoding here: keeps the callback ~free

    def on_query(self, msg):
        t = msg.data.strip()
        with self.lock:
            changed = t != self.target
            self.target = t
        if changed:
            self.get_logger().info(f"target query = {t!r}" if t else "target query cleared (VLM idle)")

    def on_prompt(self, msg):
        if msg.data.strip():
            with self.lock:
                self.prompts.append(msg.data.strip())

    def publish(self, obj):
        m = String()
        m.data = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
        self.out.publish(m)

    def publish_status(self):
        m = String()
        m.data = json.dumps({"ready": self.ready, "busy": self.busy, "model": self.model, "target": self.target,
                             "last_latency_s": self.last_latency, "inferences": self.count})
        self.status_pub.publish(m)

    # ------------------------------------------------------------------ inference
    def encode(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        if w > self.max_w:
            frame = cv2.resize(frame, (self.max_w, int(h * self.max_w / w)), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.q])
        if not ok:
            raise RuntimeError("jpeg encode failed")
        return base64.b64encode(buf.tobytes()).decode("ascii")

    use_schema = True

    def chat(self, prompt, image_b64, schema, num_predict=None):
        body = {
            "model": self.model, "stream": False, "format": schema if self.use_schema else "json",
            "keep_alive": self.keep_alive,
            "messages": [{"role": "user", "content": prompt, "images": [image_b64]}],
            "options": {"temperature": 0, "num_predict": num_predict or self.num_predict, "num_ctx": self.num_ctx},
        }
        req = urllib.request.Request(self.url, data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                data = json.loads(r.read().decode())
        except urllib.error.HTTPError as exc:
            if exc.code == 400 and self.use_schema:      # old Ollama without JSON-schema output
                self.use_schema = False
                self.get_logger().warning("Ollama rejected the JSON schema; falling back to plain JSON mode")
                return self.chat(prompt, image_b64, schema, num_predict)
            raise
        text = data.get("message", {}).get("content", "")
        if not text:
            raise RuntimeError("empty Ollama reply")
        return json.loads(text), data

    def warmup(self):
        try:
            img = np.full((96, 96, 3), 127, np.uint8)
            _, buf = cv2.imencode(".jpg", img)
            t0 = time.time()
            self.chat("Is a person visible?", base64.b64encode(buf.tobytes()).decode(), TARGET_SCHEMA, 8)
            self.get_logger().info(f"VLM warm-up done in {time.time() - t0:.1f}s - ready")
            self.ready = True
        except Exception as exc:
            self.get_logger().error(f"VLM warm-up failed ({exc}). Is `ollama serve` running and `{self.model}` pulled?")
            self.ready = True   # still try real calls; errors are reported on the observation topic

    def fresh_frame(self):
        if self.frame is None or time.time() - self.frame_wall > self.max_age:
            return None, 0.0
        return self.frame, self.frame_wall

    def worker(self):
        if bool(self.get_parameter("warmup").value):
            self.warmup()
        else:
            self.ready = True
        while rclpy.ok():
            with self.lock:
                prompt = self.prompts.pop(0) if self.prompts else None
                target = self.target
            if prompt is None and not target:
                time.sleep(0.1)
                continue
            msg, fw = self.fresh_frame()
            if msg is None or (prompt is None and fw <= self.last_frame_used):
                if msg is None and (prompt or target):
                    self.get_logger().warning("no fresh camera frame (check the camera bridge)", throttle_duration_sec=5.0)
                time.sleep(0.05)
                if prompt is not None:
                    with self.lock:
                        self.prompts.insert(0, prompt)
                continue
            self.busy = True
            t0 = time.time()
            try:
                b64 = self.encode(msg)
                if prompt is not None:
                    out, raw = self.chat(f"You are a drone's front camera. {prompt} Answer briefly.", b64, ANSWER_SCHEMA, 80)
                    res = {"ok": True, "prompt": prompt, **out}
                else:
                    q = (f"Drone front camera. Is {target} clearly visible? seen=true only if you are sure. "
                         "conf is 0-1. dir is the open passage direction (left, forward, right or none). "
                         "what is at most 8 words.")
                    out, raw = self.chat(q, b64, TARGET_SCHEMA)
                    self.last_frame_used = fw
                    conf = out.get("conf", 0.0)
                    conf = float(conf) if isinstance(conf, (int, float)) else 0.0
                    res = {"ok": True, "target": target, "seen": bool(out.get("seen", False)),
                           "conf": max(0.0, min(1.0, conf)), "dir": out.get("dir", "none"),
                           "what": str(out.get("what", ""))[:80]}
                    res["person_detected"] = res["seen"]
                res.update({"frame_time": fw, "latency_s": round(time.time() - t0, 2), "model": self.model,
                            "eval_tokens": raw.get("eval_count"), "stamp": time.time()})
                self.last_latency = res["latency_s"]
                self.count += 1
                self.publish(res)
            except urllib.error.HTTPError as exc:
                self.publish({"ok": False, "error": f"Ollama HTTP {exc.code}: {exc.read().decode(errors='replace')[:300]}"})
                time.sleep(1.0)
            except Exception as exc:
                self.publish({"ok": False, "error": str(exc), "stamp": time.time()})
                self.get_logger().error(f"VLM inference failed: {exc}", throttle_duration_sec=5.0)
                time.sleep(1.0)
            finally:
                self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = WingmanVLM()
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
