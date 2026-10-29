#!/usr/bin/env python3
"""Save frames from a ROS 2 sensor_msgs/Image topic as PNG test images for vlm_bench.py.
  python3 capture_frames.py --topic <image topic> --count 20 --every 2.0 --out ~/ws_wingman/vlm/test_images"""
import argparse
import os
import time

import numpy as np
import rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

CHANNELS = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}


def to_pil(msg):
    ch = CHANNELS.get(msg.encoding)
    if ch is None:
        raise ValueError(f'unsupported encoding {msg.encoding}')
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    arr = buf[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    if msg.encoding == 'bgr8':
        arr = arr[:, :, ::-1]
    elif msg.encoding == 'bgra8':
        arr = arr[:, :, [2, 1, 0]]
    elif msg.encoding == 'rgba8':
        arr = arr[:, :, :3]
    elif msg.encoding == 'mono8':
        arr = np.repeat(arr, 3, axis=2)
    return PILImage.fromarray(np.ascontiguousarray(arr), 'RGB')


class Capture(Node):
    def __init__(self, a):
        super().__init__('capture_frames')
        self.a, self.n, self.last = a, 0, 0.0
        os.makedirs(a.out, exist_ok=True)
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, a.topic, self.cb, q)
        self.get_logger().info(f'waiting for images on {a.topic}')

    def cb(self, msg):
        if self.n >= self.a.count or time.monotonic() - self.last < self.a.every:
            return
        self.last = time.monotonic()
        self.n += 1
        path = os.path.join(self.a.out, f'frame_{self.n:04d}.png')
        to_pil(msg).save(path)
        self.get_logger().info(f'saved {path} ({msg.width}x{msg.height} {msg.encoding})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', required=True)
    ap.add_argument('--count', type=int, default=20)
    ap.add_argument('--every', type=float, default=2.0)
    ap.add_argument('--out', default='~/ws_wingman/vlm/test_images')
    a = ap.parse_args()
    a.out = os.path.expanduser(a.out)
    rclpy.init()
    node = Capture(a)
    try:
        while rclpy.ok() and node.n < a.count:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
