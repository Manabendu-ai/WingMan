"""Turn a VLM target pixel + depth image + drone pose into a 3D world goal.

Assumes RGB and depth are aligned (same field of view), and that the
intrinsics K belong to the DEPTH image resolution.

Frames:
  camera (optical): x right, y down, z forward
  body   (ROS):     x forward, y left, z up
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class Pose:
    """Drone pose in the world frame (position + quaternion)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


@dataclass
class Goal:
    ok: bool
    reason: str = ""
    world: Optional[np.ndarray] = None   # (x, y, z) in world frame
    range_m: float = 0.0                 # distance camera -> target


def quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ])


def sample_depth(depth, u, v, win=5, depth_scale=1.0,
                 min_d=0.3, max_d=8.0) -> Optional[float]:
    """Median of valid depths around (u, v). None if too few valid pixels."""
    h, w = depth.shape[:2]
    u0, v0 = int(round(u)), int(round(v))
    patch = depth[max(0, v0-win):min(h, v0+win+1),
                  max(0, u0-win):min(w, u0+win+1)].astype(float) * depth_scale
    valid = patch[np.isfinite(patch) & (patch > min_d) & (patch < max_d)]
    if valid.size < max(3, 0.2 * patch.size):
        return None
    return float(np.median(valid))


def project_goal(target_x, target_y, depth, K: Intrinsics, pose: Pose,
                 depth_scale=1.0, standoff_m=0.5) -> Goal:
    """target_x/y are the VLM's relative coordinates (0-1000)."""
    h, w = depth.shape[:2]
    u = target_x / 1000.0 * (w - 1)
    v = target_y / 1000.0 * (h - 1)

    z = sample_depth(depth, u, v, depth_scale=depth_scale)
    if z is None:
        return Goal(False, "no valid depth at target pixel")

    # pixel -> camera frame
    p_cam = np.array([(u - K.cx) * z / K.fx, (v - K.cy) * z / K.fy, z])
    # camera optical -> body (ROS)
    p_body = np.array([p_cam[2], -p_cam[0], -p_cam[1]])

    # stop short of the target instead of aiming at the surface itself
    dist = float(np.linalg.norm(p_body))
    if dist <= standoff_m:
        return Goal(False, "target already within standoff distance", range_m=dist)
    p_body *= (dist - standoff_m) / dist

    # body -> world
    R = quat_to_rot(pose.qx, pose.qy, pose.qz, pose.qw)
    world = R @ p_body + np.array([pose.x, pose.y, pose.z])
    return Goal(True, "ok", world, dist)
