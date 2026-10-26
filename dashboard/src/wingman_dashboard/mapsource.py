"""Map sources. snapshot() mirrors a ROS nav_msgs/OccupancyGrid so a real
RTAB-Map source can replace the mock:
  data: row-major, index = gy*width + gx, gy=0 is the BOTTOM row
  values: -1 unknown, 0 free, 100 occupied.  origin = world position of cell (0,0).
"""
import math
from collections import deque
from typing import Protocol

import numpy as np

MAZE = [
    "#####################",
    "#S..#.......#.......#",
    "#.#.#.#####.#.#####.#",
    "#.#...#...#...#...#.#",
    "#.#####.#.#####.#.#.#",
    "#.......#.......#...#",
    "#.#######.#########.#",
    "#.#.....#.#.......#.#",
    "#.#.###.#.#.#####.#.#",
    "#...#.....#.....#..E#",
    "#####################",
]


class MapSource(Protocol):
    version: int
    def reset(self) -> None: ...
    def advance(self, dt: float, moving: bool) -> None: ...   # mock only; real source: no-op
    def pose_dict(self) -> dict: ...
    def snapshot(self) -> dict: ...


def _bfs(truth, start, end):
    h, w = truth.shape
    prev, q = {start: None}, deque([start])
    while q:
        cur = q.popleft()
        if cur == end:
            break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (cur[0] + dx, cur[1] + dy)
            if 0 <= n[0] < w and 0 <= n[1] < h and truth[n[1], n[0]] == 0 and n not in prev:
                prev[n] = cur
                q.append(n)
    if end not in prev:
        raise ValueError("maze has no route from S to E")
    path, c = [], end
    while c is not None:
        path.append(c)
        c = prev[c]
    return path[::-1]


class MockMapSource:
    """A small maze that is revealed as the simulated drone moves along a route.
    The pose is cosmetic: it is not tied to the executor's go_to goals."""

    def __init__(self, cell_m: float = 0.5, reveal: float = 3.0, cells_per_s: float = 1.5):
        self.cell_m, self.reveal, self.cps = cell_m, reveal, cells_per_s
        self.h, self.w = len(MAZE), len(MAZE[0])
        self.truth = np.zeros((self.h, self.w), dtype=np.int8)
        start = end = None
        for r, row in enumerate(MAZE):
            for c, ch in enumerate(row):
                gy = self.h - 1 - r
                if ch == "#":
                    self.truth[gy, c] = 100
                elif ch == "S":
                    start = (c, gy)
                elif ch == "E":
                    end = (c, gy)
        self.route = _bfs(self.truth, start, end)
        self.version = 0
        self.reset()

    def reset(self) -> None:
        self.progress = 0.0
        self.known = np.full((self.h, self.w), -1, dtype=np.int8)
        self.trail = []
        self._yaw = 0.0
        self._reveal()
        self.version += 1

    def _xy(self):
        i = min(int(self.progress), len(self.route) - 1)
        j = min(i + 1, len(self.route) - 1)
        f = self.progress - i
        a, b = self.route[i], self.route[j]
        gx, gy = a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
        if b != a:
            self._yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        return (gx + 0.5) * self.cell_m, (gy + 0.5) * self.cell_m

    def _reveal(self):
        x, y = self._xy()
        cx, cy = int(x / self.cell_m), int(y / self.cell_m)
        yy, xx = np.ogrid[:self.h, :self.w]
        mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= self.reveal ** 2
        self.known[mask] = self.truth[mask]

    def advance(self, dt: float, moving: bool) -> None:
        if not moving or self.progress >= len(self.route) - 1:
            return
        self.progress = min(len(self.route) - 1, self.progress + self.cps * dt)
        self._reveal()
        x, y = self._xy()
        self.trail.append([round(x, 2), round(y, 2)])
        self.version += 1

    def pose_dict(self) -> dict:
        x, y = self._xy()
        return {"x": round(x, 2), "y": round(y, 2), "yaw": round(self._yaw, 2)}

    def snapshot(self) -> dict:
        return {"frame_id": "map", "width": self.w, "height": self.h,
                "resolution": self.cell_m, "origin": {"x": 0.0, "y": 0.0},
                "data": self.known.flatten().tolist(), "pose": self.pose_dict(),
                "path": self.trail[-300:], "version": self.version}
