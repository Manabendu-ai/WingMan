"""Pure-numpy frontier detection (no ROS imports)."""
import math
from collections import deque
import numpy as np
 
 
def find_frontier_goals(grid, res, ox, oy, robot_xy, bounds, blacklist,
                        min_cluster=10, clearance=0.45, min_goal_dist=0.8,
                        blacklist_radius=0.7, max_candidates=40):
    """grid: int8 [h, w] (-1 unknown, 0..100 occupancy).
    Returns [(score, x, y, cluster_size, dist)] sorted best-first (lower score = better)."""
    h, w = grid.shape
    free = (grid >= 0) & (grid < 50)
    unk = grid == -1
    occ = grid >= 65
    fr = np.zeros_like(free)
    fr[1:-1, 1:-1] = free[1:-1, 1:-1] & (unk[:-2, 1:-1] | unk[2:, 1:-1] |
                                         unk[1:-1, :-2] | unk[1:-1, 2:])
    cells = np.argwhere(fr)
    if len(cells) == 0:
        return []
    cellset = set(map(tuple, cells.tolist()))
    visited = set()
    clusters = []
    for c in list(cellset):
        if c in visited:
            continue
        q = deque([c]); visited.add(c); comp = []
        while q:
            i, j = q.popleft(); comp.append((i, j))
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    n = (i + di, j + dj)
                    if n in cellset and n not in visited:
                        visited.add(n); q.append(n)
        if len(comp) >= min_cluster:
            clusters.append(comp)
 
    r = max(1, int(round(clearance / res)))
    rx, ry = robot_xy
    xmin, xmax, ymin, ymax = bounds
    out = []
    for comp in clusters:
        arr = np.array(comp, dtype=float)
        ci, cj = arr.mean(axis=0)
        order = np.argsort((arr[:, 0] - ci) ** 2 + (arr[:, 1] - cj) ** 2)[:max_candidates]
        for k in order:
            i, j = int(arr[k, 0]), int(arr[k, 1])
            x = ox + (j + 0.5) * res
            y = oy + (i + 0.5) * res
            if not (xmin <= x <= xmax and ymin <= y <= ymax):
                continue
            i0, i1, j0, j1 = max(0, i - r), min(h, i + r + 1), max(0, j - r), min(w, j + r + 1)
            if occ[i0:i1, j0:j1].any():
                continue
            if any(math.hypot(x - bx, y - by) < blacklist_radius for bx, by in blacklist):
                continue
            d = math.hypot(x - rx, y - ry)
            if d < min_goal_dist:
                continue
            out.append((d / (1.0 + 0.02 * len(comp)), x, y, len(comp), d))
            break
    out.sort()
    return out
