"""WingMan exploration core (pure numpy/cv2, no ROS imports, unit/sim testable).

Pipeline per planning cycle
  OccupancyGrid -> block-downsample (plan_res) -> occupied / unknown / free
  -> clearance field (distance transform) -> "safe" mask (clearance >= inflation)
  -> one Dijkstra from the robot over safe cells (8-connected, prefers corridor centre)
  -> frontier clusters (free cells touching unknown), filtered by size / bounds / blacklist
  -> pick ONE goal (distance - information gain + turn penalty + stickiness)
  -> line-of-sight simplified polyline (few long segments instead of an A* staircase).

PathTracker turns that polyline into body-frame velocity commands (pure pursuit,
speed shaping from LiDAR clearance, wall repulsion, slew-rate limiting).
"""
import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

INF = float("inf")


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# --------------------------------------------------------------------------- grid view
@dataclass
class GridView:
    res: float            # planning resolution [m/cell]
    ox: float             # world x of cell (0,0) corner
    oy: float
    h: int
    w: int
    occ: np.ndarray
    unk: np.ndarray
    free: np.ndarray
    clear: np.ndarray     # metres to nearest occupied cell
    safe: np.ndarray      # free & clear >= inflation
    inflation: float
    known_cells: int = 0
    bounds_mask: Optional[np.ndarray] = None

    def to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))

    def to_xy(self, i: int, j: int) -> Tuple[float, float]:
        return self.ox + (i + 0.5) * self.res, self.oy + (j + 0.5) * self.res

    def inside(self, i: int, j: int) -> bool:
        return 0 <= i < self.w and 0 <= j < self.h


def build_view(data, width, height, res, ox, oy, plan_res=0.10, inflation=0.45,
               occ_thresh=40, bounds=None) -> GridView:
    """bounds = (x0, y0, yaw0, xmin, xmax, ymin, ymax): box in a frame whose origin is
    (x0,y0,yaw0) of the map frame. Frontier goals outside the box are ignored."""
    a = np.asarray(data, dtype=np.int16).reshape(height, width)
    f = max(1, int(round(plan_res / res)))
    h2, w2 = (height // f) * f, (width // f) * f
    blocks = a[:h2, :w2].reshape(h2 // f, f, w2 // f, f)
    occ = (blocks >= occ_thresh).any(axis=(1, 3))
    unk = ((blocks < 0).mean(axis=(1, 3)) >= 0.5) & ~occ
    free = ~occ & ~unk
    rp = res * f
    src = (~occ).astype(np.uint8) * 255
    clear = cv2.distanceTransform(src, cv2.DIST_L2, 3).astype(np.float32) * rp
    safe = free & (clear >= inflation)
    view = GridView(rp, ox, oy, h2 // f, w2 // f, occ, unk, free, clear, safe, inflation,
                    known_cells=int(occ.sum() + free.sum()))
    if bounds is not None:
        x0, y0, yaw0, xmin, xmax, ymin, ymax = bounds
        jj, ii = np.mgrid[0:view.h, 0:view.w]
        wx = ox + (ii + 0.5) * rp - x0
        wy = oy + (jj + 0.5) * rp - y0
        c, s = math.cos(yaw0), math.sin(yaw0)
        lx = c * wx + s * wy
        ly = -s * wx + c * wy
        view.bounds_mask = (lx >= xmin) & (lx <= xmax) & (ly >= ymin) & (ly <= ymax)
    return view


# --------------------------------------------------------------------------- search
def nearest_cell(mask: np.ndarray, i: int, j: int, max_r: int) -> Optional[Tuple[int, int]]:
    h, w = mask.shape
    j0, j1, i0, i1 = max(0, j - max_r), min(h, j + max_r + 1), max(0, i - max_r), min(w, i + max_r + 1)
    if j0 >= j1 or i0 >= i1:
        return None
    ys, xs = np.nonzero(mask[j0:j1, i0:i1])
    if len(xs) == 0:
        return None
    d2 = (xs + i0 - i) ** 2 + (ys + j0 - j) ** 2
    k = int(np.argmin(d2))
    return int(xs[k] + i0), int(ys[k] + j0)


def dijkstra(view: GridView, start: Tuple[int, int], pref_clear=1.0, k_pen=3.0):
    """Full single-source Dijkstra over view.safe. Returns (dist list, parent list)."""
    h, w, rp = view.h, view.w, view.res
    n = h * w
    pas = view.safe.ravel().tolist()
    mult = (1.0 + k_pen * np.clip((pref_clear - view.clear) / pref_clear, 0.0, 1.0)).astype(np.float64).ravel().tolist()
    dist = [INF] * n
    parent = [-1] * n
    s = start[1] * w + start[0]
    dist[s] = 0.0
    nb = [(-1, 0, rp), (1, 0, rp), (0, -1, rp), (0, 1, rp),
          (-1, -1, rp * 1.4142), (1, -1, rp * 1.4142), (-1, 1, rp * 1.4142), (1, 1, rp * 1.4142)]
    heap = [(0.0, s)]
    pop, push = heapq.heappop, heapq.heappush
    while heap:
        d, u = pop(heap)
        if d > dist[u]:
            continue
        uy, ux = divmod(u, w)
        for dx, dy, step in nb:
            vx, vy = ux + dx, uy + dy
            if vx < 0 or vx >= w or vy < 0 or vy >= h:
                continue
            v = vy * w + vx
            if not pas[v]:
                continue
            if dx and dy and not (pas[uy * w + vx] and pas[vy * w + ux]):
                continue  # no corner cutting
            nd = d + step * mult[v]
            if nd < dist[v]:
                dist[v] = nd
                parent[v] = u
                push(heap, (nd, v))
    return dist, parent


def extract_cells(parent, w, goal):
    out, u = [], goal[1] * w + goal[0]
    while u != -1:
        out.append((u % w, u // w))
        u = parent[u]
    out.reverse()
    return out


def los_ok(view: GridView, a, b, min_clear: float, skip_first: float = 0.4) -> bool:
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(d / (view.res * 0.5)))
    for k in range(n + 1):
        t = k / n
        x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        if t * d < skip_first:
            continue
        i, j = view.to_cell(x, y)
        if not view.inside(i, j) or not view.free[j, i] or view.clear[j, i] < min_clear:
            return False
    return True


def simplify(view: GridView, pts: List[Tuple[float, float]], margin: float = 1.25) -> List[Tuple[float, float]]:
    """Greedy string-pulling: farthest visible point from each anchor."""
    if len(pts) <= 2:
        return pts
    need = view.inflation * margin
    out = [pts[0]]
    anchor = 0
    while anchor < len(pts) - 1:
        nxt = anchor + 1
        for k in range(len(pts) - 1, anchor, -1):
            if los_ok(view, pts[anchor], pts[k], need):
                nxt = k
                break
        out.append(pts[nxt])
        anchor = nxt
    return out


# --------------------------------------------------------------------------- frontiers
@dataclass
class Cluster:
    rep: Tuple[int, int]
    size_m: float
    ncells: int


def frontier_clusters(view: GridView, min_cells: int = 5) -> List[Cluster]:
    unk_d = cv2.dilate(view.unk.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    fr = view.free & unk_d
    if view.bounds_mask is not None:
        fr &= view.bounds_mask
    if not fr.any():
        return []
    n, labels, stats, cents = cv2.connectedComponentsWithStats(fr.astype(np.uint8), connectivity=8)
    out = []
    for lab in range(1, n):
        area = int(stats[lab, cv2.CC_STAT_AREA])
        if area < min_cells:
            continue
        x0, y0, bw, bh = (int(stats[lab, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                                                       cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
        sub = (labels[y0:y0 + bh, x0:x0 + bw] == lab) & view.safe[y0:y0 + bh, x0:x0 + bw]
        ys, xs = np.nonzero(sub)
        if len(xs) == 0:
            continue
        cx, cy = cents[lab][0] - x0, cents[lab][1] - y0
        k = int(np.argmin((xs - cx) ** 2 + (ys - cy) ** 2))
        out.append(Cluster((int(xs[k] + x0), int(ys[k] + y0)), area * view.res, area))
    return out


@dataclass
class Plan:
    kind: str                       # explore | goto | none
    goal_xy: Optional[Tuple[float, float]] = None
    waypoints: List[Tuple[float, float]] = field(default_factory=list)
    cost: float = 0.0
    info: str = ""
    n_clusters: int = 0


class Explorer:
    def __init__(self, gain_w=0.6, turn_w=3.0, stick_bonus=3.0, stick_radius=4.0,
                 vlm_bonus=1.0, reach_dwell=2.0, visited_radius=1.0, min_cells=5):
        self.gain_w, self.turn_w = gain_w, turn_w
        self.stick_bonus, self.stick_radius = stick_bonus, stick_radius
        self.vlm_bonus, self.reach_dwell = vlm_bonus, reach_dwell
        self.visited_radius, self.min_cells = visited_radius, min_cells
        self.black: List[list] = []     # [x, y, r, expire or None]
        self.pending: List[tuple] = []  # (x, y, t_reached)
        self.prev_goal: Optional[Tuple[float, float]] = None

    # -- memory
    def blacklist(self, xy, r, now, ttl=None):
        self.black.append([xy[0], xy[1], r, None if ttl is None else now + ttl])

    def note_goal_reached(self, xy, now):
        self.pending.append((xy[0], xy[1], now))
        self.prev_goal = None

    def _black_kind(self, xy, now):
        """None | 'perm' | 'temp' (temporary cooldown after a stall)."""
        self.black = [b for b in self.black if b[3] is None or b[3] > now]
        kind = None
        for b in self.black:
            if math.hypot(xy[0] - b[0], xy[1] - b[1]) < b[2]:
                if b[3] is None:
                    return "perm"
                kind = "temp"
        return kind

    # -- helpers
    def _start(self, view, rxy):
        i, j = view.to_cell(*rxy)
        if view.inside(i, j) and view.safe[j, i]:
            return i, j
        return nearest_cell(view.safe, i, j, int(0.8 / view.res))

    def _path(self, view, parent, rxy, cell):
        cells = extract_cells(parent, view.w, cell)
        pts = [view.to_xy(i, j) for i, j in cells]
        pts[0] = rxy
        return simplify(view, pts)

    # -- planners
    def plan_explore(self, view, rxy, ryaw, now, vlm_dir="unknown") -> Plan:
        s = self._start(view, rxy)
        if s is None:
            return Plan("none", info="robot not in safe free space")
        dist, parent = dijkstra(view, s)
        clusters = frontier_clusters(view, self.min_cells)
        # frontiers that persisted after we stood on them are lidar-unresolvable: forget them
        keep = []
        for (px, py, t0) in self.pending:
            if now - t0 < self.reach_dwell:
                keep.append((px, py, t0))
                continue
            for c in clusters:
                if math.hypot(view.to_xy(*c.rep)[0] - px, view.to_xy(*c.rep)[1] - py) < self.visited_radius:
                    self.blacklist((px, py), self.visited_radius, now)
                    break
        self.pending = keep
        best, best_cost, n_ok, n_temp = None, INF, 0, 0
        for c in clusters:
            gxy = view.to_xy(*c.rep)
            kind = self._black_kind(gxy, now)
            if kind == "perm":
                continue
            if kind == "temp":
                n_temp += 1
                continue
            d = dist[c.rep[1] * view.w + c.rep[0]]
            if d == INF:
                continue
            n_ok += 1
            ang = wrap(math.atan2(gxy[1] - rxy[1], gxy[0] - rxy[0]) - ryaw)
            cost = d - self.gain_w * math.sqrt(c.size_m) + self.turn_w * abs(ang) / math.pi
            if self.prev_goal is not None and math.hypot(gxy[0] - self.prev_goal[0], gxy[1] - self.prev_goal[1]) < self.stick_radius:
                cost -= self.stick_bonus
            if vlm_dir in ("left", "forward", "right"):
                side = "left" if ang > 0.45 else "right" if ang < -0.45 else "forward"
                if side == vlm_dir:
                    cost -= self.vlm_bonus
            if cost < best_cost:
                best, best_cost = (c, gxy, d), cost
        if best is None:
            if n_temp:
                return Plan("none", info=f"{n_temp} frontier(s) cooling down", n_clusters=len(clusters))
            return Plan("none", info="no reachable frontier", n_clusters=len(clusters))
        c, gxy, d = best
        self.prev_goal = gxy
        return Plan("explore", gxy, self._path(view, parent, rxy, c.rep), d,
                    f"frontier size={c.size_m:.1f}m cost={d:.1f}", n_ok)

    def plan_to(self, view, rxy, goal_xy) -> Plan:
        s = self._start(view, rxy)
        if s is None:
            return Plan("none", info="robot not in safe free space")
        gi, gj = view.to_cell(*goal_xy)
        g = nearest_cell(view.safe, gi, gj, int(1.5 / view.res))
        if g is None:
            return Plan("none", info="goal not near known safe space")
        dist, parent = dijkstra(view, s)
        d = dist[g[1] * view.w + g[0]]
        if d == INF:
            return Plan("none", info="goal unreachable through known safe space")
        pts = self._path(view, parent, rxy, g)
        return Plan("goto", view.to_xy(*g), pts, d, "route")


# --------------------------------------------------------------------------- scan helper
def scan_clearances(ranges, angle_min, angle_inc, range_min, range_max):
    """Min range per sector (body frame): front, left, right, front-left, front-right."""
    r = np.asarray(ranges, dtype=np.float64)
    ang = angle_min + np.arange(len(r)) * angle_inc
    ang = np.arctan2(np.sin(ang), np.cos(ang))
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r = np.where(ok, r, range_max)

    def m(lo, hi):
        sel = (ang >= math.radians(lo)) & (ang <= math.radians(hi))
        return float(r[sel].min()) if sel.any() else float(range_max)
    return m(-22, 22), m(50, 130), m(-130, -50), m(22, 50), m(-50, -22)


# --------------------------------------------------------------------------- tracking
class PathTracker:
    def __init__(self, cruise=0.8, max_yaw=1.0, accel=1.2, decel=3.0, stop_d=0.6, slow_d=1.8):
        self.cruise, self.max_yaw, self.accel, self.decel = cruise, max_yaw, accel, decel
        self.stop_d, self.slow_d = stop_d, slow_d
        self.pts: List[Tuple[float, float]] = []
        self.final = False
        self.vx = self.vy = 0.0
        self.idx = 0

    def set_path(self, pts, final=False):
        self.pts, self.final, self.idx = list(pts), final, 0

    def reset_cmd(self):
        self.vx = self.vy = 0.0

    def _project(self, x, y):
        best, bi, bt = INF, self.idx, 0.0
        for i in range(max(0, self.idx - 1), len(self.pts) - 1):
            ax, ay = self.pts[i]
            bx, by = self.pts[i + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            d = math.hypot(ax + t * dx - x, ay + t * dy - y)
            if d < best - 1e-6:
                best, bi, bt = d, i, t
        self.idx = bi
        return bi, bt

    def command(self, x, y, yaw, dt, front, left, right, fl=None, fr=None, cruise=None):
        """returns (vx, vy, wz, remaining_m). body frame: x forward, y left, wz CCW."""
        cruise = self.cruise if cruise is None else cruise
        fl = front if fl is None else fl
        fr = front if fr is None else fr
        if len(self.pts) < 2:
            vx, vy = self._slew(0.0, 0.0, dt)
            return vx, vy, 0.0, 0.0
        i, t = self._project(x, y)
        seg = [self.pts[k] for k in range(i, len(self.pts))]
        px = seg[0][0] + t * (seg[1][0] - seg[0][0])
        py = seg[0][1] + t * (seg[1][1] - seg[0][1])
        poly = [(px, py)] + seg[1:]
        lens = [math.hypot(poly[k + 1][0] - poly[k][0], poly[k + 1][1] - poly[k][1]) for k in range(len(poly) - 1)]
        remaining = sum(lens)
        if self.final and remaining < 0.25:
            vx, vy = self._slew(0.0, 0.0, dt)
            return vx, vy, 0.0, remaining
        look = max(0.6, min(1.4, 0.6 + 0.6 * abs(self.vx)))
        # sharp upcoming vertex within the lookahead: fly TO the vertex (never cut the corner)
        turn_ahead, d_vertex = 0.0, INF
        if len(poly) >= 3:
            a1 = math.atan2(poly[1][1] - poly[0][1], poly[1][0] - poly[0][0])
            a2 = math.atan2(poly[2][1] - poly[1][1], poly[2][0] - poly[1][0])
            turn_ahead, d_vertex = abs(wrap(a2 - a1)), lens[0]
        if turn_ahead > math.radians(25) and d_vertex < look + 0.3:
            tx, ty = poly[1]
        else:
            tx, ty, acc = poly[-1][0], poly[-1][1], 0.0
            for k in range(len(poly) - 1):
                if acc + lens[k] >= look and lens[k] > 1e-9:
                    u = (look - acc) / lens[k]
                    tx = poly[k][0] + u * (poly[k + 1][0] - poly[k][0])
                    ty = poly[k][1] + u * (poly[k + 1][1] - poly[k][1])
                    break
                acc += lens[k]
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        wz = max(-self.max_yaw, min(self.max_yaw, 2.2 * err))
        v = cruise
        if self.final:
            v = min(v, 0.15 + 0.8 * remaining)
        if turn_ahead > math.radians(25) and d_vertex < 2.0:      # slow into corners
            k = min(1.0, turn_ahead / (math.pi / 2))
            v *= 1.0 - 0.6 * k * (1.0 - max(0.0, (d_vertex - 0.4)) / 1.6)
        eff = min(front, fl + 0.35, fr + 0.35)
        v *= max(0.0, min(1.0, (eff - self.stop_d) / (self.slow_d - self.stop_d)))
        if abs(err) > math.radians(70):
            v = 0.0
        else:
            v *= max(0.0, math.cos(err)) ** 2
        # push away from close walls only (never pull toward openings)
        d0 = 0.9
        # +vy = move left. Wall close on the LEFT must push RIGHT (negative), and vice versa.
        push = (max(0.0, d0 - right) + 0.7 * max(0.0, 0.8 - fr)) - (max(0.0, d0 - left) + 0.7 * max(0.0, 0.8 - fl))
        vy = max(-0.35, min(0.35, 0.6 * push))
        vx, vy = self._slew(v, vy, dt)
        return vx, vy, wz, remaining

    def _slew(self, vx, vy, dt):
        def s(cur, tgt):
            lim = (self.accel if abs(tgt) > abs(cur) else self.decel) * dt
            return cur + max(-lim, min(lim, tgt - cur))
        self.vx, self.vy = s(self.vx, vx), s(self.vy, vy)
        return self.vx, self.vy
