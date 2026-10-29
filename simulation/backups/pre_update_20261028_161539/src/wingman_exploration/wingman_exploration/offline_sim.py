#!/usr/bin/env python3
"""Offline closed-loop test of the exploration core against the real maze SDF (no ROS/Gazebo).

  python3 -m wingman_exploration.offline_sim --world ~/ws_wingman/simulation/worlds/maze_exploration.sdf
Simulates: 360-ray 10 m lidar, ray-traced occupancy map, drone with accel limits.
Reports coverage, distance flown, collisions and planning time.
"""
import argparse
import math
import re
import time

import numpy as np

from wingman_exploration.explorer_core import (Explorer, PathTracker, build_view, scan_clearances)

RES = 0.05


def load_world(path, half=25.0):
    txt = open(path).read()
    n = int(2 * half / RES)
    gt = np.zeros((n, n), bool)
    for m in re.finditer(r'<model name="((?:outer|inner)_[^"]*)">.*?<pose>([^<]*)</pose>.*?<size>([^<]*)</size>', txt, re.S):
        px, py = (float(v) for v in m.group(2).split()[:2])
        sx, sy = (float(v) for v in m.group(3).split()[:2])
        i0, i1 = int((px - sx / 2 + half) / RES), int(math.ceil((px + sx / 2 + half) / RES))
        j0, j1 = int((py - sy / 2 + half) / RES), int(math.ceil((py + sy / 2 + half) / RES))
        gt[j0:j1, i0:i1] = True
    return gt, -half


def lidar_update(gt, known, ox, x, y, yaw, rmax=10.0, nray=360, noise=0.0):
    ang = yaw + np.linspace(-math.pi, math.pi, nray, endpoint=False)
    steps = np.arange(0.05, rmax, 0.025)
    xs = x + np.cos(ang)[:, None] * steps[None, :]
    ys = y + np.sin(ang)[:, None] * steps[None, :]
    ii = ((xs - ox) / RES).astype(int)
    jj = ((ys - ox) / RES).astype(int)
    ii = np.clip(ii, 0, gt.shape[1] - 1)
    jj = np.clip(jj, 0, gt.shape[0] - 1)
    hit = gt[jj, ii]
    first = np.where(hit.any(1), hit.argmax(1), steps.size)
    rng = np.where(first < steps.size, steps[np.minimum(first, steps.size - 1)], np.inf)
    if noise > 0:  # shift hit index by range noise (in 2.5 cm steps)
        shift = np.round(np.random.randn(nray) * noise / 0.025).astype(int)
        first = np.clip(np.where(first < steps.size, first + shift, first), 1, steps.size)
    mask = np.arange(steps.size)[None, :] < first[:, None]
    known[jj[mask], ii[mask]] = 0
    rows = np.nonzero(first < steps.size)[0]
    known[jj[rows, first[rows]], ii[rows, first[rows]]] = 100
    return rng


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--speed", type=float, default=0.8)
    ap.add_argument("--range", type=float, default=10.0)
    ap.add_argument("--noise", type=float, default=0.0, help="range noise sigma [m] and 1%% random spurious hits")
    ap.add_argument("--max-time", type=float, default=1500.0)
    ap.add_argument("--inflation", type=float, default=0.45)
    ap.add_argument("--start", type=float, nargs=3, default=[0.0, -17.2, math.pi / 2])
    a = ap.parse_args()

    gt, ox = load_world(a.world)
    import cv2
    gtdist = cv2.distanceTransform((~gt).astype(np.uint8) * 255, cv2.DIST_L2, 3) * RES
    min_d = 9.0
    known = np.full(gt.shape, -1, np.int8)
    x, y, yaw = a.start
    start = (x, y, yaw)
    # exploration box in start frame (inside the 32 m maze; excludes the open field around it)
    bounds = (x, y, yaw, 1.5, 32.9, -15.8, 15.8)
    ex = Explorer()
    tr = PathTracker(cruise=a.speed)
    dt, t, plan_t = 0.05, 0.0, -9.0
    plan, mode, no_front, dist_flown, coll = None, "explore", 0, 0.0, 0
    plan_times, trail = [], []
    vbx = vby = 0.0
    wall0 = time.time()
    last_goal_cmp = None
    while t < a.max_time:
        rng = lidar_update(gt, known, ox, x, y, yaw, rmax=a.range, noise=a.noise)
        if t - plan_t >= 1.0:
            plan_t = t
            view = build_view(known.ravel(), known.shape[1], known.shape[0], RES, ox, ox,
                              inflation=a.inflation, bounds=bounds)
            t0 = time.time()
            if mode == "explore":
                p = ex.plan_explore(view, (x, y), yaw, t)
                if p.kind == "none":
                    no_front += 1
                    if no_front >= 3:
                        mode = "return"
                        print(f"[{t:6.1f}s] exploration complete -> return home ({p.info})")
                else:
                    no_front = 0
                    plan = p
                    tr.set_path(p.waypoints, final=False)
            if mode == "return":
                p = ex.plan_to(view, (x, y), (start[0], start[1]))
                if p.kind != "none":
                    plan = p
                    tr.set_path(p.waypoints, final=True)
            plan_times.append(time.time() - t0)
        if plan is not None and mode == "explore" and plan.goal_xy is not None \
                and math.hypot(plan.goal_xy[0] - x, plan.goal_xy[1] - y) < 0.6:
            ex.note_goal_reached(plan.goal_xy, t)
        front, left, right, fl, fr = scan_clearances(rng, -math.pi, 2 * math.pi / 360, 0.1, a.range)
        vx, vy, wz, rem = tr.command(x, y, yaw, dt, front, left, right, fl, fr)
        if mode == "return" and rem < 0.3 and math.hypot(x - start[0], y - start[1]) < 0.8:
            print(f"[{t:6.1f}s] back at entrance")
            break
        # drone kinematics (body-frame command, first-order lag)
        vbx += (vx - vbx) * min(1.0, dt / 0.25)
        vby += (vy - vby) * min(1.0, dt / 0.25)
        yaw += wz * dt
        nx = x + (math.cos(yaw) * vbx - math.sin(yaw) * vby) * dt
        ny = y + (math.sin(yaw) * vbx + math.cos(yaw) * vby) * dt
        ci, cj = int((nx - ox) / RES), int((ny - ox) / RES)
        wd = float(gtdist[cj, ci])
        min_d = min(min_d, wd)
        if wd < 0.25:
            coll += 1
            if coll <= 3:
                print(f"  collision t={t:.1f} pos=({nx:.2f},{ny:.2f}) wall_dist={wd:.2f} mode={mode}")
        dist_flown += math.hypot(nx - x, ny - y)
        x, y = nx, ny
        trail.append((x, y))
        t += dt
    # coverage: reachable free cells inside the maze that the map knows as free
    inside = np.zeros_like(gt)
    i0, i1 = int((-16 - ox) / RES), int((16 - ox) / RES)
    inside[i0:i1, i0:i1] = True
    free_gt = inside & ~gt
    cov = float(((known == 0) & free_gt).sum()) / float(free_gt.sum())
    print(f"sim_time={t:.0f}s wall={time.time()-wall0:.0f}s flown={dist_flown:.0f}m "
          f"coverage={cov*100:.1f}% collisions(ticks)={coll} min_wall_dist={min_d:.2f}m "
          f"plan_ms mean={1000*np.mean(plan_times):.0f} max={1000*np.max(plan_times):.0f} mode={mode}")
    try:
        import cv2
        img = np.full(gt.shape, 128, np.uint8)
        img[known == 0] = 255
        img[known == 100] = 0
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for k in range(1, len(trail)):
            p = ((trail[k][0] - ox) / RES, (trail[k][1] - ox) / RES)
            q = ((trail[k - 1][0] - ox) / RES, (trail[k - 1][1] - ox) / RES)
            cv2.line(img, (int(q[0]), int(q[1])), (int(p[0]), int(p[1])), (0, 0, 255), 1)
        cv2.imwrite("/tmp/sim_result.png", cv2.flip(img, 0))
        print("saved /tmp/sim_result.png")
    except Exception as exc:
        print("no image:", exc)


if __name__ == "__main__":
    main()
