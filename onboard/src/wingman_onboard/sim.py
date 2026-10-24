"""Simulated drone, image frame source and flight loop for offline demos."""
from pathlib import Path

import numpy as np

from wingman_onboard.executor import Feedback, MissionExecutor, State
from wingman_vlm.client import observe
from wingman_vlm.projector import Intrinsics, Pose, project_goal

K = Intrinsics(fx=300.0, fy=300.0, cx=160.0, cy=120.0)
DEPTH = np.full((240, 320), 3.0, dtype=np.float32)


class SimDrone:
    def __init__(self, stuck_takeoff: bool = False, verbose: bool = True):
        self.now = 0.0
        self.alt = 0.0
        self.target_alt = 0.0
        self.goal_t = None
        self.stuck = stuck_takeoff
        self.verbose = verbose

    def _say(self, msg):
        if self.verbose:
            print(f"   [t={self.now:5.1f}] DRONE <- {msg}")

    def set_limits(self, v, a):
        self._say(f"limits speed<={v} alt<={a}")

    def takeoff(self, a):
        self.target_alt = a
        self._say(f"takeoff to {a} m" + ("  (SIMULATED STALL)" if self.stuck else ""))

    def explore(self, s):
        self._say(f"explore ({s})")

    def go_to(self, x, y, z):
        self.goal_t = self.now
        self._say(f"go_to ({x:.2f}, {y:.2f}, {z:.2f})")

    def hover(self):
        self._say("hover")

    def return_to_start(self):
        self._say("return to start")

    def land(self):
        self._say("land")

    def feedback(self, now):
        if not self.stuck:
            self.alt = min(self.target_alt, self.alt + 0.5)
        else:
            self.alt = min(0.3, self.target_alt)
        reached = self.goal_t is not None and now - self.goal_t >= 3
        return Feedback(altitude_m=self.alt, goal_reached=reached,
                        home_reached=True, landed=True)


class ImageFrames:
    """No-target images first, then each red-box image twice. Repeats the last."""
    def __init__(self, folder):
        imgs = sorted(p for p in Path(folder).iterdir()
                      if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".webp"})
        plain = [p for p in imgs if "redbox" not in p.name]
        red = [p for p in imgs for _ in (0, 1) if "redbox" in p.name]
        self._all = plain + red
        if not self._all:
            raise SystemExit(f"no images found in {folder}")
        self.reset()

    def reset(self):
        self._q = list(self._all)

    def next(self):
        return self._q.pop(0) if len(self._q) > 1 else self._q[0]


def fly(mission, drone, frames, max_sim_s: float = 400.0, verbose: bool = True):
    """Run one mission to the end. Returns the executor's status dict."""
    ex = MissionExecutor(drone)
    ex.load(mission)
    now = 0.0
    drone.now = now
    ex.start(now)
    while ex.state == State.EXECUTING and now < max_sim_s:
        now += 1.0
        drone.now = now
        req = ex.vlm_request()
        if req is not None:
            path = frames.next()
            res = observe(str(path), req.task)
            if res.ok and res.obs is not None:
                g = None
                if res.obs.target_visible and res.obs.target_x is not None:
                    g = project_goal(res.obs.target_x, res.obs.target_y, DEPTH,
                                     K, Pose(), standoff_m=req.standoff_m)
                d = ex.on_observation(res.obs, g, now)
                if verbose:
                    print(f"t={now:5.1f} {path.name:15} "
                          f"visible={res.obs.target_visible!s:5} "
                          f"conf={res.obs.confidence:.2f} -> {d.action} ({d.reason})")
            elif verbose:
                print(f"t={now:5.1f} {path.name:15} VLM call failed, skipping frame")
        ex.tick(now, drone.feedback(now))
    if ex.state == State.EXECUTING:
        ex.abort(now, "sim time limit")
    return ex.status(now)
