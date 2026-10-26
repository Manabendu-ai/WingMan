"""Offline simulation pieces: a fake drone and an in-process link to it.

The link runs the real MissionExecutor, GoalBridge and projector. The VLM
(real, or a fake in tests) looks at saved images instead of a live camera.
Depth is faked at 3 m and the pose is the origin, so goal coordinates only
prove the logic, not the geometry.
"""
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np
from pydantic import ValidationError

from wingman_ground.mission import Mission
from wingman_vlm.projector import Intrinsics, Pose, project_goal

from .executor import Feedback, MissionExecutor, State

IMG_EXT = {".jpg", ".jpeg", ".png", ".webp"}


def default_frames(data_dir) -> List[Path]:
    """No-target images first, then each red-box image twice (consecutive
    video frames look alike, which is what the bridge's confirmation needs)."""
    imgs = sorted(p for p in Path(data_dir).iterdir()
                  if p.suffix.lower() in IMG_EXT)
    plain = [p for p in imgs if "redbox" not in p.name]
    red = [p for p in imgs for _ in (0, 1) if "redbox" in p.name]
    return plain + red


class FrameFeed:
    """Hands out frames in order, then keeps repeating the last one."""
    def __init__(self, paths):
        if not paths:
            raise ValueError("no frames")
        self._paths = list(paths)

    def next(self) -> Path:
        return self._paths.pop(0) if len(self._paths) > 1 else self._paths[0]


class SimDrone:
    def __init__(self, say: Callable[[str], None] = print,
                 stuck_takeoff: bool = False):
        self.say = say
        self.stuck_takeoff = stuck_takeoff
        self.now = 0.0
        self._prev = 0.0
        self.alt = 0.0
        self.target_alt = 0.0
        self.goal_t: Optional[float] = None

    def _say(self, msg: str):
        self.say(f"   [t={self.now:5.1f}] DRONE <- {msg}")

    def set_limits(self, v, a):
        self._say(f"limits speed<={v} alt<={a}")

    def takeoff(self, a):
        self.target_alt = a
        self._say(f"takeoff to {a} m" + (" (stuck!)" if self.stuck_takeoff else ""))

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

    def feedback(self, now: float) -> Feedback:
        dt = now - self._prev
        self._prev = now
        if not self.stuck_takeoff:
            self.alt = min(self.target_alt, self.alt + 0.5 * dt)
        reached = self.goal_t is not None and now - self.goal_t >= 3.0
        return Feedback(altitude_m=self.alt, goal_reached=reached,
                        home_reached=True, landed=True)


class InProcessLink:
    def __init__(self, observe, frames, dt: float = 1.0,
                 max_sim_s: float = 400.0, say: Callable[[str], None] = print,
                 break_first_takeoff: bool = False):
        self.observe = observe
        self.feed = FrameFeed(frames)
        self.dt = dt
        self.max_sim_s = max_sim_s
        self.say = say
        self._break_next_takeoff = break_first_takeoff
        self._mission: Optional[Mission] = None
        self.K = Intrinsics(300.0, 300.0, 160.0, 120.0)
        self.depth = np.full((240, 320), 3.0, dtype=np.float32)

    def send_mission(self, mission_json: str) -> bool:
        try:
            self._mission = Mission.model_validate_json(mission_json)
        except ValidationError as e:
            self.say(f"   DRONE rejected the mission "
                     f"({e.error_count()} validation error(s))")
            self._mission = None
            return False
        return True

    def run_until_done(self) -> Dict[str, Any]:
        if self._mission is None:
            raise RuntimeError("no mission was accepted")
        drone = SimDrone(self.say, stuck_takeoff=self._break_next_takeoff)
        self._break_next_takeoff = False          # only the first run breaks
        ex = MissionExecutor(drone)
        ex.load(self._mission)
        now = 0.0
        ex.start(now)
        while ex.state == State.EXECUTING and now < self.max_sim_s:
            now += self.dt
            drone.now = now
            req = ex.vlm_request()
            if req is not None:
                path = self.feed.next()
                res = self.observe(str(path), req.task)
                if not res.ok:
                    why = (res.obs.uncertainties or ["?"])[0]
                    self.say(f"t={now:5.1f} {path.name:15} VLM FAILED: {why}")
                else:
                    g = None
                    o = res.obs
                    if o.target_visible and o.target_x is not None:
                        g = project_goal(o.target_x, o.target_y, self.depth,
                                         self.K, Pose(),
                                         standoff_m=req.standoff_m)
                    d = ex.on_observation(o, g, now)
                    out = f"{d.action} ({d.reason})" if d else "-"
                    self.say(f"t={now:5.1f} {path.name:15} scene={o.scene:16} "
                             f"visible={o.target_visible!s:5} "
                             f"conf={o.confidence:.2f} -> {out}")
            ex.tick(now, drone.feedback(now))
        if ex.state == State.EXECUTING:
            ex.abort(now, "simulation cutoff")
        return ex.status(now)
