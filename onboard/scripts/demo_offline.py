"""Offline demo: LLM mission -> executor -> real VLM on test images.
Needs: the local VLM server running (vlm/scripts/run_server.sh) and a saved
mission (python ground/scripts/plan.py "take off, find the red box, then hover over it").
"""
import json
from pathlib import Path

import numpy as np

from wingman_ground.mission import Mission
from wingman_onboard.executor import Feedback, MissionExecutor, State
from wingman_vlm.client import observe
from wingman_vlm.projector import Intrinsics, Pose, project_goal

ROOT = Path(__file__).resolve().parents[2]
mission = Mission.model_validate_json(
    (ROOT / "ground/out/last_mission.json").read_text())
print("MISSION:", mission.goal, "| VLM task:", mission.vlm_task())

imgs = sorted(p for p in (ROOT / "vlm/eval/data").iterdir()
              if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".webp"})
plain = [p for p in imgs if "redbox" not in p.name]
red = [p for p in imgs for _ in (0, 1) if "redbox" in p.name]
frames = plain + red
if not frames:
    raise SystemExit("no images found in vlm/eval/data")

K = Intrinsics(fx=300.0, fy=300.0, cx=160.0, cy=120.0)
depth = np.full((240, 320), 3.0, dtype=np.float32)


class SimDrone:
    """Pretends to fly: prints every command, reports feedback."""
    def __init__(self):
        self.now = 0.0
        self.alt = 0.0
        self.target_alt = 0.0
        self.goal_t = None

    def _say(self, msg):
        print(f"   [t={self.now:5.1f}] DRONE <- {msg}")

    def set_limits(self, v, a):
        self._say(f"limits speed<={v} alt<={a}")

    def takeoff(self, a):
        self.target_alt = a
        self._say(f"takeoff to {a} m")

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
        self.alt = min(self.target_alt, self.alt + 0.5)
        reached = self.goal_t is not None and now - self.goal_t >= 3
        return Feedback(altitude_m=self.alt, goal_reached=reached,
                        home_reached=True, landed=True)


drone = SimDrone()
ex = MissionExecutor(drone)
ex.load(mission)
now = 0.0
ex.start(now)

while ex.state == State.EXECUTING and now < 200:
    now += 1.0
    drone.now = now
    req = ex.vlm_request()
    if req is not None:
        # keep showing the last frame once the list is used up (a camera
        # that stops moving keeps seeing the same thing)
        path = frames.pop(0) if len(frames) > 1 else frames[0]
        res = observe(str(path), req.task)
        g = None
        if res.ok and res.obs.target_visible and res.obs.target_x is not None:
            g = project_goal(res.obs.target_x, res.obs.target_y, depth, K,
                             Pose(), standoff_m=req.standoff_m)
        d = ex.on_observation(res.obs, g, now)
        print(f"t={now:5.1f} {path.name:15} scene={res.obs.scene:16} "
              f"visible={res.obs.target_visible!s:5} conf={res.obs.confidence:.2f} "
              f"-> {d.action} ({d.reason})")
    ex.tick(now, drone.feedback(now))

print("\nFINAL STATUS:")
print(json.dumps(ex.status(now), indent=2))
