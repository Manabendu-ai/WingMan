"""Live, abortable, observable replacement for InProcessLink (same Link protocol)."""
import threading

import numpy as np
from pydantic import ValidationError

from wingman_ground.mission import Mission
from wingman_onboard.executor import MissionExecutor, State
from wingman_onboard.sim import FrameFeed, SimDrone
from wingman_vlm.projector import Intrinsics, Pose, project_goal

# Failures that point at the aircraft, not the plan. Never auto-replan these.
NON_REPLANNABLE = ("takeoff timeout", "return timeout", "land timeout")


class LiveSimDrone(SimDrone):
    def __init__(self, say, mapsrc, stuck_takeoff=False):
        super().__init__(say, stuck_takeoff)
        self.map, self.mode = mapsrc, "idle"

    @property
    def moving(self):
        return self.mode in ("explore", "approach")

    def takeoff(self, a): super().takeoff(a); self.mode = "takeoff"
    def explore(self, s): super().explore(s); self.mode = "explore"
    def go_to(self, x, y, z): super().go_to(x, y, z); self.mode = "approach"
    def hover(self): super().hover(); self.mode = "hover"
    def return_to_start(self): super().return_to_start(); self.mode = "return"
    def land(self): super().land(); self.mode = "land"


class LiveLink:
    def __init__(self, observe, frames, hub, mapsrc, dt=1.0, tick_real_s=0.4,
                 max_sim_s=400.0, break_first_takeoff=False):
        self.observe, self.feed, self.hub, self.map = observe, FrameFeed(frames), hub, mapsrc
        self.dt, self.tick_real_s, self.max_sim_s = dt, tick_real_s, max_sim_s
        self._break_next = break_first_takeoff
        self._abort = threading.Event()
        self._mission = None
        self.running = False
        self.K = Intrinsics(300.0, 300.0, 160.0, 120.0)
        self.depth = np.full((240, 320), 3.0, dtype=np.float32)

    def request_abort(self) -> None:
        self._abort.set()

    def send_mission(self, mission_json: str) -> bool:
        try:
            self._mission = Mission.model_validate_json(mission_json)
        except ValidationError as e:
            self.hub.say(f"DRONE rejected the mission ({e.error_count()} validation error(s))")
            self._mission = None
            return False
        self._abort.clear()
        return True

    def _publish(self, ex, drone, now):
        st = ex.status(now)
        st.update(altitude_m=round(drone.alt, 2), sim_t=round(now, 1),
                  mode=drone.mode, pose=self.map.pose_dict())
        self.hub.set_telemetry(st)
        return st

    def _observe(self, ex, now):
        req = ex.vlm_request()
        if req is None:
            return
        path = self.feed.next()
        res = self.observe(str(path), req.task)
        if not res.ok:   # failed VLM calls are never fed to the executor
            why = (res.obs.uncertainties or ["?"])[0]
            self.hub.set_frame(path, None)
            self.hub.say(f"t={now:5.1f} {path.name} VLM FAILED: {why}")
            return
        o, g = res.obs, None
        if o.target_visible and o.target_x is not None:
            g = project_goal(o.target_x, o.target_y, self.depth, self.K, Pose(),
                             standoff_m=req.standoff_m)
        d = ex.on_observation(o, g, now)
        self.hub.set_frame(path, {"scene": o.scene, "target_visible": o.target_visible,
                                  "x": o.target_x, "y": o.target_y, "confidence": o.confidence})
        self.hub.say(f"t={now:5.1f} {path.name} scene={o.scene} visible={o.target_visible} "
                     f"conf={o.confidence:.2f} -> {d.action + ' (' + d.reason + ')' if d else '-'}")

    def run_until_done(self):
        if self._mission is None:
            raise RuntimeError("no mission was accepted")
        reset = getattr(self.observe, "reset", None)
        if reset:
            reset()
        self.map.reset()
        self.hub.reset_run()
        drone = LiveSimDrone(self.hub.say, self.map, stuck_takeoff=self._break_next)
        self._break_next = False
        ex = MissionExecutor(drone)
        ex.load(self._mission)
        now = 0.0
        ex.start(now)
        self.running = True
        try:
            self._publish(ex, drone, now)
            while ex.state == State.EXECUTING and now < self.max_sim_s:
                now += self.dt
                drone.now = now
                if self._abort.is_set():
                    ex.abort(now)
                    break
                self._observe(ex, now)
                ex.tick(now, drone.feedback(now))
                self.map.advance(self.dt, drone.moving)
                self._publish(ex, drone, now)
                self._abort.wait(self.tick_real_s)     # real-time pacing, wakes on abort
            if ex.state == State.EXECUTING:
                ex.abort(now, "simulation cutoff")
            return self._publish(ex, drone, now)
        finally:
            self.running = False


class PolicyLink:
    """Wraps a Link. Hardware-looking failures are returned as 'aborted' so
    GroundSession reports them to the operator instead of replanning."""

    def __init__(self, inner):
        self.inner = inner

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def send_mission(self, mission_json):
        return self.inner.send_mission(mission_json)

    def run_until_done(self):
        st = self.inner.run_until_done()
        if st.get("state") == "failed" and st.get("failure_reason") in NON_REPLANNABLE:
            st = dict(st, state="aborted", policy_halt=True,
                      failure_reason=f"{st['failure_reason']} - possible flight fault, "
                                     "not auto-replanned; check the drone")
        return st
