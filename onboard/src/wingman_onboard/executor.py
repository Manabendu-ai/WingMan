"""Onboard mission executor: a deterministic state machine (no AI).

It runs a validated Mission step by step. The VLM pipeline (observe ->
project -> GoalBridge) feeds it through on_observation(); the drone side
(ROS later) implements DroneInterface and reports Feedback every tick.
All methods take `now` (seconds) so behaviour is deterministic and testable.
"""
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Protocol

import numpy as np

from wingman_ground.mission import (ApproachTarget, Explore, Hover, Land,
                                    Mission, ReturnToStart, Takeoff)
from wingman_vlm.bridge import Decision, GoalBridge
from wingman_vlm.projector import Goal
from wingman_vlm.schema import Observation


class State(str, Enum):
    IDLE = "idle"
    READY = "ready"
    EXECUTING = "executing"
    COMPLETE = "complete"
    FAILED = "failed"
    ABORTED = "aborted"


class DroneInterface(Protocol):
    """What the executor can ask the drone side to do."""
    def set_limits(self, max_speed_mps: float, max_altitude_m: float) -> None: ...
    def takeoff(self, altitude_m: float) -> None: ...
    def explore(self, strategy: str) -> None: ...
    def go_to(self, x: float, y: float, z: float) -> None: ...
    def hover(self) -> None: ...
    def return_to_start(self) -> None: ...
    def land(self) -> None: ...


@dataclass
class Feedback:
    """What the drone side reports every tick."""
    altitude_m: float = 0.0
    goal_reached: bool = False
    home_reached: bool = False
    landed: bool = False


@dataclass
class VlmRequest:
    """What the VLM pipeline should look for right now."""
    task: str
    standoff_m: float


@dataclass
class ExecutorConfig:
    takeoff_timeout_s: float = 30.0
    approach_timeout_s: float = 90.0
    return_timeout_s: float = 120.0
    land_timeout_s: float = 30.0
    explore_time_limit_s: float = 120.0   # for explore until="time_limit"
    altitude_tolerance_m: float = 0.15
    goal_reached_grace_s: float = 1.0     # ignore stale "reached" after a new goal
    default_standoff_m: float = 0.5
    on_fail: str = "hover"                # "hover" or "land"


class MissionExecutor:
    def __init__(self, drone: DroneInterface,
                 bridge: Optional[GoalBridge] = None,
                 cfg: Optional[ExecutorConfig] = None):
        self.drone = drone
        self.bridge = bridge or GoalBridge()
        self.cfg = cfg or ExecutorConfig()
        self.state = State.IDLE
        self.mission: Optional[Mission] = None
        self._reset_runtime()

    # ---------------- setup ----------------
    def _reset_runtime(self):
        self.index = 0
        self.t_start = 0.0
        self._step_t0 = 0.0
        self.target_goal: Optional[np.ndarray] = None
        self.failure_reason: Optional[str] = None
        self._goal_sent = False
        self._goal_sent_t = 0.0
        self._searching = False
        self._last_obs: Optional[Dict[str, Any]] = None
        self._events: List[list] = []

    @property
    def step(self):
        if self.mission is None or self.index >= len(self.mission.steps):
            return None
        return self.mission.steps[self.index]

    def load(self, mission) -> None:
        if self.state == State.EXECUTING:
            raise RuntimeError("a mission is already executing")
        m = mission if isinstance(mission, Mission) else Mission.model_validate(mission)
        # Re-validate with the same code as the ground station. Never trust the link.
        self.mission = Mission.model_validate(m.model_dump())
        self._reset_runtime()
        self.state = State.READY

    def start(self, now: float) -> None:
        if self.state != State.READY or self.mission is None:
            raise RuntimeError("load a mission before start()")
        self.state = State.EXECUTING
        self.t_start = now
        lim = self.mission.limits
        self.drone.set_limits(lim.max_speed_mps, lim.max_altitude_m)
        self.bridge.mark_reached()          # clear any old goal
        self._log(now, f"mission {self.mission.mission_id} started")
        self._enter_step(now)

    # ---------------- internals ----------------
    def _log(self, now: float, text: str):
        self._events.append([round(now - self.t_start, 1), text])
        self._events = self._events[-50:]

    def _send_goal(self, now: float):
        g = self.target_goal
        self.drone.go_to(float(g[0]), float(g[1]), float(g[2]))
        self._goal_sent = True
        self._goal_sent_t = now
        self._searching = False

    def _enter_step(self, now: float):
        s = self.step
        self._step_t0 = now
        self._goal_sent = False
        self._searching = False
        n = len(self.mission.steps)
        self._log(now, f"step {self.index + 1}/{n}: {s.action}")
        if isinstance(s, Takeoff):
            self.drone.takeoff(s.altitude_m)
        elif isinstance(s, Explore):
            self.target_goal = None         # a new search starts here
            self.bridge.mark_reached()
            self.drone.explore(s.strategy)
        elif isinstance(s, ApproachTarget):
            if self.target_goal is not None:
                self._send_goal(now)
            else:                           # nothing seen yet: look around first
                self._searching = True
                self.drone.explore("frontier")
        elif isinstance(s, Hover):
            self.drone.hover()
        elif isinstance(s, ReturnToStart):
            self.drone.return_to_start()
        elif isinstance(s, Land):
            self.drone.land()

    def _complete_step(self, now: float, note: str):
        self._log(now, f"step {self.index + 1} done: {note}")
        self.index += 1
        if self.index >= len(self.mission.steps):
            self.state = State.COMPLETE
            self._log(now, "mission complete")
            return
        self._enter_step(now)

    def _fail(self, now: float, reason: str):
        self.state = State.FAILED
        self.failure_reason = reason
        self._log(now, f"FAILED: {reason}")
        if self.cfg.on_fail == "land":
            self.drone.land()
        else:
            self.drone.hover()

    # ---------------- public API ----------------
    def vlm_request(self) -> Optional[VlmRequest]:
        """What the VLM pipeline should do now. None means: skip inference."""
        if self.state != State.EXECUTING:
            return None
        s = self.step
        if isinstance(s, ApproachTarget):
            return VlmRequest(self.mission.vlm_task(), s.standoff_m)
        if isinstance(s, Explore) and s.until == "target_found":
            nxt = (self.mission.steps[self.index + 1]
                   if self.index + 1 < len(self.mission.steps) else None)
            standoff = (nxt.standoff_m if isinstance(nxt, ApproachTarget)
                        else self.cfg.default_standoff_m)
            return VlmRequest(self.mission.vlm_task(), standoff)
        return None

    def on_observation(self, obs: Observation, goal: Optional[Goal],
                       now: float, obs_time: Optional[float] = None
                       ) -> Optional[Decision]:
        """Feed one VLM observation (plus its projected goal)."""
        if self.vlm_request() is None:
            return None
        self._last_obs = {"scene": obs.scene,
                          "target_visible": obs.target_visible,
                          "confidence": obs.confidence}
        d = self.bridge.update(obs, goal, now, obs_time)
        if d.action == "publish_goal":
            self.target_goal = np.asarray(d.goal, dtype=float)
            s = self.step
            if isinstance(s, Explore):
                self._complete_step(now, "target found")
            elif isinstance(s, ApproachTarget):
                self._log(now, "target goal updated")
                self._send_goal(now)
        return d

    def tick(self, now: float, fb: Feedback) -> None:
        """Call periodically (e.g. 5-10 Hz) with the drone's feedback."""
        if self.state != State.EXECUTING:
            return
        c = self.cfg
        if now - self.t_start > self.mission.limits.max_time_s:
            return self._fail(now, "mission time limit exceeded")
        s = self.step
        t = now - self._step_t0
        if isinstance(s, Takeoff):
            if fb.altitude_m >= s.altitude_m - c.altitude_tolerance_m:
                self._complete_step(now, "reached altitude")
            elif t > c.takeoff_timeout_s:
                self._fail(now, "takeoff timeout")
        elif isinstance(s, Explore):
            if s.until == "time_limit" and t >= c.explore_time_limit_s:
                self._complete_step(now, "explore time elapsed")
        elif isinstance(s, ApproachTarget):
            if (self._goal_sent and fb.goal_reached
                    and now - self._goal_sent_t >= c.goal_reached_grace_s):
                self.bridge.mark_reached()
                self._complete_step(now, "reached target")
            elif t > c.approach_timeout_s:
                self._fail(now, "approach timeout")
        elif isinstance(s, Hover):
            if t >= s.duration_s:
                self._complete_step(now, "hover finished")
        elif isinstance(s, ReturnToStart):
            if fb.home_reached:
                self._complete_step(now, "back at start")
            elif t > c.return_timeout_s:
                self._fail(now, "return timeout")
        elif isinstance(s, Land):
            if fb.landed:
                self._complete_step(now, "landed")
            elif t > c.land_timeout_s:
                self._fail(now, "land timeout")

    def abort(self, now: float, reason: str = "operator abort") -> None:
        if self.state != State.EXECUTING:
            return
        self.state = State.ABORTED
        self.failure_reason = reason
        self._log(now, f"ABORTED: {reason}")
        self.drone.hover()

    def status(self, now: float) -> Dict[str, Any]:
        """Compact, JSON-safe report: this is what goes up to the ground LLM."""
        s = self.step
        started = self.state not in (State.IDLE, State.READY)
        return {
            "mission_id": self.mission.mission_id if self.mission else None,
            "state": self.state.value,
            "step_index": self.index if self.mission else None,
            "steps_total": len(self.mission.steps) if self.mission else 0,
            "step_action": s.action if s is not None else None,
            "elapsed_s": round(now - self.t_start, 1) if started else 0.0,
            "target_found": self.target_goal is not None,
            "target_goal": ([round(float(v), 2) for v in self.target_goal]
                            if self.target_goal is not None else None),
            "vlm_online": self.bridge.tick(now).vlm_online,
            "last_observation": self._last_obs,
            "failure_reason": self.failure_reason,
            "events": self._events[-10:],
        }
