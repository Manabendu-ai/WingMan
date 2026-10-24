"""Goal bridge: decides when a VLM-derived goal is trusted enough to send
to the planner. Pure Python, no ROS.

Actions returned in Decision.action:
  publish_goal : send Decision.goal to EGO-Planner now
  keep         : planner continues with its current goal, nothing to send
  explore      : no trusted VLM goal, let the exploration logic choose
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .projector import Goal
from .schema import Observation


@dataclass
class BridgeConfig:
    min_confidence: float = 0.6
    max_obs_age_s: float = 2.0          # drop observations older than this
    confirm_n: int = 2                  # consistent observations needed
    consistency_radius_m: float = 1.0   # "same target" if goals within this
    min_goal_shift_m: float = 0.75      # don't republish smaller moves
    goal_ttl_s: float = 15.0            # forget a goal not re-seen for this long
    vlm_timeout_s: float = 5.0          # VLM considered offline after this


@dataclass
class Decision:
    action: str
    reason: str
    goal: Optional[np.ndarray] = None
    vlm_online: bool = True


class GoalBridge:
    def __init__(self, cfg: Optional[BridgeConfig] = None):
        self.cfg = cfg or BridgeConfig()
        self.active_goal: Optional[np.ndarray] = None
        self._active_since = 0.0
        self._cand: Optional[np.ndarray] = None
        self._streak = 0
        self._last_vlm_time: Optional[float] = None

    # ---------- helpers ----------
    def _vlm_online(self, now: float) -> bool:
        return (self._last_vlm_time is not None
                and now - self._last_vlm_time <= self.cfg.vlm_timeout_s)

    def _expire(self, now: float):
        if (self.active_goal is not None
                and now - self._active_since > self.cfg.goal_ttl_s):
            self.active_goal = None

    def _reset_candidate(self):
        self._cand = None
        self._streak = 0

    def _idle(self, now: float, reason: str) -> Decision:
        self._expire(now)
        action = "keep" if self.active_goal is not None else "explore"
        return Decision(action, reason, None, self._vlm_online(now))

    # ---------- main entry points ----------
    def update(self, obs: Observation, goal: Optional[Goal], now: float,
               obs_time: Optional[float] = None) -> Decision:
        """Call once per VLM observation (with its projected goal, if any)."""
        c = self.cfg
        self._last_vlm_time = now

        if obs_time is not None and now - obs_time > c.max_obs_age_s:
            self._reset_candidate()
            return self._idle(now, "stale observation")
        if not obs.target_visible:
            self._reset_candidate()
            return self._idle(now, "target not visible")
        if obs.confidence < c.min_confidence:
            self._reset_candidate()
            return self._idle(now, "low confidence")
        if goal is None or not goal.ok:
            self._reset_candidate()
            why = goal.reason if goal is not None else "no goal"
            return self._idle(now, f"goal rejected: {why}")

        p = np.asarray(goal.world, dtype=float)
        if (self._cand is not None
                and np.linalg.norm(p - self._cand) <= c.consistency_radius_m):
            self._streak += 1
            w = 1.0 / min(self._streak, 5)       # running average
            self._cand = self._cand + w * (p - self._cand)
        else:
            self._cand = p
            self._streak = 1

        if self._streak < c.confirm_n:
            return self._idle(now, f"confirming ({self._streak}/{c.confirm_n})")

        if (self.active_goal is not None and
                np.linalg.norm(self._cand - self.active_goal) < c.min_goal_shift_m):
            self._active_since = now             # target still seen: refresh
            return Decision("keep", "goal unchanged", None, True)

        self.active_goal = self._cand.copy()
        self._active_since = now
        return Decision("publish_goal", "confirmed", self.active_goal.copy(), True)

    def tick(self, now: float) -> Decision:
        """Call periodically (e.g. 2 Hz), even when no VLM output arrives."""
        self._expire(now)
        online = self._vlm_online(now)
        if self.active_goal is not None:
            why = "holding goal" if online else "VLM offline, holding last goal"
            return Decision("keep", why, None, online)
        why = "no active goal" if online else "VLM offline, exploring"
        return Decision("explore", why, None, online)

    def mark_reached(self):
        """Call when the planner reports the goal has been reached."""
        self.active_goal = None
        self._reset_candidate()
