"""Offline stand-ins so the dashboard runs with no LLM key and no VLM server."""
import json
import re
from types import SimpleNamespace

from wingman_vlm.schema import Observation


class MockLLM:
    last_model = "mock"

    def complete(self, messages):
        text = messages[-1]["content"]
        m = re.search(r"find (?:the |a |an )?([A-Za-z0-9 _-]{2,30}?)(?: and| then|,|\.|$|\n|>)", text, re.I)
        target = re.sub(r"[^A-Za-z0-9 _-]", "", m.group(1)).strip() if m else "red box"
        return json.dumps({
            "mission_id": "m_mock", "goal": f"find the {target} and hover over it", "target": target,
            "steps": [{"action": "takeoff", "altitude_m": 1.0},
                      {"action": "explore", "strategy": "frontier", "until": "target_found"},
                      {"action": "approach_target", "standoff_m": 0.5},
                      {"action": "hover", "duration_s": 5.0}],
            "limits": {"max_speed_mps": 0.5, "max_altitude_m": 2.0, "max_time_s": 300},
            "success_condition": f"the drone is hovering over the {target}"})


class MockVLM:
    """Sees nothing for `visible_after` calls, then sees the target."""

    def __init__(self, visible_after: int = 6):
        self.visible_after, self.n = visible_after, 0

    def reset(self):
        self.n = 0

    def __call__(self, path, task):
        self.n += 1
        vis = self.n > self.visible_after
        obs = Observation(scene="corridor", target_visible=vis,
                          target_x=500 if vis else None, target_y=500 if vis else None,
                          recommended_direction="forward", confidence=0.9, uncertainties=[],
                          suggested_next_step="approach_target" if vis else "search_again")
        return SimpleNamespace(ok=True, obs=obs, latency=0.0, attempts=1)
