import copy
import json

import numpy as np
import pytest
from pydantic import ValidationError

from wingman_onboard.executor import (ExecutorConfig, Feedback,
                                      MissionExecutor, State)
from wingman_vlm.bridge import GoalBridge
from wingman_vlm.projector import Goal
from wingman_vlm.schema import Observation

DEFAULT_STEPS = [
    {"action": "takeoff", "altitude_m": 1.0},
    {"action": "explore", "strategy": "wall_following", "until": "target_found"},
    {"action": "approach_target", "standoff_m": 0.5},
    {"action": "hover", "duration_s": 5},
]


class FakeDrone:
    def __init__(self):
        self.calls = []

    def set_limits(self, v, a):
        self.calls.append(("set_limits", v, a))

    def takeoff(self, a):
        self.calls.append(("takeoff", a))

    def explore(self, s):
        self.calls.append(("explore", s))

    def go_to(self, x, y, z):
        self.calls.append(("go_to", x, y, z))

    def hover(self):
        self.calls.append(("hover",))

    def return_to_start(self):
        self.calls.append(("return_to_start",))

    def land(self):
        self.calls.append(("land",))


def mission_dict(steps=None, target="maze exit", max_time_s=300):
    return {
        "mission_id": "m_t",
        "goal": "reach the maze exit",
        "target": target,
        "steps": copy.deepcopy(steps or DEFAULT_STEPS),
        "limits": {"max_speed_mps": 0.5, "max_altitude_m": 2.0,
                   "max_time_s": max_time_s},
        "success_condition": "drone is at the exit",
    }


def make(steps=None, target="maze exit", max_time_s=300, cfg=None):
    drone = FakeDrone()
    ex = MissionExecutor(drone, GoalBridge(), cfg)
    ex.load(mission_dict(steps, target, max_time_s))
    return ex, drone


def obs(visible=True, conf=0.9):
    return Observation(
        scene="corridor", target_visible=visible,
        target_x=500 if visible else None, target_y=500 if visible else None,
        recommended_direction="forward", confidence=conf,
        uncertainties=[], suggested_next_step="approach_target")


def goal(x, y, z=1.0):
    return Goal(True, "ok", np.array([x, y, z]), 4.0)


def to_explore(ex):
    ex.start(0)
    ex.tick(1, Feedback(altitude_m=1.0))


def to_approach(ex):
    to_explore(ex)
    ex.on_observation(obs(), goal(5, 0), 2)
    ex.on_observation(obs(), goal(5.1, 0), 3)    # confirmed -> go_to sent at t=3


def goto_calls(d):
    return [c for c in d.calls if c[0] == "go_to"]


def test_start_applies_limits_and_takes_off():
    ex, d = make()
    ex.start(0)
    assert d.calls[0] == ("set_limits", 0.5, 2.0)
    assert ("takeoff", 1.0) in d.calls
    assert ex.state == State.EXECUTING


def test_takeoff_completes_at_altitude():
    ex, d = make()
    ex.start(0)
    ex.tick(0.5, Feedback(altitude_m=0.4))
    assert ex.index == 0
    ex.tick(1.0, Feedback(altitude_m=1.0))
    assert ex.index == 1 and ("explore", "wall_following") in d.calls


def test_takeoff_timeout_fails_and_holds():
    ex, d = make()
    ex.start(0)
    ex.tick(31, Feedback(altitude_m=0.2))
    assert ex.state == State.FAILED and "takeoff" in ex.failure_reason
    assert d.calls[-1] == ("hover",)


def test_full_mission_flow():
    ex, d = make()
    to_approach(ex)
    assert ex.index == 2
    g = goto_calls(d)[-1]
    assert abs(g[1] - 5.0) < 0.2
    ex.tick(5, Feedback(goal_reached=True))
    assert ex.index == 3 and d.calls[-1] == ("hover",)
    ex.tick(9, Feedback())
    assert ex.state == State.EXECUTING
    ex.tick(10.1, Feedback())
    assert ex.state == State.COMPLETE


def test_low_confidence_does_not_finish_explore():
    ex, _ = make()
    to_explore(ex)
    ex.on_observation(obs(conf=0.3), goal(5, 0), 2)
    ex.on_observation(obs(conf=0.3), goal(5, 0), 3)
    assert ex.index == 1


def test_single_observation_is_not_enough():
    ex, _ = make()
    to_explore(ex)
    ex.on_observation(obs(), goal(5, 0), 2)
    assert ex.index == 1


def test_goal_reached_ignored_during_grace():
    ex, _ = make()
    to_approach(ex)
    ex.tick(3.5, Feedback(goal_reached=True))   # 0.5 s after go_to: stale
    assert ex.index == 2
    ex.tick(4.2, Feedback(goal_reached=True))
    assert ex.index == 3


def test_vlm_request_only_while_searching():
    ex, _ = make()
    assert ex.vlm_request() is None             # not started
    ex.start(0)
    assert ex.vlm_request() is None             # takeoff
    ex.tick(1, Feedback(altitude_m=1.0))
    req = ex.vlm_request()
    assert req.task == "find the maze exit" and req.standoff_m == 0.5


def test_explore_uses_next_steps_standoff():
    steps = copy.deepcopy(DEFAULT_STEPS)
    steps[2]["standoff_m"] = 1.2
    ex, _ = make(steps)
    to_explore(ex)
    assert ex.vlm_request().standoff_m == 1.2


def test_observations_ignored_when_not_searching():
    ex, _ = make()
    ex.start(0)                                  # still in takeoff
    assert ex.on_observation(obs(), goal(5, 0), 0.5) is None


def test_mission_time_limit():
    ex, d = make(max_time_s=30)
    to_explore(ex)
    ex.tick(31, Feedback())
    assert ex.state == State.FAILED and "time limit" in ex.failure_reason
    assert d.calls[-1] == ("hover",)


def test_abort_holds_and_stops_everything():
    ex, d = make()
    ex.start(0)
    ex.abort(2, "operator abort")
    assert ex.state == State.ABORTED and d.calls[-1] == ("hover",)
    n = len(d.calls)
    ex.tick(3, Feedback(altitude_m=1.0))
    assert len(d.calls) == n


def test_approach_without_prior_explore_searches_first():
    steps = [{"action": "takeoff", "altitude_m": 1.0},
             {"action": "approach_target", "standoff_m": 0.5},
             {"action": "hover", "duration_s": 5}]
    ex, d = make(steps)
    to_explore(ex)                               # takeoff done -> approach step
    assert d.calls[-1] == ("explore", "frontier")
    ex.on_observation(obs(), goal(5, 0), 2)
    ex.on_observation(obs(), goal(5, 0), 3)
    assert goto_calls(d) and ex.index == 1


def test_approach_follows_a_moved_target():
    ex, d = make()
    to_approach(ex)
    ex.on_observation(obs(), goal(9, 0), 4)
    ex.on_observation(obs(), goal(9, 0), 5)
    gs = goto_calls(d)
    assert len(gs) == 2 and abs(gs[-1][1] - 9.0) < 0.2
    assert ex.index == 2


def test_return_and_land_flow():
    steps = [{"action": "takeoff", "altitude_m": 1.0},
             {"action": "explore", "strategy": "frontier", "until": "time_limit"},
             {"action": "return_to_start"},
             {"action": "land"}]
    ex, d = make(steps, target=None)
    to_explore(ex)                               # explore entered at t=1
    ex.tick(100, Feedback())
    assert ex.index == 1
    ex.tick(121, Feedback())                     # 120 s of exploring
    assert ex.index == 2 and d.calls[-1] == ("return_to_start",)
    ex.tick(130, Feedback(home_reached=True))
    assert ex.index == 3 and d.calls[-1] == ("land",)
    ex.tick(135, Feedback(landed=True))
    assert ex.state == State.COMPLETE


def test_approach_timeout_fails():
    ex, d = make()
    to_approach(ex)                              # approach entered at t=3
    ex.tick(94, Feedback())
    assert ex.state == State.FAILED and "approach" in ex.failure_reason


def test_status_is_json_serializable():
    ex, _ = make()
    to_approach(ex)
    s = ex.status(4)
    json.dumps(s)
    assert s["state"] == "executing" and s["step_action"] == "approach_target"
    assert s["target_found"] and s["vlm_online"] is True
    assert s["last_observation"]["target_visible"] is True
    assert s["steps_total"] == 4 and s["mission_id"] == "m_t"


def test_load_refused_while_executing():
    ex, _ = make()
    ex.start(0)
    with pytest.raises(RuntimeError):
        ex.load(mission_dict())


def test_load_revalidates_dicts():
    ex = MissionExecutor(FakeDrone())
    bad = mission_dict()
    bad["limits"]["max_speed_mps"] = 9.0
    with pytest.raises(ValidationError):
        ex.load(bad)


def test_start_requires_loaded_mission():
    with pytest.raises(RuntimeError):
        MissionExecutor(FakeDrone()).start(0)


def test_fail_action_land_is_configurable():
    ex, d = make(cfg=ExecutorConfig(on_fail="land"))
    ex.start(0)
    ex.tick(31, Feedback(altitude_m=0.0))
    assert ex.state == State.FAILED and d.calls[-1] == ("land",)


def test_status_vlm_online_is_none_when_vlm_not_needed():
    ex, _ = make()
    assert ex.status(0)["vlm_online"] is None
