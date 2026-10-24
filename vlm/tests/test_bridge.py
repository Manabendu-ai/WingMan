import numpy as np

from wingman_vlm.bridge import GoalBridge
from wingman_vlm.projector import Goal
from wingman_vlm.schema import Observation


def obs(visible=True, conf=0.9):
    return Observation(
        scene="corridor", target_visible=visible,
        target_x=500 if visible else None, target_y=500 if visible else None,
        recommended_direction="forward", confidence=conf,
        uncertainties=[], suggested_next_step="approach_target")


def goal(x, y, z=1.0):
    return Goal(True, "ok", np.array([x, y, z]), 4.0)


def publish_at(b, x=5.0, y=0.0, t0=0.0):
    b.update(obs(), goal(x, y), t0)
    return b.update(obs(), goal(x, y), t0 + 1)


def test_low_confidence_is_rejected():
    d = GoalBridge().update(obs(conf=0.3), goal(5, 0), 0)
    assert d.action == "explore" and "confidence" in d.reason


def test_needs_two_consistent_observations():
    b = GoalBridge()
    d1 = b.update(obs(), goal(5, 0), 0)
    assert d1.action != "publish_goal" and "confirming" in d1.reason
    d2 = b.update(obs(), goal(5.1, 0), 1)
    assert d2.action == "publish_goal"
    assert abs(d2.goal[0] - 5.0) < 0.2


def test_inconsistent_goals_reset_the_streak():
    b = GoalBridge()
    b.update(obs(), goal(5, 0), 0)
    assert b.update(obs(), goal(15, 0), 1).action != "publish_goal"
    assert b.update(obs(), goal(15, 0.2), 2).action == "publish_goal"


def test_small_shift_is_not_republished():
    b = GoalBridge()
    publish_at(b)
    d = b.update(obs(), goal(5.3, 0), 2)
    assert d.action == "keep" and "unchanged" in d.reason


def test_big_shift_is_republished_after_confirmation():
    b = GoalBridge()
    publish_at(b)
    assert b.update(obs(), goal(9, 0), 2).action == "keep"
    d = b.update(obs(), goal(9, 0), 3)
    assert d.action == "publish_goal" and abs(d.goal[0] - 9) < 0.2


def test_invalid_depth_goal_is_rejected():
    bad = Goal(False, "no valid depth at target pixel")
    d = GoalBridge().update(obs(), bad, 0)
    assert d.action == "explore" and "depth" in d.reason


def test_stale_observation_is_rejected():
    d = GoalBridge().update(obs(), goal(5, 0), now=10, obs_time=5)
    assert d.action == "explore" and "stale" in d.reason


def test_goal_expires_without_new_sightings():
    b = GoalBridge()
    publish_at(b)                      # active since t=1
    assert b.tick(10).action == "keep"
    assert b.tick(17).action == "explore"   # 16 s > 15 s ttl


def test_vlm_offline_holds_goal_and_flags_it():
    b = GoalBridge()
    publish_at(b)                      # last VLM message at t=1
    d = b.tick(7)                      # 6 s later > 5 s timeout
    assert d.action == "keep" and not d.vlm_online


def test_target_briefly_out_of_view_keeps_goal():
    b = GoalBridge()
    publish_at(b)
    d = b.update(obs(visible=False), None, 2)
    assert d.action == "keep"


def test_mark_reached_clears_goal():
    b = GoalBridge()
    publish_at(b)
    b.mark_reached()
    assert b.tick(2).action == "explore"
