import json
from pathlib import Path

import pytest

from wingman_onboard.sim import FrameFeed, InProcessLink, default_frames
from wingman_vlm.client import Result
from wingman_vlm.schema import Observation, fallback_observation


def quiet(*a, **k):
    pass


def mission_json(max_time_s=300):
    return json.dumps({
        "mission_id": "m_t", "goal": "find the red box", "target": "red box",
        "steps": [{"action": "takeoff", "altitude_m": 1.0},
                  {"action": "explore", "strategy": "frontier",
                   "until": "target_found"},
                  {"action": "approach_target", "standoff_m": 0.5},
                  {"action": "hover", "duration_s": 5}],
        "limits": {"max_speed_mps": 0.5, "max_altitude_m": 2.0,
                   "max_time_s": max_time_s},
        "success_condition": "hovering over the red box"})


def fake_observe(visible_after=2):
    calls = {"n": 0}

    def observe(path, task):
        calls["n"] += 1
        vis = calls["n"] > visible_after
        obs = Observation(
            scene="corridor", target_visible=vis,
            target_x=500 if vis else None, target_y=500 if vis else None,
            recommended_direction="forward", confidence=0.9, uncertainties=[],
            suggested_next_step="approach_target" if vis else "search_again")
        return Result(obs, 0.1, 0, True, "")
    return observe


def failing_observe(path, task):
    return Result(fallback_observation("boom"), 0.1, 2, False, "")


def make_link(observe, **kw):
    return InProcessLink(observe, [Path("a.jpg"), Path("b.jpg")],
                         say=quiet, **kw)


def test_full_run_completes_with_fake_vlm():
    link = make_link(fake_observe())
    assert link.send_mission(mission_json())
    status = link.run_until_done()
    assert status["state"] == "complete" and status["target_found"]


def test_invalid_mission_is_rejected_by_the_drone_side():
    link = make_link(fake_observe())
    assert link.send_mission('{"mission_id": "x"}') is False
    with pytest.raises(RuntimeError):
        link.run_until_done()


def test_stuck_takeoff_fails_once_then_works():
    link = make_link(fake_observe(), break_first_takeoff=True)
    assert link.send_mission(mission_json())
    first = link.run_until_done()
    assert first["state"] == "failed" and "takeoff" in first["failure_reason"]
    assert link.send_mission(mission_json())
    assert link.run_until_done()["state"] == "complete"


def test_failed_vlm_calls_are_skipped_not_trusted():
    link = make_link(failing_observe)
    assert link.send_mission(mission_json(max_time_s=30))
    status = link.run_until_done()
    assert status["state"] == "failed" and "time limit" in status["failure_reason"]
    assert status["target_found"] is False


def test_frame_feed_repeats_the_last_frame():
    a, b = Path("a"), Path("b")
    feed = FrameFeed([a, b])
    assert [feed.next() for _ in range(4)] == [a, b, b, b]


def test_frame_feed_needs_frames():
    with pytest.raises(ValueError):
        FrameFeed([])


def test_default_frames_order_and_doubling(tmp_path):
    for name in ("indoor1.jpg", "redbox1.webp", "notes.txt"):
        (tmp_path / name).write_text("x")
    names = [p.name for p in default_frames(tmp_path)]
    assert names == ["indoor1.jpg", "redbox1.webp", "redbox1.webp"]
