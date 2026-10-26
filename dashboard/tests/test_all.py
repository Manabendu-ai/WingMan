import json
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from wingman_dashboard.app import create_app
from wingman_dashboard.camera import ReplayCamera
from wingman_dashboard.hub import Hub
from wingman_dashboard.live import LiveLink, PolicyLink
from wingman_dashboard.manager import MissionManager
from wingman_dashboard.mapsource import MockMapSource
from wingman_dashboard.mocks import MockLLM, MockVLM

FRAMES = [Path(f"f{i}.jpg") for i in range(12)]
MISSION = {"mission_id": "m_t", "goal": "find the red box", "target": "red box",
           "steps": [{"action": "takeoff", "altitude_m": 1.0},
                     {"action": "explore", "strategy": "frontier", "until": "target_found"},
                     {"action": "approach_target", "standoff_m": 0.5},
                     {"action": "hover", "duration_s": 5}],
           "limits": {"max_speed_mps": 0.5, "max_altitude_m": 2.0, "max_time_s": 300}, "success_condition": "x"}


def make(**kw):
    hub, m = Hub(), MockMapSource()
    inner = LiveLink(kw.pop("observe", MockVLM(2)), FRAMES, hub, m, tick_real_s=0.0, **kw)
    return hub, m, inner


# ---- map ----
def test_maze_route_and_reveal():
    m = MockMapSource()
    assert len(m.route) > 10
    s0 = m.snapshot()
    assert len(s0["data"]) == s0["width"] * s0["height"]
    known0 = sum(v != -1 for v in s0["data"])
    v0 = m.version
    for _ in range(10):
        m.advance(1.0, True)
    assert sum(v != -1 for v in m.snapshot()["data"]) > known0 and m.version > v0
    m.advance(1.0, False)
    assert m.version == m.snapshot()["version"]


# ---- live link ----
def test_full_run_completes_and_publishes_telemetry():
    hub, m, link = make()
    assert link.send_mission(json.dumps(MISSION))
    st = link.run_until_done()
    assert st["state"] == "complete" and st["target_found"]
    assert hub.view()["telemetry"]["state"] == "complete" and "pose" in st
    assert link.running is False and len(m.trail) > 0


def test_abort_interrupts_a_running_mission():
    holder = {}
    vlm = MockVLM(50)                       # never sees the target

    def observe(path, task):
        if vlm.n == 2:
            holder["link"].request_abort()
        return vlm(path, task)
    observe.reset = vlm.reset
    hub, m, link = make(observe=observe)
    holder["link"] = link
    link.send_mission(json.dumps(MISSION))
    st = link.run_until_done()
    assert st["state"] == "aborted" and st["failure_reason"] == "operator abort"


def test_invalid_mission_rejected():
    _, _, link = make()
    assert link.send_mission('{"mission_id": "x"}') is False
    with pytest.raises(RuntimeError):
        link.run_until_done()


def test_takeoff_failure_is_not_replanned():
    hub, m, inner = make(break_first_takeoff=True)
    link = PolicyLink(inner)
    link.send_mission(json.dumps(MISSION))
    st = link.run_until_done()
    assert st["state"] == "aborted" and st["policy_halt"] and "not auto-replanned" in st["failure_reason"]


def test_navigation_failure_still_reported_as_failed():
    hub, m, inner = make(observe=MockVLM(10 ** 6))
    link = PolicyLink(inner)
    mis = dict(MISSION, limits={"max_speed_mps": 0.5, "max_altitude_m": 2.0, "max_time_s": 30})
    link.send_mission(json.dumps(mis))
    st = link.run_until_done()
    assert st["state"] == "failed" and "time limit" in st["failure_reason"]


def test_camera_returns_jpeg_with_and_without_frame():
    hub = Hub()
    cam = ReplayCamera(hub)
    assert cam.jpeg()[:2] == b"\xff\xd8"
    hub.set_frame(Path("nope.jpg"), {"target_visible": True, "x": 500, "y": 500, "confidence": 0.9})
    assert cam.jpeg()[:2] == b"\xff\xd8"


def test_mock_llm_extracts_target():
    d = json.loads(MockLLM().complete([{"role": "user", "content": "Operator request:\n<<<\ntake off, find the blue door and hover\n>>>"}]))
    assert d["target"] == "blue door"


# ---- API ----
TOKEN = "secret-token"


@pytest.fixture
def api():
    hub, m, inner = make()
    mgr = MissionManager(MockLLM(), PolicyLink(inner), hub, approval_timeout_s=5)
    return TestClient(create_app(mgr, ReplayCamera(hub), m, TOKEN)), mgr


H = {"x-token": TOKEN}


def wait_for(client, pred, timeout=10):
    end = time.time() + timeout
    while time.time() < end:
        s = client.get("/api/state", headers=H).json()
        if pred(s):
            return s
        time.sleep(0.02)
    raise AssertionError(f"timeout; last phase={s['phase']}")


def test_auth_required(api):
    c, _ = api
    assert c.get("/api/state").status_code == 401
    assert c.get("/api/state", headers={"x-token": "bad"}).status_code == 401
    assert c.get("/api/state?token=" + TOKEN).status_code == 200
    assert c.post("/api/abort").status_code == 401
    assert c.get("/api/health").status_code == 200


def test_submit_approve_run_success(api):
    c, _ = api
    assert c.post("/api/mission", json={"request": "find the red box"}, headers=H).status_code == 200
    s = wait_for(c, lambda s: s["phase"] == "awaiting_approval")
    assert s["pending"]["kind"] == "approval" and "PROPOSED MISSION" in s["pending"]["detail"]
    assert c.post("/api/mission", json={"request": "again"}, headers=H).status_code == 409
    assert c.post("/api/answer", json={"text": "hi"}, headers=H).status_code == 409
    assert c.post("/api/approve", json={"approve": True}, headers=H).status_code == 200
    s = wait_for(c, lambda s: s["phase"] == "done")
    assert s["result"]["outcome"] == "success"
    assert c.get("/api/map", headers=H).json()["width"] > 0


def test_reject_and_validation(api):
    c, _ = api
    assert c.post("/api/mission", json={"request": ""}, headers=H).status_code == 422
    assert c.post("/api/mission", json={"request": "x" * 600}, headers=H).status_code == 422
    c.post("/api/mission", json={"request": "find the red box"}, headers=H)
    wait_for(c, lambda s: s["phase"] == "awaiting_approval")
    c.post("/api/approve", json={"approve": False}, headers=H)
    assert wait_for(c, lambda s: s["phase"] == "done")["result"]["outcome"] == "rejected"


def test_abort_while_awaiting_approval(api):
    c, _ = api
    c.post("/api/mission", json={"request": "find the red box"}, headers=H)
    wait_for(c, lambda s: s["phase"] == "awaiting_approval")
    c.post("/api/abort", headers=H)
    assert wait_for(c, lambda s: s["phase"] == "done")["result"]["outcome"] == "rejected"


def test_abort_while_executing(api):
    c, mgr = api
    mgr.link.inner.tick_real_s = 0.05
    c.post("/api/mission", json={"request": "find the red box"}, headers=H)
    wait_for(c, lambda s: s["phase"] == "awaiting_approval")
    c.post("/api/approve", json={"approve": True}, headers=H)
    wait_for(c, lambda s: s["phase"] == "executing")
    c.post("/api/abort", headers=H)
    assert wait_for(c, lambda s: s["phase"] == "done")["result"]["outcome"] == "aborted"


def test_websocket_streams_state_and_map(api):
    c, _ = api
    with c.websocket_connect("/ws?token=" + TOKEN) as ws:
        msg = ws.receive_json()
        assert msg["phase"] == "idle" and "map" in msg
    with pytest.raises(Exception):
        with c.websocket_connect("/ws?token=bad") as ws:
            ws.receive_json()


def test_camera_stream_route(api):
    c, _ = api
    assert c.get("/camera.mjpg").status_code == 401


def test_docs_off_by_default_and_on_with_flag():
    hub, m, inner = make()
    mgr = MissionManager(MockLLM(), PolicyLink(inner), hub)
    off = TestClient(create_app(mgr, ReplayCamera(hub), m, TOKEN))
    on = TestClient(create_app(mgr, ReplayCamera(hub), m, TOKEN, docs=True))
    assert off.get("/docs").status_code == 404
    assert on.get("/docs").status_code == 200
    spec = on.get("/openapi.json").json()
    assert "APIKeyHeader" in str(spec["components"]["securitySchemes"])
    assert on.get("/api/state").status_code == 401        # docs open, API still locked
