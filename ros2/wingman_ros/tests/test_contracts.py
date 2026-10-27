import math

import pytest

from wingman_ros.contracts import StateStore, vector3


class Point:
    x, y, z = 1, 2.5, float("nan")


def test_vector3_rejects_non_finite_values():
    assert vector3(Point()) == {"x": 1.0, "y": 2.5, "z": None}


def test_state_store_reports_missing_and_stale_channels():
    store = StateStore(stale_after_s=2.0)
    store.update("telemetry", {"position": {}}, now=10.0)
    health = store.health(now=13.0)
    assert health["stale_channels"] == ["map", "telemetry", "rgbd"]
    assert health["ages_s"]["telemetry"] == 3.0


def test_state_store_rejects_unknown_channels_and_bad_time():
    store = StateStore()
    with pytest.raises(ValueError):
        store.update("commands", {}, now=1)
    with pytest.raises(ValueError):
        store.update("map", {}, now=math.nan)


def test_dashboard_state_is_versioned():
    store = StateStore()
    store.update("map", {"width": 1}, now=1)
    state = store.dashboard_state(now=2)
    assert state["schema"] == "wingman.dashboard.v1"
    assert state["map"]["width"] == 1
    assert state["health"]["version"] == 1
