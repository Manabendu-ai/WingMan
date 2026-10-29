import math

import numpy as np

from wingman_exploration.explorer_core import Explorer, PathTracker, build_view
from wingman_exploration.mission_parser import parse_mission


def _corridor_map():
    """20x6 m map at 0.05 m: a horizontal corridor, known only for x<8 m (frontier at x=8)."""
    h, w = 160, 400
    g = np.full((h, w), -1, np.int8)
    g[40:120, 0:160] = 0                  # free
    g[36:40, 0:160] = 100                 # walls
    g[120:124, 0:160] = 100
    g[40:120, 0:4] = 100                  # closed west end
    return g.ravel(), w, h


def test_frontier_is_selected_and_path_is_short():
    data, w, h = _corridor_map()
    view = build_view(data, w, h, 0.05, 0.0, 0.0, inflation=0.45)
    plan = Explorer().plan_explore(view, (2.0, 4.0), 0.0, now=0.0)
    assert plan.kind == "explore"
    assert plan.goal_xy[0] > 6.0                    # the open east end
    assert len(plan.waypoints) <= 3                 # line-of-sight simplified, no staircase


def test_no_frontier_reports_empty():
    data, w, h = _corridor_map()
    g = np.asarray(data).reshape(h, w).copy()
    g[40:120, 160:164] = 100                        # close the east end
    view = build_view(g.ravel(), w, h, 0.05, 0.0, 0.0, inflation=0.45)
    assert Explorer().plan_explore(view, (2.0, 4.0), 0.0, now=0.0).info == "no reachable frontier"


def test_tracker_moves_toward_target_and_stops_in_front_of_wall():
    tr = PathTracker(cruise=0.8)
    tr.set_path([(0.0, 0.0), (5.0, 0.0)])
    vx = 0.0
    for _ in range(40):
        vx, vy, wz, _r = tr.command(0.0, 0.0, 0.0, 0.05, 5.0, 3.0, 3.0)
    assert vx > 0.5
    tr2 = PathTracker(cruise=0.8)
    tr2.set_path([(0.0, 0.0), (5.0, 0.0)])
    for _ in range(40):
        vx, *_ = tr2.command(0.0, 0.0, 0.0, 0.05, 0.4, 3.0, 3.0)
    assert vx < 0.05


def test_wall_repulsion_sign():
    tr = PathTracker()
    tr.set_path([(0.0, 0.0), (5.0, 0.0)])
    for _ in range(40):
        _vx, vy, _wz, _r = tr.command(0.0, 0.0, 0.0, 0.05, 5.0, 0.4, 3.0)   # wall close on the LEFT
    assert vy < 0                                                           # -> move RIGHT


def test_parser():
    m, _ = parse_mission("find the person and come back")
    assert (m["type"], m["target"], m["then"]) == ("find", "person", "return")
    m, _ = parse_mission("find a red box and stay there")
    assert (m["target"], m["then"]) == ("red box", "hover")
    m, _ = parse_mission("go to 4 -10")
    assert m["type"] == "goto" and m["xy"] == [4.0, -10.0]
    assert parse_mission("make me a sandwich")[0] is None
