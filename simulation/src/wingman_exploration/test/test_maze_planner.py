"""Unit tests for WingMan's custom global maze planner."""

from wingman_exploration.maze_planner import (
    AStarPlanner,
    GridMap,
    MazeExplorationPlanner,
)


def test_astar_finds_shortest_path_on_open_grid():
    grid = GridMap.from_rows([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
    path = AStarPlanner().plan(grid, (0, 0), (2, 2))
    assert path[0] == (0, 0)
    assert path[-1] == (2, 2)
    assert len(path) == 5  # 4 moves + the initial cell
    assert all(grid.is_free(cell) for cell in path)
    assert all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
               for a, b in zip(path, path[1:]))


def test_astar_routes_around_a_wall():
    grid = GridMap.from_rows([
        [0, 0, 0],
        [100, 100, 0],
        [0, 0, 0],
    ])
    path = AStarPlanner().plan(grid, (0, 0), (0, 2))
    assert path
    assert path[0] == (0, 0) and path[-1] == (0, 2)
    assert all(grid.is_free(cell) for cell in path)


def test_astar_rejects_unknown_and_unreachable_goal():
    grid = GridMap.from_rows([[0, 100, 0], [0, -1, 0]])
    assert AStarPlanner().plan(grid, (0, 0), (2, 0)) == []
    assert AStarPlanner().plan(grid, (0, 0), (1, 1)) == []


def test_grid_parser_returns_start_and_entrance():
    grid, start, entrance = GridMap.from_ascii(("#####", "#S.E#", "#####"))
    assert (grid.width, grid.height) == (5, 3)
    assert start == (1, 1)
    assert entrance == (3, 1)


def test_explorer_selects_frontier_then_returns_to_entrance():
    # One known corridor cell touching one unknown cell.
    grid, start, entrance = GridMap.from_ascii(("#####", "#S.?#", "#####"))
    planner = MazeExplorationPlanner(entrance=start)

    first = planner.decide(grid, start)
    assert first.action == "explore"
    assert first.goal == (2, 1)
    assert first.path[0] == start
    assert first.path[-1] == first.goal

    reached_frontier = planner.decide(grid, first.goal)
    assert reached_frontier.action == "return_to_entrance"
    assert reached_frontier.goal == start
    assert reached_frontier.path[-1] == start

    home = planner.decide(grid, start)
    assert home.action == "mission_complete"


def test_non_free_current_pose_waits_instead_of_planning():
    grid = GridMap.from_rows([[0, 100], [0, 0]])
    planner = MazeExplorationPlanner(entrance=(0, 0))
    decision = planner.decide(grid, (1, 0))
    assert decision.action == "wait"
    assert not decision.path


def test_vlm_direction_preference_biases_frontier_selection():
    # Two unknown frontiers are reachable from the central start. At yaw=0,
    # positive grid-y is classified as left and negative grid-y as right.
    grid = GridMap.from_rows([
        [100, 100, 100, -1, 100, 100, 100],
        [100, 100, 100, 0, 100, 100, 100],
        [100, 0, 0, 0, 0, 0, 100],
        [100, 100, 100, 0, 100, 100, 100],
        [100, 100, 100, -1, 100, 100, 100],
    ])
    planner = MazeExplorationPlanner(entrance=(3, 2))
    decision = planner.decide(grid, (3, 2), preferred_direction="left", current_yaw=0.0)
    assert decision.action == "explore"
    assert decision.goal == (3, 3)
    assert "VLM direction preference" in decision.reason


def test_vlm_return_request_overrides_frontier_exploration():
    grid = GridMap.from_rows([
        [100, 100, 100, -1, 100],
        [100, 100, 100, 0, 100],
        [100, 0, 0, 0, 100],
        [100, 100, 100, 0, 100],
        [100, 100, 100, -1, 100],
    ])
    planner = MazeExplorationPlanner(entrance=(3, 2))
    decision = planner.decide(grid, (3, 3), force_return=True)
    assert decision.action == "return_to_entrance"
    assert decision.path[0] == (3, 3)
    assert decision.path[-1] == (3, 2)
