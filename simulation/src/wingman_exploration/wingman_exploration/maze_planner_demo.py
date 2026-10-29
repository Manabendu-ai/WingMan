"""Offline demo for WingMan's custom maze planner; no ROS or flight commands."""

from .maze_planner import GridMap, MazeExplorationPlanner


DEMO_MAP = (
    "###########",
    "#S....????#",
    "###.#?????#",
    "#...#?????#",
    "#.###?????#",
    "#.....????#",
    "###########",
)


def main() -> None:
    grid, start, marked_entrance = GridMap.from_ascii(DEMO_MAP)
    entrance = marked_entrance if marked_entrance is not None else start
    planner = MazeExplorationPlanner(entrance=entrance, information_gain_weight=1.0)
    current = start

    print("WingMan custom maze planner — OFFLINE TEST ONLY")
    print("Grid: {} columns x {} rows".format(grid.width, grid.height))
    print("Start/entrance: {}".format(current))
    print(
        "Unknown cells are blocked for path traversal; "
        "free frontier cells are exploration goals.\n"
    )

    for decision_number in range(1, 20):
        decision = planner.decide(grid, current)
        print("{}. action={} goal={} reason={}".format(
            decision_number, decision.action, decision.goal, decision.reason
        ))
        if decision.path:
            print("   path ({} cells): {}".format(len(decision.path), list(decision.path)))
        if decision.action == "explore":
            current = decision.goal
            continue
        if decision.action == "return_to_entrance":
            current = entrance
            continue
        if decision.action == "mission_complete":
            print("\nOFFLINE DEMO PASSED: planner returned to entrance after frontier exhaustion.")
            return
        print("\nOFFLINE DEMO STOPPED: planner returned action {!r}.".format(decision.action))
        return
    raise RuntimeError("Demo exceeded decision limit; check exploration-state logic")


if __name__ == "__main__":
    main()
