"""Custom grid-based A* and frontier exploration logic for WingMan.

This module has no ROS imports. It is intended to be tested independently,
then connected to a live occupancy grid and pose through a ROS node.

Grid values follow nav_msgs/OccupancyGrid conventions:
    -1: unknown, 0: free, 1..100: occupied or uncertain (not traversable).
Cells are represented as (x, y), where x is column and y is row.
"""

from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Iterable, List, Optional, Sequence, Set, Tuple

Cell = Tuple[int, int]


@dataclass(frozen=True)
class GridMap:
    """Small immutable occupancy grid used by the planner and offline tests."""

    rows: Tuple[Tuple[int, ...], ...]

    def __post_init__(self) -> None:
        if not self.rows or not self.rows[0]:
            raise ValueError("GridMap must contain at least one cell")
        width = len(self.rows[0])
        if any(len(row) != width for row in self.rows):
            raise ValueError("GridMap rows must all have the same width")

    @classmethod
    def from_rows(cls, rows: Sequence[Sequence[int]]) -> "GridMap":
        """Build from row-major numeric values (-1 unknown, 0 free, >0 blocked)."""
        return cls(tuple(tuple(int(value) for value in row) for row in rows))

    @classmethod
    def from_ascii(cls, lines: Sequence[str]) -> Tuple["GridMap", Cell, Optional[Cell]]:
        """Parse #=wall, ?=unknown, .=free, S=start, E=entrance/goal.

        Returns (grid, start, optional E). Spaces are treated as free cells.
        """
        if not lines:
            raise ValueError("ASCII map cannot be empty")
        width = len(lines[0])
        if width == 0 or any(len(line) != width for line in lines):
            raise ValueError("ASCII map rows must have equal non-zero width")
        rows: List[List[int]] = []
        start: Optional[Cell] = None
        entrance: Optional[Cell] = None
        for y, line in enumerate(lines):
            row: List[int] = []
            for x, char in enumerate(line):
                if char == "#":
                    row.append(100)
                elif char == "?":
                    row.append(-1)
                elif char in (".", "S", "E", " "):
                    row.append(0)
                    if char == "S":
                        if start is not None:
                            raise ValueError("ASCII map must contain at most one S")
                        start = (x, y)
                    if char == "E":
                        if entrance is not None:
                            raise ValueError("ASCII map must contain at most one E")
                        entrance = (x, y)
                else:
                    raise ValueError("Unsupported map character: {!r}".format(char))
            rows.append(row)
        if start is None:
            raise ValueError("ASCII map must contain one S start cell")
        return cls.from_rows(rows), start, entrance

    @property
    def width(self) -> int:
        return len(self.rows[0])

    @property
    def height(self) -> int:
        return len(self.rows)

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.rows[cell[1]][cell[0]] == 0

    def is_unknown(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.rows[cell[1]][cell[0]] == -1

    def neighbors(self, cell: Cell) -> Iterable[Cell]:
        """Yield 4-connected neighbors in deterministic order."""
        x, y = cell
        for neighbor in ((x, y - 1), (x - 1, y), (x + 1, y), (x, y + 1)):
            if self.in_bounds(neighbor):
                yield neighbor


class AStarPlanner:
    """Four-connected A* planner for a known-free occupancy grid."""

    @staticmethod
    def heuristic(a: Cell, b: Cell) -> int:
        """Manhattan distance, matching the four-connected motion model."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> List[Cell]:
        """Return a shortest path from start to goal, inclusive; [] if impossible.

        Only cells with occupancy value exactly 0 are traversable. Unknown and
        occupied cells are never crossed. The path is in grid-cell coordinates.
        """
        if not grid.is_free(start) or not grid.is_free(goal):
            return []
        if start == goal:
            return [start]

        counter = itertools.count()
        open_heap = [(self.heuristic(start, goal), 0, next(counter), start)]
        came_from = {}
        g_cost = {start: 0}
        closed: Set[Cell] = set()

        while open_heap:
            _, cost, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            closed.add(current)

            for neighbor in grid.neighbors(current):
                if not grid.is_free(neighbor) or neighbor in closed:
                    continue
                new_cost = cost + 1
                if new_cost < g_cost.get(neighbor, math.inf):
                    g_cost[neighbor] = new_cost
                    came_from[neighbor] = current
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(
                        open_heap, (priority, new_cost, next(counter), neighbor)
                    )
        return []


def _frontier_cells(grid: GridMap) -> Set[Cell]:
    """Return free cells adjacent (4-connected) to at least one unknown cell."""
    frontiers: Set[Cell] = set()
    for y in range(grid.height):
        for x in range(grid.width):
            cell = (x, y)
            if grid.is_free(cell) and any(grid.is_unknown(n) for n in grid.neighbors(cell)):
                frontiers.add(cell)
    return frontiers


def _frontier_components(frontiers: Set[Cell]) -> List[Set[Cell]]:
    """Split frontier cells into 4-connected components."""
    remaining = set(frontiers)
    components: List[Set[Cell]] = []
    while remaining:
        seed = min(remaining, key=lambda c: (c[1], c[0]))
        remaining.remove(seed)
        component = {seed}
        stack = [seed]
        while stack:
            x, y = stack.pop()
            for neighbor in ((x, y - 1), (x - 1, y), (x + 1, y), (x, y + 1)):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    stack.append(neighbor)
        components.append(component)
    return components


@dataclass(frozen=True)
class PlanningDecision:
    """High-level result returned to the future ROS mission manager."""

    action: str
    goal: Optional[Cell]
    path: Tuple[Cell, ...]
    reason: str


class MazeExplorationPlanner:
    """Global exploration policy over an occupancy grid.

    The planner selects a reachable frontier, retains that goal while moving,
    records a frontier region once reached, and returns to the entrance when no
    new reachable frontiers remain. It does not move a drone or publish ROS data.
    """

    def __init__(self, entrance: Cell, information_gain_weight: float = 1.0) -> None:
        if information_gain_weight < 0:
            raise ValueError("information_gain_weight cannot be negative")
        self.entrance = entrance
        self.information_gain_weight = information_gain_weight
        self.astar = AStarPlanner()
        self.visited_frontier_cells: Set[Cell] = set()
        self.active_goal: Optional[Cell] = None
        self.active_component: Set[Cell] = set()

    def reset(self) -> None:
        """Clear exploration memory while keeping the configured entrance."""
        self.visited_frontier_cells.clear()
        self.active_goal = None
        self.active_component.clear()

    @staticmethod
    def _unknown_count(grid: GridMap, cell: Cell) -> int:
        return sum(1 for neighbor in grid.neighbors(cell) if grid.is_unknown(neighbor))

    @staticmethod
    def _representative(cells: Set[Cell]) -> Cell:
        """Pick a deterministic cell nearest to a component centroid."""
        mean_x = sum(c[0] for c in cells) / float(len(cells))
        mean_y = sum(c[1] for c in cells) / float(len(cells))
        return min(cells, key=lambda c: ((c[0] - mean_x) ** 2 + (c[1] - mean_y) ** 2,
                                        c[1], c[0]))

    def decide(
        self,
        grid: GridMap,
        current: Cell,
        preferred_direction: str = "unknown",
        current_yaw: float = 0.0,
        force_return: bool = False,
    ) -> PlanningDecision:
        """Choose the next frontier or a return-to-entrance path.

        Actions: explore, return_to_entrance, mission_complete, wait, no_path.
        A caller should call this again after receiving updated map/pose data.
        """
        if not grid.is_free(current):
            return PlanningDecision("wait", None, (), "Current pose is not in a known-free cell")
        if not grid.is_free(self.entrance):
            return PlanningDecision("no_path", None, (), "Entrance is not a known-free cell")

        if force_return:
            self.active_goal = None
            self.active_component.clear()
            if current == self.entrance:
                return PlanningDecision("mission_complete", self.entrance, (current,),
                                        "VLM requested return; already at the recorded entrance")
            home_path = self.astar.plan(grid, current, self.entrance)
            if home_path:
                return PlanningDecision("return_to_entrance", self.entrance, tuple(home_path),
                                        "VLM mission decision requested return to the recorded entrance")
            return PlanningDecision("no_path", None, (),
                                    "VLM requested return, but no known-free path to entrance exists")

        # Continue the active route unless the target was reached or became unreachable.
        if self.active_goal is not None:
            if current == self.active_goal:
                self.visited_frontier_cells.update(self.active_component)
                self.active_goal = None
                self.active_component.clear()
            else:
                active_path = self.astar.plan(grid, current, self.active_goal)
                if active_path:
                    return PlanningDecision(
                        "explore", self.active_goal, tuple(active_path),
                        "Continue toward the active frontier"
                    )
                # Map changed or route disappeared: retire this target to avoid a retry loop.
                self.visited_frontier_cells.update(self.active_component)
                self.visited_frontier_cells.add(self.active_goal)
                self.active_goal = None
                self.active_component.clear()

        frontier_set = _frontier_cells(grid)
        components = _frontier_components(frontier_set)
        candidates = []
        for component in components:
            unexplored = component - self.visited_frontier_cells
            if not unexplored:
                continue
            goal = self._representative(unexplored)
            path = self.astar.plan(grid, current, goal)
            if not path:
                continue
            # Prefer short routes but reward frontier regions with more unknown neighbors.
            gain = sum(self._unknown_count(grid, cell) for cell in unexplored)
            score = (len(path) - 1) - self.information_gain_weight * math.log1p(gain)
            dx = float(goal[0] - current[0])
            dy = float(goal[1] - current[1])
            relative = math.atan2(math.sin(math.atan2(dy, dx) - current_yaw),
                                  math.cos(math.atan2(dy, dx) - current_yaw))
            if relative > 0.45:
                direction = "left"
            elif relative < -0.45:
                direction = "right"
            else:
                direction = "forward"
            candidates.append((score, len(path), goal[1], goal[0], goal, path, set(component), direction))

        if candidates:
            preferred_direction = preferred_direction if preferred_direction in {"left", "forward", "right"} else "unknown"
            matching = [candidate for candidate in candidates if candidate[7] == preferred_direction]
            pool = matching if matching else candidates
            _, _, _, _, goal, path, component, selected_direction = min(pool)
            self.active_goal = goal
            self.active_component = component
            if matching:
                reason = f"Selected reachable frontier aligned with VLM direction preference: {preferred_direction}"
            elif preferred_direction != "unknown":
                reason = f"No reachable frontier aligned with VLM preference {preferred_direction}; selected best reachable alternative ({selected_direction})"
            else:
                reason = "Selected reachable frontier using route cost and expected information gain"
            return PlanningDecision("explore", goal, tuple(path), reason)

        # No unexplored reachable frontier: return safely to the recorded entrance.
        if current == self.entrance:
            return PlanningDecision(
                "mission_complete", self.entrance, (current,),
                "No reachable unexplored frontiers remain; already at entrance"
            )
        home_path = self.astar.plan(grid, current, self.entrance)
        if home_path:
            return PlanningDecision(
                "return_to_entrance", self.entrance, tuple(home_path),
                "No reachable unexplored frontiers remain"
            )
        return PlanningDecision(
            "no_path", None, (), "No reachable frontier and no known-free route to entrance"
        )