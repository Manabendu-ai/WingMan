#!/usr/bin/env python3
"""Read-only ROS adapter for WingMan's custom maze planner.

Consumes an OccupancyGrid and map->base TF, then publishes a planned route,
goal marker, and JSON decision for visualization/debugging only. It does NOT
send Nav2 actions, publish cmd_vel, call takeoff/land services, or control PX4.

Start exploring explicitly with:
  ros2 service call /wingman/custom_planner/start std_srvs/srv/Trigger {}
Pause planning with:
  ros2 service call /wingman/custom_planner/stop std_srvs/srv/Trigger {}
"""

import json
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from wingman_exploration.maze_planner import GridMap, MazeExplorationPlanner


Cell = Tuple[int, int]


def _yaw_from_quaternion(q) -> float:
    """Return planar yaw from a ROS quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def world_to_cell(grid_msg: OccupancyGrid, x: float, y: float) -> Optional[Cell]:
    """Convert a world/map-frame point into the occupancy grid's cell indices."""
    resolution = float(grid_msg.info.resolution)
    if not math.isfinite(resolution) or resolution <= 0.0:
        return None

    origin = grid_msg.info.origin
    dx = x - origin.position.x
    dy = y - origin.position.y
    origin_yaw = _yaw_from_quaternion(origin.orientation)

    # Rotate by the inverse of the occupancy-grid origin orientation.
    local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
    local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
    cell_x = math.floor(local_x / resolution)
    cell_y = math.floor(local_y / resolution)

    if not (0 <= cell_x < grid_msg.info.width and 0 <= cell_y < grid_msg.info.height):
        return None
    return int(cell_x), int(cell_y)


def cell_to_world(grid_msg: OccupancyGrid, cell: Cell) -> Tuple[float, float]:
    """Convert a grid cell into the world coordinates of its center."""
    resolution = float(grid_msg.info.resolution)
    origin = grid_msg.info.origin
    local_x = (cell[0] + 0.5) * resolution
    local_y = (cell[1] + 0.5) * resolution
    origin_yaw = _yaw_from_quaternion(origin.orientation)
    world_x = origin.position.x + math.cos(origin_yaw) * local_x - math.sin(origin_yaw) * local_y
    world_y = origin.position.y + math.sin(origin_yaw) * local_x + math.cos(origin_yaw) * local_y
    return world_x, world_y


def occupancy_to_grid(
    grid_msg: OccupancyGrid,
    free_threshold: int = 20,
    obstacle_inflation_radius_m: float = 0.25,
) -> GridMap:
    """Normalize OccupancyGrid values for conservative A* traversal.

    -1 remains unknown; cells 0..free_threshold become free; all other known
    values become blocked. Treating uncertain cells as blocked is deliberate.
    """
    width = int(grid_msg.info.width)
    height = int(grid_msg.info.height)
    expected = width * height
    if width <= 0 or height <= 0 or len(grid_msg.data) != expected:
        raise ValueError(
            f"Invalid OccupancyGrid dimensions/data: {width}x{height}, "
            f"received {len(grid_msg.data)} values"
        )

    rows = []
    for row_index in range(height):
        start = row_index * width
        raw_row = grid_msg.data[start:start + width]
        normalized = []
        for raw in raw_row:
            value = int(raw)
            if value < 0:
                normalized.append(-1)
            elif value <= free_threshold:
                normalized.append(0)
            else:
                normalized.append(100)
        rows.append(normalized)

    # Inflate known occupied cells by the drone's safety radius. Unknown cells
    # remain unknown and non-traversable; inflation never turns unknown into free.
    inflation = max(0.0, float(obstacle_inflation_radius_m))
    if inflation > 0.0:
        radius_cells = int(math.ceil(inflation / float(grid_msg.info.resolution)))
        occupied = [(x, y) for y in range(height) for x in range(width) if rows[y][x] == 100]
        for ox, oy in occupied:
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy > radius_cells * radius_cells:
                        continue
                    x, y = ox + dx, oy + dy
                    if 0 <= x < width and 0 <= y < height and rows[y][x] == 0:
                        rows[y][x] = 100
    return GridMap.from_rows(rows)


class MazePlannerNode(Node):
    """Visualize custom A* exploration decisions without controlling the drone."""

    def __init__(self) -> None:
        super().__init__('wingman_custom_maze_planner')

        self.declare_parameter('map_topic', '/rtabmap/map')
        self.declare_parameter('map_frame_fallback', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('free_threshold', 20)
        self.declare_parameter('obstacle_inflation_radius_m', 0.25)
        self.declare_parameter('information_gain_weight', 1.0)
        self.declare_parameter('planning_rate_hz', 2.0)
        self.declare_parameter('decision_topic', '/wingman/vlm/mission_decision')
        self.declare_parameter('require_vlm_decision', True)

        self.map_msg: Optional[OccupancyGrid] = None
        self.started = False
        self.planner: Optional[MazeExplorationPlanner] = None
        self.entrance_world: Optional[Tuple[float, float]] = None
        self._last_map_signature = None
        self._last_geometry_warning = None
        self.vlm_decision_received = False
        self.vlm_action = 'hold'
        self.preferred_direction = 'unknown'
        self.person_detected = 'unknown'
        self.operator_started = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            map_qos,
        )
        self.path_pub = self.create_publisher(Path, '/wingman/planner_debug/path', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/wingman/planner_debug/goal', 10)
        self.marker_pub = self.create_publisher(Marker, '/wingman/planner_debug/goal_marker', 10)
        self.decision_pub = self.create_publisher(String, '/wingman/planner_debug/decision', 10)
        self.vlm_sub = self.create_subscription(
            String,
            str(self.get_parameter('decision_topic').value),
            self._on_vlm_decision,
            10,
        )
        self.start_srv = self.create_service(
            Trigger, '/wingman/custom_planner/start', self._start_cb
        )
        self.stop_srv = self.create_service(
            Trigger, '/wingman/custom_planner/stop', self._stop_cb
        )

        rate = max(0.2, float(self.get_parameter('planning_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            'Custom A* maze planner ready. It only publishes a path; it does not command PX4. '
            'It waits for a validated VLM mission decision and explicit start service.'
        )

    def _frame(self) -> str:
        if self.map_msg is not None and self.map_msg.header.frame_id:
            return self.map_msg.header.frame_id
        return str(self.get_parameter('map_frame_fallback').value)

    @staticmethod
    def _geometry_signature(msg: OccupancyGrid):
        info = msg.info
        origin = info.origin
        return (
            int(info.width), int(info.height), round(float(info.resolution), 9),
            round(float(origin.position.x), 6), round(float(origin.position.y), 6),
            round(_yaw_from_quaternion(origin.orientation), 6),
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        old_msg = self.map_msg
        if old_msg is not None and self.planner is not None:
            old_sig = self._geometry_signature(old_msg)
            new_sig = self._geometry_signature(msg)
            if old_sig != new_sig:
                self._remap_memory(old_msg, msg)
        self.map_msg = msg
        self._last_map_signature = self._geometry_signature(msg)

    def _remap_memory(self, old_msg: OccupancyGrid, new_msg: OccupancyGrid) -> None:
        """Preserve exploration history if RTAB-Map expands/shifts its grid."""
        if self.planner is None:
            return
        visited_world = [cell_to_world(old_msg, c) for c in self.planner.visited_frontier_cells]
        active_goal_world = (
            cell_to_world(old_msg, self.planner.active_goal)
            if self.planner.active_goal is not None else None
        )
        active_component_world = [
            cell_to_world(old_msg, c) for c in self.planner.active_component
        ]

        if self.entrance_world is None:
            return
        new_entrance = world_to_cell(new_msg, *self.entrance_world)
        if new_entrance is None:
            self.get_logger().warning(
                'Map geometry changed and no longer contains the recorded entrance; '
                'planner paused until the entrance is inside the map.'
            )
            self.started = False
            self.planner = None
            return

        new_planner = MazeExplorationPlanner(
            new_entrance,
            information_gain_weight=float(
                self.get_parameter('information_gain_weight').value
            ),
        )
        new_planner.visited_frontier_cells = {
            cell for xy in visited_world
            if (cell := world_to_cell(new_msg, *xy)) is not None
        }
        if active_goal_world is not None:
            new_planner.active_goal = world_to_cell(new_msg, *active_goal_world)
        new_planner.active_component = {
            cell for xy in active_component_world
            if (cell := world_to_cell(new_msg, *xy)) is not None
        }
        self.planner = new_planner
        self.get_logger().info('Occupancy grid geometry changed; reprojected planner state.')

    def _lookup_pose(self):
        if self.map_msg is None:
            return None
        target_frame = self._frame()
        base_frame = str(self.get_parameter('base_frame').value)
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame, base_frame, rclpy.time.Time()
            )
        except Exception as exc:  # TF can be unavailable during startup/reset.
            self.get_logger().warning(
                f'Waiting for TF {target_frame} -> {base_frame}: {exc}',
                throttle_duration_sec=3.0,
            )
            return None

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        return float(t.x), float(t.y), float(t.z), _yaw_from_quaternion(q)

    def _on_vlm_decision(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            if not isinstance(obj, dict):
                raise ValueError('decision must be a JSON object')
            action = obj.get('mission_action')
            direction = obj.get('preferred_direction', 'unknown')
            person = obj.get('person_detected', 'unknown')
            if action not in {'explore', 'return_to_entrance', 'hold'}:
                raise ValueError(f'unsupported mission_action: {action!r}')
            if direction not in {'left', 'forward', 'right', 'unknown'}:
                direction = 'unknown'
            self.vlm_action = action
            self.preferred_direction = direction
            self.person_detected = person if person in (True, False, 'unknown') else 'unknown'
            self.vlm_decision_received = True
            self.get_logger().info(
                f'VLM decision received: action={action}, direction={direction}, '
                f'person_detected={self.person_detected}'
            )
            if action == 'hold':
                self.started = False
                self._publish_empty_path()
                self.get_logger().warning('VLM action is hold; path publication paused')
            elif self.operator_started:
                # Resume route publication only after the operator has started the mission once.
                self.started = True
        except Exception as exc:
            self.vlm_action = 'hold'
            self.preferred_direction = 'unknown'
            self.vlm_decision_received = False
            self.started = False
            self._publish_empty_path()
            self.get_logger().error(f'Invalid VLM decision; planner paused: {exc}')

    def _start_cb(self, _request, response):
        if bool(self.get_parameter('require_vlm_decision').value) and not self.vlm_decision_received:
            response.success = False
            response.message = 'No valid VLM decision received yet; planner remains stopped.'
            return response
        if self.vlm_action == 'hold':
            response.success = False
            response.message = 'VLM action is hold; planner remains stopped.'
            return response
        if self.map_msg is None:
            response.success = False
            response.message = 'No OccupancyGrid received yet; planner remains stopped.'
            return response
        pose = self._lookup_pose()
        if pose is None:
            response.success = False
            response.message = 'No fresh map-to-base transform; planner remains stopped.'
            return response

        if self.planner is not None:
            self.started = True
            self.operator_started = True
            response.success = True
            response.message = 'Custom planner resumed; original entrance retained.'
            return response

        grid = occupancy_to_grid(
            self.map_msg,
            int(self.get_parameter('free_threshold').value),
            float(self.get_parameter('obstacle_inflation_radius_m').value),
        )
        if self.entrance_world is None:
            start_cell = world_to_cell(self.map_msg, pose[0], pose[1])
            if start_cell is None or not grid.is_free(start_cell):
                response.success = False
                response.message = (
                    'Current pose is outside the map or its cell is not known-free. '
                    'Planner remains stopped; check map and TF alignment.'
                )
                return response
            self.entrance_world = (pose[0], pose[1])
        else:
            # A map resize can temporarily remove the stored entrance. Never replace
            # it with the current pose: resume only when the original entrance is
            # represented by a known-free cell again.
            start_cell = world_to_cell(self.map_msg, *self.entrance_world)
            if start_cell is None or not grid.is_free(start_cell):
                response.success = False
                response.message = (
                    'Recorded entrance is not currently inside a known-free map cell; '
                    'planner remains stopped rather than redefining the entrance.'
                )
                return response

        self.planner = MazeExplorationPlanner(
            start_cell,
            information_gain_weight=float(
                self.get_parameter('information_gain_weight').value
            ),
        )
        self.started = True
        self.operator_started = True
        response.success = True
        response.message = f'Planning started from entrance grid cell {start_cell}; VLM guides frontier preference; no flight commands are sent.'
        self.get_logger().info(response.message)
        return response

    def _stop_cb(self, _request, response):
        self.started = False
        self.operator_started = False
        self._publish_empty_path()
        response.success = True
        response.message = 'Custom planning paused. This service does not command drone braking or landing.'
        self.get_logger().warning(response.message)
        return response

    def _publish_empty_path(self) -> None:
        msg = Path()
        msg.header.frame_id = self._frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(msg)
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'wingman_custom_planner'
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def _publish_decision(self, decision, grid: GridMap, pose) -> None:
        now = self.get_clock().now().to_msg()
        frame = self._frame()
        path_msg = Path()
        path_msg.header.frame_id = frame
        path_msg.header.stamp = now

        path_world = [cell_to_world(self.map_msg, cell) for cell in decision.path]
        previous_yaw = float(pose[3])
        for index, (x, y) in enumerate(path_world):
            if index + 1 < len(path_world):
                nx, ny = path_world[index + 1]
                previous_yaw = math.atan2(ny - y, nx - x)
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.header.stamp = now
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(pose[2])  # visualization only; not a flight target
            ps.pose.orientation.z = math.sin(previous_yaw / 2.0)
            ps.pose.orientation.w = math.cos(previous_yaw / 2.0)
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

        output = {
            'action': decision.action,
            'goal_cell': list(decision.goal) if decision.goal is not None else None,
            'path_cells': len(decision.path),
            'reason': decision.reason,
            'started': self.started,
            'control_mode': 'path_only_no_flight_commands',
            'vlm_action': self.vlm_action,
            'vlm_preferred_direction': self.preferred_direction,
            'person_detected': self.person_detected,
        }
        if decision.goal is not None:
            goal_x, goal_y = cell_to_world(self.map_msg, decision.goal)
            output['goal_world_xy'] = [round(goal_x, 3), round(goal_y, 3)]
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = frame
            goal_msg.header.stamp = now
            goal_msg.pose.position.x = goal_x
            goal_msg.pose.position.y = goal_y
            goal_msg.pose.position.z = float(pose[2])
            yaw = math.atan2(goal_y - pose[1], goal_x - pose[0])
            goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
            self.goal_pub.publish(goal_msg)

            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = now
            marker.ns = 'wingman_custom_planner'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = goal_msg.pose
            marker.scale.x = marker.scale.y = marker.scale.z = 0.45
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.2
            marker.color.a = 0.95
            self.marker_pub.publish(marker)
        else:
            self.goal_pub.publish(PoseStamped())
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = now
            marker.ns = 'wingman_custom_planner'
            marker.id = 0
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
        decision_msg = String()
        decision_msg.data = json.dumps(output, sort_keys=True)
        self.decision_pub.publish(decision_msg)
        self.get_logger().info(
            f"action={decision.action}; goal={decision.goal}; "
            f"path_cells={len(decision.path)}; reason={decision.reason}"
        )

    def _tick(self) -> None:
        if not self.started or self.planner is None or self.map_msg is None:
            return
        pose = self._lookup_pose()
        if pose is None:
            return

        try:
            grid = occupancy_to_grid(
                self.map_msg,
                int(self.get_parameter('free_threshold').value),
                float(self.get_parameter('obstacle_inflation_radius_m').value),
            )
        except ValueError as exc:
            self.get_logger().error(f'Invalid map: {exc}', throttle_duration_sec=3.0)
            return

        current_cell = world_to_cell(self.map_msg, pose[0], pose[1])
        if current_cell is None:
            self.get_logger().warning(
                'Current pose is outside occupancy grid; no route published.',
                throttle_duration_sec=3.0,
            )
            self._publish_empty_path()
            return

        decision = self.planner.decide(
            grid,
            current_cell,
            preferred_direction=self.preferred_direction,
            current_yaw=pose[3],
            force_return=(self.vlm_action == 'return_to_entrance'),
        )
        self._publish_decision(decision, grid, pose)
        if decision.action == 'no_path':
            self.get_logger().warning(
                f'Planner has no safe known-free path: {decision.reason}',
                throttle_duration_sec=3.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MazePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
