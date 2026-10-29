#!/usr/bin/env python3
"""Frontier exploration on the RTAB-Map occupancy grid, using Nav2 goals.
1) /wingman/takeoff, wait for hover.  2) /wingman/start_explore (records the entrance pose).
3) When no frontiers remain it returns to the entrance and calls /wingman/land.
RViz topic: /wingman/frontiers (cyan = candidates, green = current goal, red = entrance)."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
import tf2_ros
from wingman_exploration.frontier_utils import find_frontier_goals
 
 
class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        # Bounds are in the MAP frame (origin = spawn pose). From maze_exploration.sdf: maze interior is
        # world X,Y in [-16,16]; spawn (0,-17.2) facing +Y  =>  map x in [1.2,33.2], y in [-16,16].
        self.declare_parameter('bounds', [0.9, 33.5, -16.5, 16.5])
        self.declare_parameter('min_cluster_cells', 10)
        self.declare_parameter('clearance', 0.45)
        self.declare_parameter('min_goal_dist', 0.8)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('empty_checks_to_finish', 5)
        self.declare_parameter('blacklist_radius', 0.7)
        self.declare_parameter('autostart', False)
        self.declare_parameter('land_on_return', True)
 
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, '/rtabmap/map', self.on_map, qos)
        self.mark_pub = self.create_publisher(MarkerArray, '/wingman/frontiers', 5)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.land_cli = self.create_client(Trigger, '/wingman/land')
        self.create_service(Trigger, '/wingman/start_explore', self.srv_start)
 
        self.map = None
        self.state = 'starting' if self.p('autostart') else 'waiting'  # waiting|starting|exploring|returning|done
        self.entrance = None
        self.goal_active = False
        self.goal_xy = None
        self.goal_t = 0.0
        self.goal_handle = None
        self.blacklist = []
        self.empty = 0
        self.return_tries = 0
        self.cands = []
        self.create_timer(1.0, self.tick)
        self.get_logger().info('frontier_explorer ready. Call /wingman/start_explore after takeoff.')
 
    def p(self, name):
        return self.get_parameter(name).value
 
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9
 
    def on_map(self, m):
        self.map = m
 
    def robot_pose(self):
        try:
            t = self.tfbuf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw
 
    def srv_start(self, req, res):
        pose = self.robot_pose()
        if pose is None:
            res.success = False; res.message = 'no map->base_link TF yet'; return res
        self.entrance = pose
        self.state = 'exploring'; self.empty = 0
        self.get_logger().info(f'Entrance recorded at x={pose[0]:.2f} y={pose[1]:.2f}. Exploring.')
        res.success = True; res.message = 'exploring'; return res
 
    def send_goal(self, x, y, yaw):
        if not self.nav.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('Nav2 navigate_to_pose not available yet', throttle_duration_sec=5.0)
            return
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x); g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw / 2); g.pose.pose.orientation.w = math.cos(yaw / 2)
        self.goal_active = True; self.goal_xy = (x, y); self.goal_t = self.now_s()
        self.nav.send_goal_async(g).add_done_callback(self.on_goal_response)
        self.get_logger().info(f'[{self.state}] goal -> x={x:.2f} y={y:.2f}')
 
    def on_goal_response(self, fut):
        h = fut.result()
        if not h.accepted:
            self.get_logger().warn('goal rejected')
            self.fail_goal(); return
        self.goal_handle = h
        h.get_result_async().add_done_callback(self.on_result)
 
    def on_result(self, fut):
        status = fut.result().status
        self.goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_active = False
            if self.state == 'returning':
                self.state = 'done'
                self.get_logger().info('Back at entrance. Mission complete.')
                if self.p('land_on_return') and self.land_cli.service_is_ready():
                    self.land_cli.call_async(Trigger.Request())
        elif self.goal_active:
            self.get_logger().warn(f'goal ended with status {status}')
            self.fail_goal()
 
    def fail_goal(self):
        if self.state == 'exploring' and self.goal_xy:
            self.blacklist.append(self.goal_xy)
        elif self.state == 'returning':
            self.return_tries += 1
        self.goal_active = False
 
    def tick(self):
        if self.state == 'starting':
            self.srv_start(None, Trigger.Response())
        if self.map is None or self.state in ('waiting', 'done', 'starting'):
            return
        pose = self.robot_pose()
        if pose is None:
            return
        if self.goal_active:
            if self.now_s() - self.goal_t > float(self.p('goal_timeout')):
                self.get_logger().warn('goal timeout - cancelling')
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.fail_goal()
            return
 
        if self.state == 'returning':
            if self.return_tries >= 5:
                self.get_logger().error('Cannot reach entrance after 5 tries - landing here')
                self.state = 'done'
                if self.land_cli.service_is_ready():
                    self.land_cli.call_async(Trigger.Request())
                return
            self.send_goal(self.entrance[0], self.entrance[1], pose[2]); return
 
        m = self.map
        grid = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        self.cands = find_frontier_goals(
            grid, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y,
            (pose[0], pose[1]), tuple(self.p('bounds')), self.blacklist,
            min_cluster=int(self.p('min_cluster_cells')), clearance=float(self.p('clearance')),
            min_goal_dist=float(self.p('min_goal_dist')), blacklist_radius=float(self.p('blacklist_radius')))
        self.publish_markers()
        if not self.cands:
            self.empty += 1
            self.get_logger().info(f'no valid frontier ({self.empty}/{self.p("empty_checks_to_finish")})')
            if self.empty >= int(self.p('empty_checks_to_finish')):
                self.get_logger().info('Exploration finished - returning to entrance')
                self.state = 'returning'; self.return_tries = 0
            return
        self.empty = 0
        _, x, y, _, _ = self.cands[0]
        self.send_goal(x, y, math.atan2(y - pose[1], x - pose[0]))
 
    def publish_markers(self):
        arr = MarkerArray()
 
        def mk(i, color, size, pts):
            m = Marker(); m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'wingman'; m.id = i; m.type = Marker.SPHERE_LIST; m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = size
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.points = [Point(x=float(a), y=float(b), z=0.3) for a, b in pts]
            return m
        arr.markers.append(mk(0, (0.0, 0.8, 1.0, 1.0), 0.25, [(c[1], c[2]) for c in self.cands]))
        arr.markers.append(mk(1, (0.0, 1.0, 0.0, 1.0), 0.45,
                              [self.goal_xy] if (self.goal_active and self.goal_xy) else []))
        arr.markers.append(mk(2, (1.0, 0.0, 0.0, 1.0), 0.45, [self.entrance[:2]] if self.entrance else []))
        self.mark_pub.publish(arr)
 
 
def main():
    rclpy.init()
    rclpy.spin(FrontierExplorer())
 
 
if __name__ == '__main__':
    main()
