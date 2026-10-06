#!/usr/bin/env python3

import math
from typing import Optional, Tuple, List
import numpy as np
from scipy import ndimage

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Quaternion, Point, PointStamped
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.time import Time

import tf2_ros


def quat_to_yaw(q: Quaternion) -> float:
    """Extract planar yaw (rad) from a Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert planar yaw (rad) to Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def disk_offsets(r, res):
    k = int(np.ceil(r/res))
    a = np.arange(-k, k +1)
    dr, dc = np.meshgrid(a, a, indexing="ij")

    mask = dr**2 + dc**2 <= (r/res)**2
    dR = dr[mask]
    dC = dc[mask]

    return dR, dC

def points_free(pts, occ, unk, ox, oy, res, dr, dc): # True where the robot's disk fits at each point

    # world cordinates to cell cordinates
    col = np.floor((pts[:,0] - ox) / res).astype(int)
    row = np.floor((pts[:,1] - oy) / res).astype(int)
    h,w = occ.shape

    rows = row[:, None] + dr[None,:]
    cols = col[:, None] + dc[None,:]

    inb = (rows >= 0) & (rows < h) & (cols >= 0) & ( cols < w)
    hit = occ[np.clip(rows, 0, h-1), np.clip(cols, 0, w-1)] | ~inb

    free = ~hit.any(axis=1)
    free &= ~unk[np.clip(row, 0, h-1), np.clip(col, 0, w-1)]

    return free

def segment_free(start, end, spacing, occ, unk, ox, oy, res, dr, dc):
    sx,sy = start[0],start[1]
    ex,ey = end[0],end[1]
    l = np.hypot(ex-sx,ey-sy)
    n = int(np.ceil(l / spacing)) + 1

    pts = np.linspace(start, end, n)

    return points_free(pts=pts[1:], occ=occ, unk=unk, ox=ox, oy=oy, res=res, dr=dr, dc=dc).all()

class TreeNode():
    """
    Create the Nodes.
    It takes:
        x and y cordinates in the world view.
        parent is default None for root node. every other node will have a parent.
        Cost is later added based on the distance and the cost of the parent node.
        Each node gets it own list of children.
    """
    def __init__(self, x: float, y: float, parent = None, cost = 0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost
        self.children = []


class PathPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('path_planner_node')

        # Parameters
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('frontier_topic', 'frontiers')
        self.declare_parameter('path_topic', 'path')
        self.declare_parameter("tree_topic", "RRTtree")

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter("goal_pub_topic", "goal_pub_topic")
        self.declare_parameter("my_target_topic", "my_target_topic")
        self.declare_parameter("other_target_topic", "other_target_topic")

        self.declare_parameter("segment_size", 0.1)
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter("goal_radius", 0.3)
        self.declare_parameter("goal_bias", 0.1)
        self.declare_parameter("step_size", 0.4)
        self.declare_parameter("max_iter", 2000)
        self.declare_parameter("bot_radius", 0.113)
        self.declare_parameter("safty_margin", 0.1)
        self.declare_parameter("min_cluster_size", 3)
        self.declare_parameter("distance_gain", 1.0)
        self.declare_parameter("yaw_gain", 0.5)
        self.declare_parameter("cluster_gain", 0.02)
        self.declare_parameter("sticky_bonus", 0.5)
        self.declare_parameter("fail_timeout", 60.0)
        self.declare_parameter("fail_radius", 0.5)
        self.declare_parameter("w_other", 3.0)
        self.declare_parameter("r_other", 1.5)

        map_topic: str = self.get_parameter('map_topic').get_parameter_value().string_value
        frontier_topic: str = self.get_parameter('frontier_topic').get_parameter_value().string_value
        path_topic: str = self.get_parameter('path_topic').get_parameter_value().string_value
        tree_topic: str = self.get_parameter("tree_topic").get_parameter_value().string_value
        goal_pub_topic: str = self.get_parameter("goal_pub_topic").get_parameter_value().string_value
        my_target_topic: str = self.get_parameter("my_target_topic").get_parameter_value().string_value
        other_target_topic: str = self.get_parameter("other_target_topic").get_parameter_value().string_value

        self.goal_radius: float = self.get_parameter("goal_radius").get_parameter_value().double_value
        self.goal_bias: float = self.get_parameter("goal_bias").get_parameter_value().double_value
        self.step_size: float = self.get_parameter("step_size").get_parameter_value().double_value
        self.max_iter: int = self.get_parameter("max_iter").get_parameter_value().integer_value
        self.segment_size: float = self.get_parameter("segment_size").get_parameter_value().double_value
        self.bot_radius: float = self.get_parameter("bot_radius").get_parameter_value().double_value
        self.safty_margin: float = self.get_parameter("safty_margin").get_parameter_value().double_value
        self.min_cluster_size: int = self.get_parameter("min_cluster_size").get_parameter_value().integer_value
        self.distance_gain: float = self.get_parameter("distance_gain").get_parameter_value().double_value
        self.yaw_gain: float = self.get_parameter("yaw_gain").get_parameter_value().double_value
        self.cluster_gain: float = self.get_parameter("cluster_gain").get_parameter_value().double_value
        self.sticky_bonus: float = self.get_parameter("sticky_bonus").get_parameter_value().double_value
        self.fail_timeout: float = self.get_parameter("fail_timeout").get_parameter_value().double_value
        self.fail_radius: float = self.get_parameter("fail_radius").get_parameter_value().double_value
        self.w_other: float = self.get_parameter("w_other").get_parameter_value().double_value
        self.r_other: float = self.get_parameter("r_other").get_parameter_value().double_value

        self.base_frame: str = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout_sec').get_parameter_value().double_value)





        default_qos = QoSProfile(depth=10)


        # Publishers
        self.path_pub =  self.create_publisher(Path, path_topic, default_qos)
        self.tree_pub = self.create_publisher(Marker, tree_topic, default_qos)
        self.goal_pub = self.create_publisher(Marker, goal_pub_topic, default_qos)
        self.target_pub = self.create_publisher(PointStamped, my_target_topic, default_qos)

        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self._on_map, default_qos)
        self.frontier_sub = self.create_subscription(OccupancyGrid, frontier_topic, self._on_frontier, default_qos)
        self.target_sub = self.create_subscription(PointStamped, other_target_topic, self._other_target, default_qos)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.goal = None
        self.target = None
        self.publish_time = None
        self.map_frame = None
        self.blocked_frontier_counter = 0

        self.arrival_threshold = self.goal_radius * 1.2
        self.foot_print = self.goal_radius * 2
        self.sticky_radius = self.goal_radius * 1.5

        self.low_frontiers = False
        self.stopped = False
        self.failed = []

        self.timeout = Duration(seconds=30)
        self.rng = np.random.default_rng(0)
        self._latest_map: Optional[OccupancyGrid] = None
        self._latest_frontier: Optional[OccupancyGrid] = None
        self._latest_target: Optional[PointStamped] = None
        self.get_logger().info('PathPlannerNode initialized.')

    # ----------------- TF Helper -----------------

    def get_robot_pose(self, target_frame, source_frame: Optional[str] = None
                       ) -> Optional[Tuple[float, float, float]]:
        """
        Lookup TF to get robot pose (x, y, yaw) in target_frame.
        Returns None if not available within timeout.
        """
        tgt = target_frame
        src = source_frame or self.base_frame

        try:
            transform = self.tf_buffer.lookup_transform(
                tgt, src, Time(), timeout=self.tf_timeout
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup {tgt} <- {src} failed: {e}')
            return None

        t = transform.transform.translation
        r = transform.transform.rotation
        yaw = quat_to_yaw(r)

        return (t.x, t.y, yaw)

    # ----------------- Callbacks -----------------

    def _other_target(self, msg: PointStamped) -> None:
        """Store the latest target point from the other robot"""
        self._latest_target = msg

    def _on_frontier(self, msg: OccupancyGrid) -> None:
        """Store the latest frontier map (could be used by your planner)."""
        self._latest_frontier = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Trigger planning when a new map arrives."""
        self.map_frame = msg.header.frame_id
        self._latest_map = msg # keep latest map
        if self._latest_frontier is None: return

        pose = self.get_robot_pose(target_frame=self.map_frame)
        if pose is None:
            return

        x, y, yaw = pose

        if self.goal is not None: # Three states for a goal to end.
            d = self.distance(self.goal,x,y)
            if d < self.arrival_threshold: # Distance of current position to goal position is shorter than a arrival_threshold.
                self.get_logger().info(f"arrived")
                self.goal = None
            elif not self.frontier(self._latest_frontier, self.target): # Checks if the target does no longer hold any active frontiers.
                self.get_logger().info(f"Target stale")
                self.goal = None
            elif self.get_clock().now() - self.publish_time > self.timeout: # If the bot gets stuck for to long.
                self.failed.append((self.target[0], self.target[1], self.get_clock().now()))
                self.get_logger().info(f"marking {self.target[0],self.target[1]} as failed")
                self.get_logger().info(f"timeout")
                self.goal = None
            else:
                return


        path = self.plan_path((x, y, yaw),
                              msg,
                              self._latest_frontier)

        if path is None or not path.poses:
            if  self.low_frontiers and not self.stopped: # Logic for sending a ZERO path to the path_follower_node so there is a termination.
                empty_path = Path()
                empty_path.header.frame_id = msg.header.frame_id
                empty_path.header.stamp = msg.header.stamp

                self.path_pub.publish(empty_path)
                self.goal = None
                self.stopped = True
                self.get_logger().info(f"Exploration complete")
            elif not self.low_frontiers:
                self.stopped = False
            return
        else:
            self.publish_target(self.target)
            self.path_pub.publish(path)
            self.goal = (self.target[0], self.target[1])
            self.publish_time = self.get_clock().now()



    # ----------------- help functions -----------------
    def to_my_frame(self, msg):
        if self.map_frame is None: return None
        try:
            return self.tf_buffer.transform(msg, self.map_frame, timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f"cannot transform {msg.header.frame_id} -> {self.map_frame}: {e}")
            return None

    def distance(self,goal,x,y): # Calculats the distance between the goal and current position.
            goal_x, goal_y = goal[0], goal[1]
            x = goal_x - x
            y = goal_y - y
            return math.sqrt(x*x+y*y)

    def cell_to_world(self,origin, rowORcol, resolution): # Converts from Cell cordinates to world cordinates.
        return origin +(rowORcol + 0.5) * resolution


    def world_to_cell(self, origin, cordinate, resolution): # Converts from world cordinates to cell cordinates.
        return int(np.floor((cordinate - origin) / resolution))


    def frontier_clusters(self, start, frontier_msg, occ, unk, ox, oy, res, dr, dc): # Computes valid clusters.

        if frontier_msg is None:
            return []

        frontier_grid = np.array(frontier_msg.data, dtype=np.int8).reshape(frontier_msg.info.height, frontier_msg.info.width)
        labels, n = ndimage.label(frontier_grid == 100, structure=np.ones((3,3)))
        sizes = np.bincount(labels.ravel())
        clusters = []

        for i in range(1,n+1):
            if sizes[i] < self.min_cluster_size: continue
            r, c = np.where(labels==i)
            fx = self.cell_to_world(frontier_msg.info.origin.position.x, c, frontier_msg.info.resolution)
            fy = self.cell_to_world(frontier_msg.info.origin.position.y, r, frontier_msg.info.resolution)
            valid = points_free(np.c_[fx, fy], occ, unk, ox, oy, res, dr, dc)
            d = np.hypot((fx - start[0]),(fy - start[1]))
            ok = valid & (d >= self.foot_print)
            if not ok.any(): continue
            k = np.argmin(np.where(ok, d, np.inf))
            target = (fx[k], fy[k])
            clusters.append((target[0], target[1], sizes[i]))

        return clusters

    def heuristic_func(self, start, clusters):
        if not clusters:
            self.low_frontiers = True
            return None
        self.low_frontiers = False
        arr = np.array(clusters)
        fx, fy, gain = arr[:,0], arr[:,1], arr[:,2]

        d = np.hypot(fx - start[0], fy - start[1])
        ang = np.arctan2(fy - start[1], fx - start[0]) - start[2]
        turn = abs(np.arctan2(np.sin(ang), np.cos(ang)))

        cost = self.distance_gain * d + self.yaw_gain * turn - self.cluster_gain * gain # Target gets picked by distance, turning needed and cluster size.
        if self._latest_target is not None:
            p = self.to_my_frame(self._latest_target)
            if p is not None:
                d_o = np.hypot(fx - p.point.x, fy - p.point.y)
                cost += self.w_other * np.maximum(0, 1- d_o / self.r_other)

        now = self.get_clock().now()
        self.failed = [f for f in self.failed if now - f[2] < Duration(seconds=self.fail_timeout)]
        if self.failed:
            F = np.array([(x,y) for x, y, _ in self.failed])            # (M,2)
            dist = np.hypot(fx[:,None] - F[:,0], fy[:,None] - F[:,1])   # (N,M)
            cost[(dist < self.fail_radius).any(axis=1)] = np.inf        # if cluster near failur set it as invalid right now


        if self.target is not None: # Checks if the current target is allready with in a lenght of the new possible frontier.
            near = np.hypot(fx - self.target[0], fy - self.target[1]) < self.sticky_radius
            cost[near] -= self.sticky_bonus # Gives the current target some bias then.

        min_cost = np.argmin(cost)
        if np.isinf(cost[min_cost]): return None

        return (fx[min_cost], fy[min_cost])


    def frontier(self, frontier_msg: Optional[OccupancyGrid], target): # Checks if frontiers is still valid.
        if frontier_msg is None: return True

        grid = np.array(frontier_msg.data, dtype=np.int8).reshape(frontier_msg.info.height, frontier_msg.info.width)
        h, w = grid.shape
        res = frontier_msg.info.resolution
        col = self.world_to_cell(frontier_msg.info.origin.position.x, target[0], res)
        row = self.world_to_cell(frontier_msg.info.origin.position.y, target[1], res)
        if row < 0 or col < 0 or row >= h or col >= w: return False

        radius = int(np.ceil(self.goal_radius / res))
        window = grid[max(row - radius,0): min(row + radius + 1, h)
                      ,max(col - radius, 0): min(col + radius + 1, w)]
        return np.any(window==100) # Check if any cell around the target is a valid frontier.

    def publish_target(self,pt):
        # publish target msg for other bot
        explo = PointStamped()
        explo.header.frame_id = self.map_frame
        explo.header.stamp = self.get_clock().now().to_msg()
        explo.point.x = pt[0]
        explo.point.y = pt[1]
        explo.point.z = 0.0
        self.target_pub.publish(explo)

    def publish_marker(self, pt):
        # publish target for rviz
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'goal_frontier'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(pt[0])
        marker.pose.position.y = float(pt[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.goal_pub.publish(marker)


    # ----------------- Planning -----------------
    def plan_path(self,
                  start: Tuple[float, float, float],
                  map_msg: OccupancyGrid,
                  frontier_msg: Optional[OccupancyGrid]):

        mapResolution = map_msg.info.resolution
        mapOrigin = map_msg.info.origin
        grid = np.array(map_msg.data, dtype=np.int8).reshape(map_msg.info.height, map_msg.info.width)

        h, w = grid.shape

        unk = grid < 0
        occ = grid > 50

        x_min = mapOrigin.position.x
        y_min = mapOrigin.position.y
        x_max = x_min + w * mapResolution
        y_max = y_min + h * mapResolution
        c = self.bot_radius + self.safty_margin
        disk_radius = c + (.71*mapResolution)
        spacing = 2*np.sqrt(disk_radius**2 - c**2)

        dr, dc = disk_offsets(disk_radius, mapResolution)

        clusters = self.frontier_clusters(start, frontier_msg, occ, unk, x_min, y_min, mapResolution, dr, dc) # Get valid clusters

        self.target = self.heuristic_func(start, clusters)
        if self.target is None: return None

        self.publish_marker(self.target)

        root = TreeNode(start[0], start[1])
        tree = [root]
        goal_nodes = []


        if segment_free(start=start[:2], end=self.target, spacing=spacing, occ=occ, unk=unk, ox=x_min, oy=y_min, res=mapResolution, dr=dr, dc=dc):

            waypoint = [(start[0], start[1]),(self.target[0], self.target[1])]
            self.get_logger().info(f"Straight path, skip RRT")

        else:

            #RRT
            for iteration in range(self.max_iter):

                if self.rng.random() < self.goal_bias: # Move goal_bias% of the time towards the target.
                    q = self.target
                else:
                    q = (self.rng.uniform(x_min, x_max), self.rng.uniform(y_min, y_max))

                nearest = None
                best_d = np.inf

                for node in tree: # loop through tree to see which node is closest to point q.
                    d = self.distance(q, node.x, node.y)
                    if d < best_d:
                        best_d = d
                        nearest = node
                if best_d < 1e-9: continue # Dived by Zero guard

                dx = q[0] - nearest.x
                dy = q[1] - nearest.y

                if best_d > self.step_size: # limit new point to be no longer then step_size
                    q_new = (nearest.x + self.step_size * dx / best_d,
                            nearest.y + self.step_size * dy / best_d)
                else:
                    q_new = (q[0], q[1])


                if not segment_free(start=(nearest.x, nearest.y), end=q_new,
                                    spacing=spacing, occ=occ, unk=unk,
                                    ox=x_min, oy=y_min, res=mapResolution,
                                    dr=dr, dc=dc): continue

                edge = self.distance((nearest.x, nearest.y), q_new[0], q_new[1])
                new = TreeNode(q_new[0],q_new[1],nearest, nearest.cost + edge)
                nearest.children.append(new)
                tree.append(new)
                if self.distance(self.target, new.x, new.y) < self.goal_radius:
                    goal_nodes.append(new)


            self.get_logger().info(f"tree size: {len(tree)}, goal nodes: {len(goal_nodes)}")

            # Publish tree structure for visulization in Rviz
            tree_msg = Marker()
            tree_msg.header.frame_id = map_msg.header.frame_id
            tree_msg.header.stamp = map_msg.header.stamp
            tree_msg.ns = "rrt"
            tree_msg.id = 0
            tree_msg.type = Marker.LINE_LIST
            tree_msg.action = Marker.ADD
            tree_msg.pose.orientation.w = 1.0
            tree_msg.scale.x = 0.01
            tree_msg.color.r = 0.2
            tree_msg.color.g = 0.6
            tree_msg.color.b = 1.0
            tree_msg.color.a = 1.0

            for node in tree:
                if node.parent is None: continue
                tree_msg.points.append(Point(x=node.x, y=node.y ,z=0.0))
                tree_msg.points.append(Point(x=node.parent.x, y=node.parent.y, z=0.0))

            self.tree_pub.publish(tree_msg)

            if not goal_nodes:
                self.failed.append((self.target[0], self.target[1], self.get_clock().now()))
                self.get_logger().info(f"marking {self.target[0],self.target[1]} as failed")
                return None

            best_cost = np.inf
            best_node = None
            for index in range(len(goal_nodes)):
                if goal_nodes[index].cost < best_cost:
                    best_node = goal_nodes[index]
                    best_cost = goal_nodes[index].cost

            self.get_logger().info(f"best cost: {best_cost:.2f}")

            waypoint = []
            node = best_node

            while node is not None: # Loop from goal node to root node
                waypoint.append((node.x, node.y))
                node = node.parent

            waypoint.reverse() # reverse list to get from root to goal
        self.get_logger().info(f"{len(waypoint)}, first:{waypoint[0]}, last:{waypoint[-1]}")


        shorter_waypoints = [waypoint[0]] # init list with root node
        i = 0
        while i < len(waypoint) - 1:
            j = len(waypoint) - 1
            while j > i + 1 and not segment_free(start=waypoint[i], end=waypoint[j],
                                                 spacing=spacing, occ=occ, unk=unk,
                                                 ox=x_min, oy=y_min,
                                                 res=mapResolution, dr=dr, dc=dc):
                j = j - 1
            shorter_waypoints.append(waypoint[j])
            i = j

        waypoint = shorter_waypoints

        segment_path = [waypoint[0]]
        for i in range(len(waypoint)-1):
            segment_length = self.distance(waypoint[i+1],waypoint[i][0],waypoint[i][1])
            n = max(int(np.ceil(segment_length/self.segment_size)),3)
            segment = list(np.linspace(waypoint[i],waypoint[i+1],n))
            segment_path.extend(segment[1:])
        waypoint = segment_path


        new_msg = Path()
        new_msg.header.frame_id = map_msg.header.frame_id


        for i in range(len(waypoint)):
            plannedPath = PoseStamped()
            plannedPath.header.frame_id = map_msg.header.frame_id
            plannedPath.header.stamp = self.get_clock().now().to_msg()
            plannedPath.pose.position.x = waypoint[i][0]
            plannedPath.pose.position.y = waypoint[i][1]

            if i < len(waypoint) - 1:
                yaw = np.arctan2((waypoint[i + 1][1] - waypoint[i][1]), (waypoint[i + 1][0] - waypoint[i][0]))

            plannedPath.pose.orientation = yaw_to_quaternion(yaw)

            new_msg.poses.append(plannedPath)

        return new_msg





def main() -> None:
    rclpy.init()
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
