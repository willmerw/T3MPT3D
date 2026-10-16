#!/usr/bin/env python3

import math
from typing import Optional, Tuple, List
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from scipy.ndimage import distance_transform_edt
from geometry_msgs.msg import PoseStamped, Quaternion,Point,PolygonStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import Bool
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import numpy as np

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



class TreeNode:
    def __init__(self, pt, parent):
        self.pt = np.array(pt)
        self.children = []
        self.parent = parent

    def get_closest_node(self, pt):
        pt = np.array(pt)

        closest_dist = np.linalg.norm(pt-self.pt)
        closest_node = self

        for node in self.children:
            child_dist, child_node = node.get_closest_node(pt)
            if child_dist < closest_dist:
                closest_dist = child_dist
                closest_node = child_node

        return closest_dist, closest_node

    def get_path_to_root(self):
        path = [self.pt]

        if self.parent == None:
            return path
        path = self.parent.get_path_to_root() + path
        return path

class PathPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('path_planner_node')

        default_qos = QoSProfile(depth=10)
        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        plank_qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                                       reliability=ReliabilityPolicy.RELIABLE)

        self.received_start = False
        self.start = [0.0,0.0,0.0] #x,y, thp
        self.r = 0.5 # Plank radius
        self.goal = [0.0,0.0] #x,y
        self.pose_path = []

        self.node_max_dist = 0.2
        self.obs_fid = 0.01 #obstacle fidelity
        self.plank_twist = np.pi/10
        self.goal_radius = 0.2
        self.map_inflation = 3

        x = self.start[0]
        y = self.start[1]
        self.tree = TreeNode([x,y], None)
        self.tree_pts = []

        self.path_pub = self.create_publisher(Path, '/path', transient_qos)

        self.tree_pub = self.create_publisher(Marker, 'rrt_tree', 10)
        self.inf_map_pub = self.create_publisher(OccupancyGrid, 'inf_map', 10)
        self.goal_pub_vis = self.create_publisher(Marker, '/exit',transient_qos)
        self.plank_path_pub = self.create_publisher(MarkerArray, '/plank_path_vis', transient_qos) #Planks on the path

        #HARRYS FLAGGOR
        self.rivstart = False
        self.start_sub = self.create_subscription(Bool, '/rivstart',self._on_start, plank_qos)


        # Subscribers
        self.map_received = False
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._on_map, default_qos)

        self.plank_received = False
        self.plank_sub = self.create_subscription(PolygonStamped, '/plank', self._on_plank, plank_qos)

        self.goal_received = False
        self.goal_sub = self.create_subscription(PoseStamped, '/goal', self._on_goal, default_qos)

        self.declare_parameter('tb1_odom_world', '/turtlebot1/odom_world')
        self.declare_parameter('tb2_odom_world', '/turtlebot2/odom_world')

        tb1_odom_world: str = self.get_parameter('tb1_odom_world').get_parameter_value().string_value
        tb2_odom_world: str = self.get_parameter('tb2_odom_world').get_parameter_value().string_value

        self.odom1_sub = message_filters.Subscriber(
            self, Odometry, tb1_odom_world,10
        )
        self.odom2_sub = message_filters.Subscriber(
            self, Odometry, tb2_odom_world,10
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.odom1_sub, self.odom2_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self.odom_callback)

        # State
        self._latest_map: Optional[OccupancyGrid] = None

    def odom_callback(self,odom1_msg,odom2_msg):

        if not self.rivstart:
            return

        if self.received_start:
            return

        p1x = odom1_msg.pose.pose.position.x
        p1y = odom1_msg.pose.pose.position.y

        p2x = odom2_msg.pose.pose.position.x
        p2y = odom2_msg.pose.pose.position.y

        pt1 = np.array([p1x,p1y])
        pt2 = np.array([p2x,p2y])
        d = pt2-pt1
        self.r = np.linalg.norm(d)/2 + 0.2
        thp = np.arctan2(d[1],d[0])
        pt = pt1 + d/2
        x = pt[0]
        y = pt[1]
        self.start = [x,y,thp]
        self.get_logger().info(f"START: {self.start}")
        self.received_start = True


    def _on_start(self,start_msg):
        self.rivstart = start_msg.data
        self.get_logger().info("RAISING SAILS")

    def _on_map(self,map_msg):
        if not self.rivstart:
            return
        if not self.plank_received:
            return
        if not self.goal_received:
            return
        if not self.received_start:
            return
        if not self.map_received:
            fail = True
            while fail:
                self.map_received = True
                self.map = map_msg
                self.publish_goal(self.goal)
                fail = self.plan_path(map_msg)
                if fail == True:
                    continue
                else:
                    fail = False



    def _on_plank(self,plank_msg):
        print(plank_msg)
        if not self.plank_received:
            self.plank_received = True
            self.get_logger().info("THE SHIP IS READY")


    def _on_goal(self,goal_msg):
        if not self.goal_received:
            self.goal_received = True
            x = goal_msg.pose.position.x
            y = goal_msg.pose.position.y
            self.goal = [x,y]
            self.get_logger().info("THE GOAL IS SET")


    def plan_path(self,map_msg: OccupancyGrid):

        self.get_logger().info("READ THE MAP YE DOGS!")

        inf_map_msg = self.inflate_map(map_msg)
        self.inf_map_pub.publish(inf_map_msg)
        print(map_msg.header.frame_id)

        x,y,thp = self.start

        self.get_logger().info(f"THE GOAL IS {self.goal}")
        self.get_logger().info(f"THE START IS {self.start}")

        map_res = map_msg.info.resolution
        map_height = map_msg.info.height * map_res
        map_width = map_msg.info.width * map_res
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        print(map_width,map_height,origin_x,origin_y)
        self.tree = TreeNode([x,y], None)

        self.tree_pts = []
        self.leaves = []

        # RRT
        valid_pose_paths = []
        valid_paths = []
        while len(valid_paths) == 0:
            num_nodes = 5000
            for it in range(num_nodes):

                r_x = np.random.uniform(origin_x, origin_x+map_width)
                r_y = np.random.uniform(origin_y, origin_y+map_height)
                random_pt = np.array([r_x, r_y], dtype=float)

                _, closest_node = self.tree.get_closest_node(random_pt)
                d = np.array(random_pt - closest_node.pt)
                d_norm = np.linalg.norm(d)
                border_pt = closest_node.pt + (d/d_norm)*self.node_max_dist
                obs = self.check_obs(closest_node.pt,border_pt,map_msg)
                if obs:
                    continue

                if d_norm < self.node_max_dist:
                    new_node = TreeNode(pt=random_pt,parent=closest_node)

                else:
                    new_pt = border_pt
                    new_node = TreeNode(pt=new_pt,parent=closest_node)

                self.tree_pts.append((closest_node.pt,new_node.pt))

                closest_node.children.append(new_node)

                if closest_node in self.leaves:
                    self.leaves.remove(closest_node)

                if np.linalg.norm(np.array(self.goal)-np.array(new_node.pt)) < self.goal_radius:
                    self.leaves.append(new_node)

            self.publish_tree(self.tree_pts)


            r = self.r
            self.get_logger().info(f"THE TREASSURE CAN BE REACHED VIA {len(self.leaves)} SALTY SEAS")
            for leaf in self.leaves:
                path = leaf.get_path_to_root()
                path = self.maximize_clearance(path, map_msg)
                path = self.cut_trim(path)
                poses = []
                """
                for i in range(1,len(path)):
                    prev_pt = np.array(path[i-1])
                    pt = np.array(path[i])
                    d = pt-prev_pt
                    th = np.arctan2(d[1],d[0])
                    poses.append([pt[0],pt[1],th])
                valid_paths.append(path)
                valid_pose_paths.append(poses)
                """
                for pt in path:
                    poses.append([pt[0],pt[1],thp])

                valid_path = True
                i = 0
                nthp =thp
                for pt in path:

                    v_yaw, nthp = self.valid_yaw_exists(pt,nthp,r)

                    if v_yaw:
                        poses = np.array(poses)
                        poses[i:,2] = nthp
                    else:
                        valid_path = False
                        break
                    i+=1

                if valid_path:
                    valid_pose_paths.append(poses)
                    valid_paths.append(path)
                if len(valid_paths) < 1:
                    self.get_logger().info("YER HOLDIN THE MAP UPSIDE DOWN YE SCURVY DOG!")

                self.get_logger().info(f"NODES IN TREE: {len(self.tree_pts)}")
        print(f"DONE, Found {len(valid_paths)} valid paths")



        best_path, best_pose_path = self.evaluate_valid_paths(valid_paths, valid_pose_paths)

        self.pose_path = best_pose_path
        path = best_path
        self.publish_plank_path()
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_pt_ls = []
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_pt_ls = []
        for pt in best_pose_path:
            path_pt = PoseStamped()
            path_pt.header.frame_id = 'world'
            path_pt.pose.position.x = pt[0]
            path_pt.pose.position.y = pt[1]

            yaw = pt[2]
            path_pt.pose.orientation.z = math.sin(yaw / 2.0)
            path_pt.pose.orientation.w = math.cos(yaw / 2.0)

            path_pt_ls.append(path_pt)
        path_msg.poses = path_pt_ls
        self.path_pub.publish(path_msg)
        self.get_logger().info("OOH I CAN SMELL THE RUM OOOZIN")

        return path


    def cut_trim(self,path, dist=0.4):
        path = np.array(path)
        if len(path) < 2:
            return path

        # 1. Calculate cumulative distances along the path
        diffs = np.diff(path, axis=0)
        segment_lengths = np.linalg.norm(diffs, axis=1)
        cumulative_dist = np.insert(np.cumsum(segment_lengths), 0, 0)
        total_length = cumulative_dist[-1]

        # 2. Generate target distances at uniform intervals
        target_dists = np.arange(0, total_length, dist)
        if target_dists[-1] != total_length:
            target_dists = np.append(target_dists, total_length)

        # 3. Interpolate new points
        new_path = np.zeros((len(target_dists), path.shape[1]))
        for i in range(path.shape[1]):
            new_path[:, i] = np.interp(target_dists, cumulative_dist, path[:, i])

        return new_path



    def evaluate_valid_paths(self,paths,pose_paths):
        scores = []
        d_w = 1.0
        yaw_w = 1.0
        for path,pose_path in zip(paths,pose_paths):
            score = 0
            for i in range(1,len(pose_path)):
                yaw = pose_path[i][2]
                prev_yaw = pose_path[i-1][2]
                pt = np.array(path[i])
                prev_pt = np.array(path[i-1])
                d = np.linalg.norm(pt-prev_pt)
                d_yaw = np.abs(yaw-prev_yaw)

                score = score + d_w*d + yaw_w*d_yaw
            scores.append(score)

        best_i = np.argmin(scores)
        return paths[best_i], pose_paths[best_i]


    def publish_plank_path(self):
        planks = MarkerArray()
        #planks.header.frame_id = 'world'
        for i,(x,y,thp) in enumerate(self.pose_path):
            plank = self.plank_marker(x,y,thp,i)
            plank.ns = "path_planks"
            planks.markers.append(plank)

        self.plank_path_pub.publish(planks)

    def plank_marker(self,x,y,thp,id):
            marker = Marker()

            # Frame and timestamp configuration
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()

            # Namespace and ID to identify the marker
            marker.ns = "lines"
            marker.id = id

            # Type: LINE_STRIP connects points sequentially (0-1, 1-2, 2-3...)
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Scale: scale.x controls the line thickness (in meters)
            marker.scale.x = 0.05

            # Color and Alpha (opacity) - Alpha must be > 0 to be visible
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            shift = np.array([np.cos(thp),np.sin(thp)]) * self.r
            p1 = np.array([x,y]) + shift
            p2 = np.array([x,y]) - shift
            pp1 = Point(x=p1[0], y=p1[1], z=0.0)
            pp2 = Point(x=p2[0], y=p2[1], z=0.0)

            marker.points = [pp1, pp2]

            return marker

    def valid_yaw_exists(self,pt,thp,r):
        inc = 0
        while inc < np.pi/2:
            nthp = thp+inc
            shift = np.array([np.cos(nthp)*r,np.sin(nthp)*r])
            pt1 = pt + shift
            pt2 = pt - shift
            obs = self.check_obs(pt1,pt2,self.map)

            if not obs:
                return True, nthp

            nthp = thp-inc
            shift = np.array([np.cos(nthp)*r,np.sin(nthp)*r])
            pt1 = pt + shift
            pt2 = pt - shift
            obs = self.check_obs(pt1,pt2,self.map)

            if not obs:
                return True, nthp
            inc += self.plank_twist


        return False, None

    def check_obs(self, start, goal, map_msg):
        map_res = map_msg.info.resolution
        map_values = map_msg.data
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        d = goal - start
        d_norm = np.linalg.norm(d)

        increment = (d/d_norm)*self.obs_fid
        check_pt = start + increment
        obs = False
        while np.linalg.norm(check_pt - start) < d_norm:
            cell_x = int((check_pt[0]-origin_x)/map_res)
            cell_y = int((check_pt[1]-origin_y)/map_res)
            grid_index = int(cell_y * map_msg.info.width + cell_x)
            if map_values[grid_index] >= 0.5 or map_values[grid_index] == -1:
                obs = True
                break
            check_pt += increment
        return obs

    def publish_tree(self, tree_edges):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rrt_tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.02
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for parent, child in tree_edges:
            p_start = Point(x=float(parent[0]), y=float(parent[1]), z=0.0)
            p_end = Point(x=float(child[0]), y=float(child[1]), z=0.0)
            marker.points.append(p_start)
            marker.points.append(p_end)

        self.tree_pub.publish(marker)

    def publish_goal(self, pt):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'exit'
        marker.id = 40
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

        self.goal_pub_vis.publish(marker)

    def optimize_path(self, path, map_msg):
        i = 0
        while (i+2) < len(path)-1:
            child = path[i]
            gp = path[i+2]
            obs = self.check_obs(child,gp,map_msg)

            if not obs:
                path.pop(i+1)
            else:
                i+=1

        i = 0
        while (i+2) < len(path)-1:
            child = path[i]
            middle = path[i+1]
            gp = path[i+2]

            step_size = 0.1

            c_m = np.array(middle-child)
            d1 = np.linalg.norm(c_m)
            c_md = np.linalg.norm(c_m)
            c_m = (c_m/c_md)*step_size


            gp_m = np.array(middle-gp)
            d2 = np.linalg.norm(gp_m)
            gp_md = np.linalg.norm(gp_m)
            gp_m = (gp_m/gp_md)*step_size

            child_n = child.copy()
            gp_n = gp.copy()


            while (np.linalg.norm(child_n-child) < d1 and np.linalg.norm(gp_n-gp) < d2):
                child_n += c_m
                gp_n += gp_m
                obs = self.check_obs(child_n,gp_n,map_msg)
                if not obs:
                    path.pop(i+1)
                    path[i+1:i+1] = [child_n, gp_n]

                    break
            i = i+2
        segment_path = []
        for i in range(len(path)-1):
            segment_length = np.linalg.norm(path[i]-path[i+1])
            segment = list(np.linspace(path[i],path[i+1],int(segment_length/0.1)))
            segment_path = segment_path + segment
        path = segment_path

        return path

    def inflate_map(self,map_msg):
        map_grid = np.array(map_msg.data, dtype=np.int16).reshape(
        (map_msg.info.height, map_msg.info.width))


        rows, cols = np.where(map_grid >= 50)

        for row,col in zip(rows,cols):
            infl = self.map_inflation
            for i in range(-infl,infl):
                for j in range(-infl,infl):
                    try:
                        map_grid[row+i][col+j] = 100
                    except Exception as e:
                        continue

        map_msg.data = map_grid.ravel().tolist()
        return map_msg

    def maximize_clearance(self, path, map_msg, max_iterations: int = 15, step_size: float = 0.05):

        map_grid = np.array(map_msg.data, dtype=np.int16).reshape(
            (map_msg.info.height, map_msg.info.width)
        )

        # Create obstacle mask (obstacles >= 50 or unknown == -1)
        obstacle_mask = (map_grid >= 50) | (map_grid == -1)

        # Compute distance transform in pixels
        dist_map_px = distance_transform_edt(~obstacle_mask)

        # Scale distance map to meters using map resolution
        map_res = map_msg.info.resolution
        dist_map = dist_map_px * map_res

        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        optimized_path = [np.array(pt, dtype=float) for pt in path]

        for _ in range(max_iterations):
            new_path = []
            for i, pt in enumerate(optimized_path):
                # Fix start and goal points
                if i == 0 or i == len(optimized_path) - 1:
                    new_path.append(pt)
                    continue

                gx = (pt[0] - origin_x) / map_res
                gy = (pt[1] - origin_y) / map_res
                ix, iy = int(round(gx)), int(round(gy))

                if not (1 <= ix < map_msg.info.width - 1 and 1 <= iy < map_msg.info.height - 1):
                    new_path.append(pt)
                    continue

                # Calculate numerical gradient of distance map (row=y, col=x)
                dx = dist_map[iy, ix+1] - dist_map[iy, ix-1]
                dy = dist_map[iy+1, ix] - dist_map[iy-1, ix]
                grad = np.array([dx, dy])
                grad_norm = np.linalg.norm(grad)

                if grad_norm > 1e-5:
                    candidate_pt = pt + step_size * (grad / grad_norm)
                    # Verify the adjustment does not collide with obstacles
                    if not self.check_obs(optimized_path[i-1], candidate_pt, map_msg) and \
                       not self.check_obs(candidate_pt, optimized_path[i+1], map_msg):
                        new_path.append(candidate_pt)
                        continue

                new_path.append(pt)
            optimized_path = new_path

        return optimized_path


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
