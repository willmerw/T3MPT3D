#!/usr/bin/env python3
# multi_map_merge.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import message_filters
import cv2
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException

class MultiMapMerge(Node):
    def __init__(self):
        super().__init__('multi_map_merge')

        self.declare_parameter("tb1_map_topic", "tb1_map_topic")
        self.declare_parameter("tb2_map_topic", "tb2_map_topic")

        self.declare_parameter("tb1_odom_world", "turtlebot1/odom_world")
        self.declare_parameter("tb2_odom_world", "turtlebot2/odom_world")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        tb1: str = self.get_parameter("tb1_map_topic").get_parameter_value().string_value
        tb2: str = self.get_parameter("tb2_map_topic").get_parameter_value().string_value

        tb1_odom_world: str = self.get_parameter("tb1_odom_world").get_parameter_value().string_value
        tb2_odom_world: str = self.get_parameter("tb2_odom_world").get_parameter_value().string_value

        self.p1 = []
        self.p2 = []

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

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.sub_map1 = message_filters.Subscriber(self, OccupancyGrid, tb1, qos_profile=map_qos)
        self.sub_map2 = message_filters.Subscriber(self, OccupancyGrid, tb2, qos_profile=map_qos)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_map1, self.sub_map2],
            queue_size=10,
            slop=0.5
        )
        self.ts.registerCallback(self.map_callback)

        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        self.get_logger().info("Map Synchronizer and TF2 Listener started.")

    def odom_callback(self,odom1_msg,odom2_msg):
        self.p1[0] = odom1_msg.pose.pose.position.x
        self.p1[1] = odom1_msg.pose.pose.position.y

        self.p2[0] = odom2_msg.pose.pose.position.x
        self.p2[1] = odom2_msg.pose.pose.position.y

    def map_callback(self, map1_msg: OccupancyGrid, map2_msg: OccupancyGrid):
        frame1 = map1_msg.header.frame_id.lstrip('/')
        frame2 = map2_msg.header.frame_id.lstrip('/')

        try:
            t = self.tf_buffer.lookup_transform(frame1, frame2, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {frame2} to {frame1}: {ex}')
            return

        res = map1_msg.info.resolution

        # 1. Global spatial boundaries for map 1 in frame1
        m1_min_x = map1_msg.info.origin.position.x
        m1_min_y = map1_msg.info.origin.position.y
        m1_max_x = m1_min_x + map1_msg.info.width * res
        m1_max_y = m1_min_y + map1_msg.info.height * res

        # 2. Global spatial boundaries for map 2 transformed into frame1
        m2_min_x = t.transform.translation.x + map2_msg.info.origin.position.x
        m2_min_y = t.transform.translation.y + map2_msg.info.origin.position.y
        m2_max_x = m2_min_x + map2_msg.info.width * res
        m2_max_y = m2_min_y + map2_msg.info.height * res

        # 3. Compute union bounding box
        union_min_x = min(m1_min_x, m2_min_x)
        union_min_y = min(m1_min_y, m2_min_y)
        union_max_x = max(m1_max_x, m2_max_x)
        union_max_y = max(m1_max_y, m2_max_y)

        union_w = int(np.ceil((union_max_x - union_min_x) / res))
        union_h = int(np.ceil((union_max_y - union_min_y) / res))

        # 4. Compute pixel translation vectors relative to the new origin
        dx1_px = int(round((m1_min_x - union_min_x) / res))
        dy1_px = int(round((m1_min_y - union_min_y) / res))

        dx2_px = int(round((m2_min_x - union_min_x) / res))
        dy2_px = int(round((m2_min_y - union_min_y) / res))

        # 5. Extract grid arrays
        grid1 = np.array(map1_msg.data, dtype=np.int16).reshape((map1_msg.info.height, map1_msg.info.width))
        grid2 = np.array(map2_msg.data, dtype=np.int16).reshape((map2_msg.info.height, map2_msg.info.width))

        # 6. Warp both grids onto the unified dynamic canvas
        M1 = np.float32([[1, 0, dx1_px], [0, 1, dy1_px]])
        M2 = np.float32([[1, 0, dx2_px], [0, 1, dy2_px]])

        aligned_g1 = cv2.warpAffine(grid1, M1, (union_w, union_h), borderValue=-1)
        aligned_g2 = cv2.warpAffine(grid2, M2, (union_w, union_h), borderValue=-1)

        # 7. Merge maps (prioritize known values over unknown -1)
        merged = np.where(aligned_g1 != -1, aligned_g1, aligned_g2)

        #Clear free space round robots
        robot_coords = [
            (self.p1[0], self.p1[1]),
            (self.p2[0], self.p2[1])
        ]

        for rx, ry in robot_coords:
            cx = int(round((rx - union_min_x) / res))
            cy = int(round((ry - union_min_y) / res))

            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < union_w and 0 <= ny < union_h:
                        merged[ny, nx] = 0  # free space

        # 8. Construct output OccupancyGrid message
        out_msg = OccupancyGrid()
        out_msg.header = map1_msg.header
        out_msg.info.resolution = res
        out_msg.info.width = union_w
        out_msg.info.height = union_h
        out_msg.info.origin.position.x = union_min_x
        out_msg.info.origin.position.y = union_min_y
        out_msg.info.origin.position.z = map1_msg.info.origin.position.z
        out_msg.info.origin.orientation = map1_msg.info.origin.orientation
        out_msg.data = merged.astype(np.int8).flatten().tolist()

        self.map_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MultiMapMerge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()