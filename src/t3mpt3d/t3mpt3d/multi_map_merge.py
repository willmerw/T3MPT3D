#!/usr/bin/env python3
# multi_map_merge.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import message_filters
import cv2  # Needed for shifting the map array
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

class MultiMapMerge(Node):
    def __init__(self):
        super().__init__('multi_map_merge')

        # 1. TF2 Setup to lookup frame differences
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 2. QoS Profile for Maps
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.sub_map1 = message_filters.Subscriber(self, OccupancyGrid, '/tb3_1/map', qos_profile=map_qos)
        self.sub_map2 = message_filters.Subscriber(self, OccupancyGrid, '/tb3_2/map', qos_profile=map_qos)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_map1, self.sub_map2],
            queue_size=10,
            slop=0.5
        )
        self.ts.registerCallback(self.map_callback)

        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        self.get_logger().info("Map Synchronizer and TF2 Listener started.")

    def map_callback(self, map1_msg: OccupancyGrid, map2_msg: OccupancyGrid):
        # 1. Get the frame IDs from the map messages
        frame1 = map1_msg.header.frame_id
        frame2 = map2_msg.header.frame_id

        # 2. STRIP THE LEADING SLASH!
        # ROS 2 TF2 will fail if frames start with '/'
        if frame1.startswith('/'):
            frame1 = frame1[1:]
        if frame2.startswith('/'):
            frame2 = frame2[1:]

        try:
            # Lookup transform using the cleaned frame names (e.g. 'tb3_1/map')
            t = self.tf_buffer.lookup_transform(
                frame1,
                frame2,
                rclpy.time.Time() # Get the latest available transform
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {frame2} to {frame1}: {ex}')
            return

        # Extract map data
        grid1 = np.array(map1_msg.data, dtype=np.int16).reshape((map1_msg.info.height, map1_msg.info.width))
        grid2 = np.array(map2_msg.data, dtype=np.int16).reshape((map2_msg.info.height, map2_msg.info.width))
        res = map1_msg.info.resolution

        # Calculate the real-world offset in meters
        dx_meters = t.transform.translation.x + map2_msg.info.origin.position.x - map1_msg.info.origin.position.x
        dy_meters = t.transform.translation.y + map2_msg.info.origin.position.y - map1_msg.info.origin.position.y

        # Convert meter offset to pixel/cell offset
        dx_pixels = int(dx_meters / res)
        dy_pixels = int(dy_meters / res)

        # Transformation matrix for translation
        M = np.float32([
            [1, 0, dx_pixels],
            [0, 1, dy_pixels]
        ])

        # Warp grid2 to match grid1's dimensions and alignment
        aligned_grid2 = cv2.warpAffine(
            grid2,
            M,
            (map1_msg.info.width, map1_msg.info.height),
            borderValue=-1
        )

        # Merge the grids
        merge_grid = np.where(grid1 != -1, grid1, aligned_grid2)

        # Publish the merged map
        out_msg = OccupancyGrid()
        # Keep the exact same header the first map used, so other nodes recognize it
        out_msg.header = map1_msg.header
        out_msg.info = map1_msg.info
        out_msg.data = merge_grid.astype(np.int8).flatten().tolist()

        self.map_pub.publish(out_msg)
        self.get_logger().info("Aligned and merged map published.")

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