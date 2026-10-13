#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import Buffer, TransformListener



class MultiRobotOdomTransformer(Node):
    def __init__(self):
        super().__init__('odom_to_world_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        if not self.has_parameter('robots'):
            self.declare_parameter('robots', ['turtlebot1', 'turtlebot2'])

        self.declare_parameter('target_frame', '/turtlebot1/map')
        self.TARGET_FRAME = self.get_parameter('target_frame').get_parameter_value().string_value

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.map_found = False

        self.robots = self.get_parameter('robots').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subs = []
        self.pubs = {}

        for robot in self.robots:
            sub = self.create_subscription(
                Odometry,
                f'/{robot}/odom',
                lambda msg, r=robot: self.odom_callback(msg, r),
                10
            )

            self.subs.append(sub)

            pub = self.create_publisher(
                Odometry,
                f'/{robot}/odom_world',
                10
            )
            self.pubs[robot] = pub

        self.get_logger().info(f'Initialized odom transformer for robots: {self.robots}')

    def _on_map(self, msg):
        if not self.map_found:
            self.map = msg
            self.map_found = True

    def odom_callback(self, msg: Odometry, robot_name: str):
        try:
            target_frame = self.TARGET_FRAME
            source_frame = msg.header.frame_id

            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(seconds=0.0),
                timeout=Duration(seconds=0.1)
            )

            # Extract translation values from the transform
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # Construct and publish the new Odometry message
            new_odom = Odometry()
            new_odom.header.stamp = self.get_clock().now().to_msg()
            new_odom.header.frame_id = target_frame
            new_odom.child_frame_id = msg.child_frame_id

            # Apply transform translation to the pose position
            new_odom.pose.pose.position.x = msg.pose.pose.position.x + tx
            new_odom.pose.pose.position.y = msg.pose.pose.position.y + ty
            new_odom.pose.pose.position.z = msg.pose.pose.position.z + tz
            new_odom.pose.pose.orientation = msg.pose.pose.orientation

            new_odom.pose.covariance = msg.pose.covariance
            new_odom.twist = msg.twist

            self.pubs[robot_name].publish(new_odom)

        except Exception as e:
            self.get_logger().warn(f'Transform failed for {robot_name}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotOdomTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()