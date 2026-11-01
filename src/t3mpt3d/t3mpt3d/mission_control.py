import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
import numpy as np
from functools import partial
import math


class MissionControlNode(Node):
    def __init__(self):
        super().__init__("mission_control")

        self.declare_parameter("exploration_trigger", "/exploration_trigger")
        self.declare_parameter("plank_pos", "/plank")
        self.declare_parameter("plank_reached", "/plank_reached")
        self.declare_parameter("riv_start_trigger", "/rivstart")
        self.declare_parameter("goal_pose", "/goal")
        self.declare_parameter("reached_topic", "/reached")
        self.declare_parameter("found_plank", "/found")
        self.declare_parameter("low_frontiers", "/low")
        self.declare_parameter("move_to_plank", "/move_to_plank")
        self.declare_parameter("auto_start", True)

        # Declare the position of the plank as two points
        self.declare_parameter("plank_pos1", [2.5, -0.3])
        self.declare_parameter("plank_pos2", [2.5, 0.2])
        self.declare_parameter("goal", [0.0, 0.0, 0.0])
        
        exploration_trigger_suffix = self.get_parameter("exploration_trigger").value
        plank_pos_topic = self.get_parameter("plank_pos").value
        plank_reached_suffix = self.get_parameter("plank_reached").value
        reached_suffix = self.get_parameter("reached_topic").value
        goal_topic = self.get_parameter("goal_pose").value
        riv_start_topic = self.get_parameter("riv_start_trigger").value
        found_plank_topic = self.get_parameter("found_plank").value
        low_frontier_topic = self.get_parameter("low_frontiers").value
        move_to_plank_topic = self.get_parameter("move_to_plank").value
        self.auto_start = self.get_parameter("auto_start").value
        self.plank_pos1 = self.get_parameter("plank_pos1").value
        self.plank_pos2 = self.get_parameter("plank_pos2").value
        self.goal = self.get_parameter("goal").value

        # QoS profile suitable for latching one-time messages like state flags
        plank_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)

        self.reached_subscriptions = {}

        # State flags representing the states received from the navigation node
        self.low = False
        self.found_plank = False
        self.started = False

        self.reached = []
        
        # List of robot namespaces in use
        self.robots = ["tb3_1", "tb3_2"]
        
        # Publishers
        self.riv_start_publishers = self.create_publisher(
            Bool, riv_start_topic, 10
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, goal_topic, 10
        )
        self.plank_pos_publisher = self.create_publisher(
            PolygonStamped, plank_pos_topic, plank_qos
        )
        self.move_to_plank_publisher = self.create_publisher(
            Bool, move_to_plank_topic, plank_qos
        )
        
        # Subscribers
        self.found_plank_subcritption = self.create_subscription(
            Bool, found_plank_topic,
            self.found_plank_callback, plank_qos
        )
        self.low_frontiers_subscription = self.create_subscription(
            Bool, low_frontier_topic,
            self.low_frontiers_callback, plank_qos
        )

        # Loop through each robot to create its specific subscriptions
        for robot in self.robots:
            reached_topic = f"/{robot}{reached_suffix}"
            self.reached_subscriptions[robot] = self.create_subscription(
                Bool, reached_topic,
                partial(self.reached_callback, robot_name=robot), plank_qos
            )

        self.get_logger().info(f"Mission Control initialized for robots: {self.robots}")
        
        # Auto-start with a 10-second timer
        if self.auto_start:
            self.start_timer = self.create_timer(10.0, self.start_robot)

    def start_robot(self):
        # Start the robots by publishing the initial plank position to the navigation node
        self.get_logger().info(f"Auto-start triggered (Status: {self.auto_start}). Publishing initial coordinates.")
        self.start_timer.cancel()
        self.plank_pos_pub()

    def go_to_plank_pub(self):
        # Publish the command instructing the robots to navigate to the plank position
        msg = Bool()
        msg.data = True
        self.move_to_plank_publisher.publish(msg)
        self.get_logger().info("Dispatched move-to-plank command to all robots.")

    def found_plank_callback(self, msg):
        # Callback triggered when a robot has successfully located the plank
        if msg.data is True:
            self.get_logger().info("Plank position has been identified by the navigation node.")
            self.found_plank = True
            
        # If the plank is found and exploration frontiers are low, command robots to move to the plank
        if self.low is True and self.found_plank is True:
            self.get_logger().info("Exploration threshold met and plank identified. Initiating movement to plank.")
            self.go_to_plank_pub()

    def low_frontiers_callback(self, msg):
        # Callback triggered when the exploration node reports low remaining frontiers
        if msg.data is True:
            self.get_logger().info("Low frontiers detected. Exploration phase nearing completion.")
            self.low = True
            
        # If the plank is found and exploration frontiers are low, command robots to move to the plank
        if self.low is True and self.found_plank is True:
            self.get_logger().info("Exploration threshold met and plank identified. Initiating movement to plank.")
            self.go_to_plank_pub()

    def plank_pos_pub(self):
        # Publish the polygon coordinates defining the plank position
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        # PolygonStamped utilizes Point32 (x, y, z as 32-bit floats)
        p1 = Point32(x=self.plank_pos1[0], y=self.plank_pos1[1], z=0.0)
        p2 = Point32(x=self.plank_pos2[0], y=self.plank_pos2[1], z=0.0)
        msg.polygon.points = [p1, p2]

        self.plank_pos_publisher.publish(msg)
        self.get_logger().info("Plank polygon coordinates published to 'world' frame.")

    def goal_pose_publish(self, goal: list) -> PoseStamped:
        # Publish the final goal pose for the path planner
        self.get_logger().info("Publishing final goal pose to the navigation stack.")
    
        msg = PoseStamped()

        # Header setup
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Goal is configured as [x, y, yaw]
        msg.pose.position.x = float(goal[0])
        msg.pose.position.y = float(goal[1])
        msg.pose.position.z = 0.0

        # Convert yaw to quaternion
        yaw = float(goal[2]) if len(goal) > 2 else 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_publisher.publish(msg)
        return msg

    def reached_callback(self, msg, robot_name):
        # Namespaced callback triggered when a specific robot reaches the target position
        self.get_logger().info(f"Robot '{robot_name}' has reached the designated position.")
        
        if robot_name not in self.reached:
            self.reached.append(robot_name)
            
        # If all active robots have reached the plank, transition MPC modes
        if len(self.reached) == len(self.robots):
            self.get_logger().info("All robots have reached the objective. Transitioning control states.")
            enable = True
            self.started = self.clawstart_publish(enable)
            self.goal_pose_publish(self.goal)

    def clawstart_publish(self, enable: bool) -> Bool:
        # Publish the trigger to start the plank MPC and disable multi-robot aware MPC
        self.get_logger().info("Triggering MPC mode switch via riv_start publisher.")
        msg = Bool()
        msg.data = enable

        self.riv_start_publishers.publish(msg)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down Mission Control node.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()