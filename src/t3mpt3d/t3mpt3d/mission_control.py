import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PolygonStamped,Point32
import numpy as np
from functools import partial
import math as math



class MissionControlNode(Node):
    def __init__(self):
        super().__init__("mission_control")
        self.robots = ["tb3_1","tb3_2"]
        # 1. Declare parameters (base suffixes for topics)
        # Note: I removed the duplicate parameter declarations you had.
        # Also added "reached_topic" which was missing from the declare block.
        self.declare_parameter("exploration_trigger", "/exploration_trigger")
        self.declare_parameter("plank_pos", "/plank")
        self.declare_parameter("plank_reached", "/plank_reached")
        self.declare_parameter("riv_start_trigger", "/rivstart")
        self.declare_parameter("goal_pose", "/goal")
        self.declare_parameter("reached_topic", "/reached")
        self.declare_parameter("found_plank","/found")
        self.declare_parameter("low_fronteirs","/low")
        self.declare_parameter("move_to_plank","/move_to_plank")
        self.declare_parameter("auto_start", True)

        # 2. Get parameter values (these act as our base topic suffixes)
        exploration_trigger_suffix = self.get_parameter("exploration_trigger").value
        plank_pos_topic = self.get_parameter("plank_pos").value
        plank_reached_suffix = self.get_parameter("plank_reached").value
        reached_suffix = self.get_parameter("reached_topic").value
        goal_topic = self.get_parameter("goal_pose").value
        riv_start_topic = self.get_parameter("riv_start_trigger").value
        found_plank_topic = self.get_parameter("found_plank").value
        low_fronteir_topic = self.get_parameter("low_fronteirs").value
        move_to_plank_topic = self.get_parameter("move_to_plank").value
        self.auto_start =self.get_parameter("auto_start").value
        # 3. Create dictionaries to hold publishers and subscribers per robot
        plank_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                       reliability=ReliabilityPolicy.RELIABLE)

        self.goal_pose_publishers = {}
        self.model_trigger_publishers = {}
        self.exploration_trigger_publishers = {}

        self.plank_pos_subscriptions = {}
        self.plank_reached_subscriptions = {}
        self.reached_subscriptions = {}
        self.plank_pos1 = [-1.0,-1.0]
        self.plank_pos2 =[-1.0,-1.4]
        self.goal = [0,0,0]
        self.found_plank = False
        self.reached =[]
        self.started = False
        #publishers
        self.riv_start_publishers= self.create_publisher(
                                            Bool, riv_start_topic, 10
                                        )
        self.goal_publisher= self.create_publisher(
                                                    PoseStamped, goal_topic, 10
                                                )
        self.plank_pos_publisher = self.create_publisher(
                        PolygonStamped, plank_pos_topic, plank_qos
                    )
        self.move_to_plank_publisher = self.create_publisher(
                            Bool, move_to_plank_topic, plank_qos
                        )
        #subscribers
        self.found_plank_subcritption =self.create_subscription(
                        Bool, found_plank_topic,
                        self.found_plank_callback, plank_qos
                    )
        self.low_fronteirs_sbuscription = self.create_subscription(
                                Bool,low_fronteir_topic,
                            self.low_fronteirs_callback,plank_qos
                            )

        # 4. Loop through each robot to create its specific topics
        for robot in self.robots:
            # Construct the namespaced topics (e.g., "/tb3_1/plank_reached")

            explore_topic = f"/{robot}{exploration_trigger_suffix}"


            plank_reached_topic = f"/{robot}{plank_reached_suffix}"
            reached_topic = f"/{robot}{reached_suffix}"

            # --- Publishers ---
            self.goal_pose_publishers[robot] = self.create_publisher(
                PoseStamped, goal_topic, plank_qos
            )


            self.exploration_trigger_publishers[robot] = self.create_publisher(
                Bool, explore_topic, plank_qos
            )

            # --- Subscribers ---
            # We use partial() to pass the 'robot' variable into the callback
            # so the callback knows which robot sent the message.


            # self.plank_reached_subscriptions[robot] = self.create_subscription(
            #     Bool, plank_reached_topic,
            #     partial(self.plank_reached_callback, robot_name=robot), plank_qos
            # )
            self.reached_subscriptions[robot] = self.create_subscription(
                Bool, reached_topic,
                partial(self.reached_callback, robot_name=robot), plank_qos
            )


        self.get_logger().info(f"Mission Control initialized for robots: {self.robots}")
        if self.auto_start:
            self.start_timer = self.create_timer(10.0, self.start_robot)

    def start_robot(self):
        self.get_logger().info(f"is this a diesel engine because this shit rolling coal {self.auto_start}")
        self.start_timer.cancel()
        self.plank_pos_pub()

    def go_to_plank_pub(self):
        msg = Bool()
        msg.data = True
        self.move_to_plank_publisher.publish(msg)


    def low_fronteirs_callback(self,msg):
        if msg.data is True and self.found_plank is True:
            self.get_logger().info("This was the last seaweed, time to walk the plank")
            self.go_to_plank_pub()

    def found_plank_callback(self,msg):
        if msg.data is True:
            self.get_logger().info("No Time to walk the plank")
            self.found_plank = True


    def plank_pos_pub(self):
          # Add your logic here
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        # PolygonStamped uses Point32 (x, y, z as 32-bit floats)
        p1 = Point32(x=self.plank_pos1[0], y =self.plank_pos1[1], z=0.0)
        p2 = Point32(x=self.plank_pos2[0], y =self.plank_pos2[1], z=0.0)
        msg.polygon.points = [p1, p2]

        self.plank_pos_publisher.publish(msg)
        self.get_logger().info('Watch out this publisher will ricochet')

        #   self.mission_log.set_plank_leader(msg.pose.position, robot_name)

        #   target_pose, followers = self.mission_log.go_to_plank()

        #   if target_pose:
        #     for follower in followers:
        #         self.goal_pose_publish(follower, target_pose)



    def goal_pose_publish(self, goal: list) -> PoseStamped:
    # 1. Verify we have a publisher

        self.get_logger().info(f"OOOOOOONNNNNN BOOOOOAAARD!")
        # Construct the PoseStamped from list values
        msg = PoseStamped()

        # 2. Header setup
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Assume goal is [x, y, yaw]
        msg.pose.position.x = float(goal[0])
        msg.pose.position.y = float(goal[1])
        msg.pose.position.z = 0.0

        # Convert planar yaw to quaternion
        yaw = float(goal[2]) if len(goal) > 2 else 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        # 3. Publish
        self.goal_publisher.publish(msg)

        # 4. Return message
        return msg

    def reached_callback(self, msg, robot_name):
        self.get_logger().info(f"below deck ye sea cow! {robot_name}")
        if robot_name not in self.reached:
            self.reached.append(robot_name)

        if len(self.reached) == len(self.robots):
            enable =True
            self.started = self.clawstart_publish(enable)

            this = self.goal_pose_publish(self.goal)

    def clawstart_publish(self, enable:bool)->Bool:
        self.get_logger().info("This old scar Me got From Thi grandest of seaDogs in me water!")
        msg = Bool()
        msg.data = True

        self.riv_start_publishers.publish(msg)
        return msg





def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down mission control.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()