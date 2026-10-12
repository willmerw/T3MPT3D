import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PolygonStamped,Point32
import numpy as np
from functools import partial



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
        self.declare_parameter("model_trigger", "/model")
        self.declare_parameter("goal_pose", "/goal_pose")
        self.declare_parameter("reached_topic", "/reached")
        self.declare_parameter("found_plank","/found")
        self.declare_parameter("low_fronteirs","/found")
        self.declare_parameter("auto_start", False)

        # 2. Get parameter values (these act as our base topic suffixes)
        exploration_trigger_suffix = self.get_parameter("exploration_trigger").value
        plank_pos_topic = self.get_parameter("plank_pos").value
        plank_reached_suffix = self.get_parameter("plank_reached").value
        reached_suffix = self.get_parameter("reached_topic").value
        goal_pose_suffix = self.get_parameter("goal_pose").value
        model_trigger_suffix = self.get_parameter("model_trigger").value
        found_plank_suffix = self.get_parameter("found_plank").value
        low_fronteir_topic = self.get_parameter("low_fronteirs").value

        # 3. Create dictionaries to hold publishers and subscribers per robot
        self.goal_pose_publishers = {}
        self.model_trigger_publishers = {}
        self.exploration_trigger_publishers = {}

        self.plank_pos_subscriptions = {}
        self.plank_reached_subscriptions = {}
        self.reached_subscriptions = {}
        self.plank_pos1 = [2,3]
        self.plank_pos2 =[1,3]
        self.found_plank = False
        self.reached =[]
        #publishers
        self.model_trigger_publishers= self.create_publisher(
                                            Bool, model_topic, 10
                                        )
        self.plank_pos_publisher = self.create_publisher(
                        PolygonStamped, plank_pos_topic, 10
                    )
        #subscribers
        self.found_plank_subcritption =self.create_subscription(
                        Bool, found_plank_topic,
                        self.found_plank_callback, 10
                    )
        self.found_plank_subcritption =self.create_subscription(
                                Bool, found_plank_topic,
                                self.found_plank_callback, 10
                            )
        self.low_fronteirs_sbuscription = self.create_subscription(
                                Bool,found_plank_topic,
                                self.lowfronteirs,10
                            )
        # 4. Loop through each robot to create its specific topics
        for robot in self.robots:
            # Construct the namespaced topics (e.g., "/tb3_1/plank_reached")
            goal_topic = f"/{robot}{goal_pose_suffix}"
            model_topic = f"/{robot}{model_trigger_suffix}"
            explore_topic = f"/{robot}{exploration_trigger_suffix}"


            plank_reached_topic = f"/{robot}{plank_reached_suffix}"
            reached_topic = f"/{robot}{reached_suffix}"
            found_plank_topic =f"/{robot}{found_plank_suffix}"

            # --- Publishers ---
            self.goal_pose_publishers[robot] = self.create_publisher(
                PoseStamped, goal_topic, 10
            )


            self.exploration_trigger_publishers[robot] = self.create_publisher(
                Bool, explore_topic, 10
            )

            # --- Subscribers ---
            # We use partial() to pass the 'robot' variable into the callback
            # so the callback knows which robot sent the message.


            self.plank_reached_subscriptions[robot] = self.create_subscription(
                Bool, plank_reached_topic,
                partial(self.plank_reached_callback, robot_name=robot), 10
            )
            self.reached_subscriptions[robot] = self.create_subscription(
                Bool, reached_topic,
                partial(self.reached_callback, robot_name=robot), 10
            )


        self.get_logger().info(f"Mission Control initialized for robots: {self.robots}")
        if self.auto_start:
            self.start_timer = self.create_timer(1.0, self.start_robot)

    def start_robot(self):
        self.start_timer.cancel()
        self.plank_pos_pub()
    def go_to_plank_pub(self):
        msg = Bool()
        msg.data = True
        self.reached_pub.publish(msg)


    def plank_reached_callback(self, msg, robot_name):
        pass
    def low_fronteirs_callback(self,msg):
        if msg.data is True and self.found_plank is True:
            self.go_to_plank_pub()

    def found_plank_callback(self,msg):
        if msg.data is True:
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

        self.publisher_.publish(msg)
        self.get_logger().info('Published plank polygon')

        #   self.mission_log.set_plank_leader(msg.pose.position, robot_name)

        #   target_pose, followers = self.mission_log.go_to_plank()

        #   if target_pose:
        #     for follower in followers:
        #         self.goal_pose_publish(follower, target_pose)



    def goal_pose_publish(self, robot_name: str, goal: PoseStamped) -> PoseStamped:
        # 1. Verify we have a publisher for this robot
        if robot_name not in self.goal_pose_publishers:
            self.get_logger().error(f"Failed to publish: No goal publisher found for {robot_name}!")
            return None

        # 2. Update the timestamp to the current time (good ROS practice)
        goal.header.stamp = self.get_clock().now().to_msg()

        # Ensure a frame_id is set (usually "map" or "odom" depending on your setup)
        if not goal.header.frame_id:
            goal.header.frame_id = "map"

        # 3. Publish using the specific robot's publisher
        self.goal_pose_publishers[robot_name].publish(goal)

        self.get_logger().info(f"Published new goal pose for {robot_name}")

        # 4. Return the message as indicated by your type hint
        return goal

    def reached_callback(self, msg, robot_name):
        if robot_name not in self.reached:
            self.reached.append(robot_name) 
        pass
    def model_trigger_publish(self, robot_name:str, enable:bool)->Bool:
        if robot_name not in self.model_trigger_publishers:
            self.get_logger().error(f"No model publisher found for {robot_name}!")
            return None

        msg = Bool()
        msg.data = enable

        self.model_trigger_publishers.publish(msg)

        self.get_logger().info(f"Triggered model for {robot_name} (Value: {enable})")

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