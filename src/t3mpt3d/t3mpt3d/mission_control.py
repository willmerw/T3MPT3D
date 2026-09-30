import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import numpy as np
from functools import partial
class Robot_log():
    def __init__(self, robot_name:str):
        pass
class MissionLog():
    def __init__(self, players:list):
        self.plank_leader = None
        self.players = players
        self.plank_pos = None
        self.num_palyers = len(self.players)

        self.plankers = []

        self.maper = None

    def set_plank_leader(self,plank_pos, player):
        if self.plank_pos is None:
            if player in self.players:
                self.plank_pos = plank_pos
                self.plank_leader = player
                self.plankers.append(self.plank_leader)


    def go_to_plank(self):
        if self.plank_pos is None:
            return None, []
        
        # FIX: Find all players who are NOT the leader
        other_players = [p for p in self.players if p != self.plank_leader]
        
        return self.plank_pos, other_players
    def redgister(self, robot_name:bool):
        if len(self.plankers) == len(self.players):
            return True
        elif robot_name in self.plankers:
            return True
        else:
            self.plankers.append(robot_name)
            return False
    

class MissionControlNode(Node):
    def __init__(self):
        super().__init__("mission_control")
        self.robots = ["tb3_1", "tb3_2"]
        self.mission_log = MissionLog(self.robots)

        # 1. Declare parameters (base suffixes for topics)
        # Note: I removed the duplicate parameter declarations you had.
        # Also added "reached_topic" which was missing from the declare block.
        self.declare_parameter("exploration_trigger", "/exploration_trigger")
        self.declare_parameter("plank_pos", "/plank_pose")
        self.declare_parameter("plank_reached", "/plank_reached")
        self.declare_parameter("model_trigger", "/model")
        self.declare_parameter("goal_pose", "/goal_pose")
        self.declare_parameter("reached_topic", "/reached")
        self.declare_parameter("auto_start", False)

        # 2. Get parameter values (these act as our base topic suffixes)
        exploration_trigger_suffix = self.get_parameter("exploration_trigger").value
        plank_pos_suffix = self.get_parameter("plank_pos").value
        plank_reached_suffix = self.get_parameter("plank_reached").value
        reached_suffix = self.get_parameter("reached_topic").value
        goal_pose_suffix = self.get_parameter("goal_pose").value
        model_trigger_suffix = self.get_parameter("model_trigger").value

        # 3. Create dictionaries to hold publishers and subscribers per robot
        self.goal_pose_publishers = {}
        self.model_trigger_publishers = {}
        self.exploration_trigger_publishers = {}

        self.plank_pos_subscriptions = {}
        self.plank_reached_subscriptions = {}
        self.reached_subscriptions = {}

        # 4. Loop through each robot to create its specific topics
        for robot in self.robots:
            # Construct the namespaced topics (e.g., "/tb3_1/plank_reached")
            goal_topic = f"/{robot}{goal_pose_suffix}"
            model_topic = f"/{robot}{model_trigger_suffix}"
            explore_topic = f"/{robot}{exploration_trigger_suffix}"

            plank_pos_topic = f"/{robot}{plank_pos_suffix}"
            plank_reached_topic = f"/{robot}{plank_reached_suffix}"
            reached_topic = f"/{robot}{reached_suffix}"

            # --- Publishers ---
            self.goal_pose_publishers[robot] = self.create_publisher(
                PoseStamped, goal_topic, 10
            )
            self.model_trigger_publishers[robot] = self.create_publisher(
                Bool, model_topic, 10
            )
            self.exploration_trigger_publishers[robot] = self.create_publisher(
                Bool, explore_topic, 10
            )

            # --- Subscribers ---
            # We use partial() to pass the 'robot' variable into the callback
            # so the callback knows which robot sent the message.
            self.plank_pos_subscriptions[robot] = self.create_subscription(
                PoseStamped, plank_pos_topic,
                partial(self.plank_pos_callback, robot_name=robot), 10
            )
            self.plank_reached_subscriptions[robot] = self.create_subscription(
                Bool, plank_reached_topic,
                partial(self.plank_reached_callback, robot_name=robot), 10
            )
            self.reached_subscriptions[robot] = self.create_subscription(
                Bool, reached_topic,
                partial(self.reached_callback, robot_name=robot), 10
            )

        self.get_logger().info(f"Mission Control initialized for robots: {self.robots}")

    def plank_reached_callback(self, msg, robot_name):
        if self.mission_log.redgister(robot_name) is True:
            self.model_trigger_publish()

    def plank_pos_callback(self, msg, robot_name):
          self.get_logger().info(f"{robot_name} reached the plank! Value: {msg.data}")
          # Add your logic here
          self.mission_log.set_plank_leader(msg, robot_name)

          target_pose, followers = self.mission_log.go_to_plank()

          if target_pose:
            for follower in followers:
                self.goal_pose_publish(follower, target_pose)



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
        pass
    def model_trigger_publish(self, robot_name:str, enable:bool)->Bool:
        if robot_name not in self.model_trigger_publishers:
            self.get_logger().error(f"No model publisher found for {robot_name}!")
            return None

        msg = Bool()
        msg.data = enable

        self.model_trigger_publishers[robot_name].publish(msg)
        
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