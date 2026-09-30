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

        self.maper = None

    def set_plank_leader(self,plank_pos, player):
        if self.plank_pos is None:
            if player in self.players:
                self.plank_pos = plank_pos
                self.plank_leader = player


    def go_to_plank(self):
        if self.plank_pos is None:
            return
        return self.plank_pos, self.players[~self.plank_leader]

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
        pass

    def plank_pos_callback(self, msg, robot_name):
          self.get_logger().info(f"{robot_name} reached the plank! Value: {msg.data}")
          # Add your logic here
          self.mission_log.set_plank_leader(robot_name, robot_name)

          self.goal_pose_publish(robot_name,)



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

    def publish_pick(self) -> PoseStamped:
        if not self.pick_stations:
            self.get_logger().warn("No pick stations configured.")
            return None

        entry = self.pick_stations[self.current_pick_idx]
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        reference_topic = (
            self.get_parameter("reference_topic").get_parameter_value().string_value
        )

        pose_msg = entry.to_pose_stamped(
            frame_id=frame_id, stamp=self.get_clock().now().to_msg()
        )
        self.reference_publisher.publish(pose_msg)

        self.get_logger().info(
            f"[PICK] Published {entry.name} (brick {entry.brick_count}) "
            f"target {entry.coordinates} to '{reference_topic}'."
        )

        # Advance pick station and update state
        self.current_pick_idx = (self.current_pick_idx + 1) % len(self.pick_stations)
        self.current_action = "PLACE"
        self.state = "NAV_TO_PICK"
        self.reached = False
        return pose_msg

    def publish_place(self) -> PoseStamped:
        if not self.place_stations:
            self.get_logger().warn("No place stations configured.")
            return None

        entry = self.place_stations[self.current_place_idx]
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        reference_topic = (
            self.get_parameter("reference_topic").get_parameter_value().string_value
        )

        pose_msg = entry.to_pose_stamped(
            frame_id=frame_id, stamp=self.get_clock().now().to_msg()
        )
        self.reference_publisher.publish(pose_msg)

        self.get_logger().info(
            f"[PLACE] Published {entry.name} (brick {entry.brick_count}) "
            f"target {entry.coordinates} to '{reference_topic}'."
        )

        # Advance place station and update state
        self.current_place_idx = (self.current_place_idx + 1) % len(self.place_stations)
        self.current_action = "PICK"
        self.state = "NAV_TO_PLACE"
        self.reached = False
        return pose_msg

    def reached_callback(self, msg: Bool):
        """Called when robot target reached status is updated."""
        trigger_on_true_only = (
            self.get_parameter("trigger_on_true_only").get_parameter_value().bool_value
        )
        self.reached = msg.data
        if trigger_on_true_only and not msg.data:
            return

        pick_ready_topic = (
            self.get_parameter("pick_ready_topic").get_parameter_value().string_value
        )
        place_ready_topic = (
            self.get_parameter("place_ready_topic").get_parameter_value().string_value
        )

        self.get_logger().info(
            f"Target reached signal received: {msg.data} (current state: {self.state})"
        )

        if self.state == "NAV_TO_PICK":
            self.state = "PICKING"
            self.get_logger().info(
                f"Pick position reached! Publishing True to '{pick_ready_topic}'."
            )
            self.pick_ready_publisher.publish(Bool(data=True))

        elif self.state == "NAV_TO_PLACE":
            self.state = "PLACING"
            self.get_logger().info(
                f"Place position reached! Publishing True to '{place_ready_topic}'."
            )
            self.place_ready_publisher.publish(Bool(data=True))

        else:
            self.get_logger().info(
                f"Target reached received while in state '{self.state}'; not in a navigation state."
            )

    def pick_done_callback(self, msg: Bool):
        """Called when pick mission finishes -> publishes Place pose."""
        trigger_on_true_only = (
            self.get_parameter("trigger_on_true_only").get_parameter_value().bool_value
        )
        if trigger_on_true_only and not msg.data:
            return

        self.get_logger().info("Pick mission completed! Sending Place reference pose.")
        self.publish_place()

    def place_done_callback(self, msg: Bool):
        """Called when place mission finishes -> publishes next Pick pose."""
        trigger_on_true_only = (
            self.get_parameter("trigger_on_true_only").get_parameter_value().bool_value
        )
        if trigger_on_true_only and not msg.data:
            return

        self.get_logger().info(
            "Place mission completed! Sending next Pick reference pose."
        )
        self.publish_pick()

    def trigger_callback(self, msg: Bool):
        """Manual / start trigger to start or advance the cycle."""
        trigger_on_true_only = (
            self.get_parameter("trigger_on_true_only").get_parameter_value().bool_value
        )
        if trigger_on_true_only and not msg.data:
            self.get_logger().info("Manual trigger received with data=False. Ignoring.")
            return

        self.get_logger().info(
            f"Manual trigger received. Current action: {self.current_action}, state: {self.state}"
        )
        if self.state == "IDLE":
            self.publish_pick()
        elif self.current_action == "PICK":
            self.publish_pick()
        else:
            self.publish_place()


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