import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import numpy as np

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

        # Parameters for configuration
        #subs
        self.declare_parameter("exploration_trigger", "/exploartion_trigger")
        self.declare_parameter("plank_pos", "/plank_pose")
        self.declare_parameter("plank_reached", "/plank_reached")
        self.declare_parameter("model_trigger", "/model")

        #pubs
        self.declare_parameter("goal_pose", "/goal_pose")
        self.declare_parameter("exploration_trigger", "/exploartion_trigger")
        self.declare_parameter("model_trigger", "/model")



        exploration_trigger_topic = (
            self.get_parameter("exploration_trigger").get_parameter_value().string_value
        )
        plank_pos_topic = (
            self.get_parameter("plank_pos").get_parameter_value().string_value
        )
        plank_reached_topic = (
            self.get_parameter("plank_reached").get_parameter_value().string_value
        )
        reached_topic = (
            self.get_parameter("reached_topic").get_parameter_value().string_value
        )

        # Pubs
        goal_pose_topic = (
            self.get_parameter("goal_pose").get_parameter_value().string_value
        )
        model_trigger_topic = (
            self.get_parameter("model_trigger").get_parameter_value().string_value
        )

        # Publishers
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, goal_pose_topic, 10
        )
        self.model_trigger_publisher = self.create_publisher(
            Bool, model_trigger_topic, 10
        )
        self.exploration_trigger_publisher = self.create_publisher(
            Bool, exploration_trigger_topic, 10
        )

        # Subscribers
        self.plank_pos_subscription = self.create_subscription(
            PoseStamped, plank_pos_topic, self.plank_pos_callback, 10
        )
        self.plank_reached_subscription = self.create_subscription(
            Bool, plank_reached_topic, self.plank_reached_callback, 10
        )
        self.reached_subscription = self.create_subscription(
            Bool, reached_topic, self.reached_callback, 10
        )

        self.get_logger().info(
            f"Node initialized.\n"
            f"  Goal Pose Pub:           {goal_pose_topic}\n"
            f"  Model Trigger Pub:       {model_trigger_topic}\n"
            f"  Exploration Trigger Pub: {exploration_trigger_topic}\n"
            f"  Plank Pos Sub:           {plank_pos_topic}\n"
            f"  Plank Reached Sub:       {plank_reached_topic}\n"
            f"  Reached Sub:             {reached_topic}"
        )
        if self.get_parameter("auto_start").get_parameter_value().bool_value:
            self.get_logger().info("Auto-start enabled. Publishing initial pick pose.")
            self.publish_pick()

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