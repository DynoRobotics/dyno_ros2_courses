import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from dynoturtle_interfaces.action import MoveTo


class GoalPoseClient:
    """
    Node that subscribes to /goal_pose topic and sends MoveTo action goals

    This node acts as a bridge between pose messages published to /goal_pose
    and the MoveTo action server. When a new pose is received, it automatically
    sends it as a goal to the MoveTo action server.
    """

    def __init__(self, node: Node):
        self.node = node
        self.node._default_callback_group = ReentrantCallbackGroup()
        self.logger = self.node.get_logger()

        self.logger.info("Initializing goal pose client")

        # State management
        self.current_goal_handle = None
        self.is_processing_goal = False

        # Configuration
        self.default_speed = 0.5  # m/s
        self.use_speed = True

        # Subscriber to /goal_pose topic
        self.goal_pose_subscriber = self.node.create_subscription(
            msg_type=PoseStamped,
            topic="/goal_pose",
            callback=self.goal_pose_callback,
            qos_profile=10,
        )

        # Action client for MoveTo action
        self.move_to_client = ActionClient(self.node, MoveTo, "move_to")

        # Wait for action server to be available
        self.logger.info("Waiting for MoveTo action server...")
        if self.move_to_client.wait_for_server(timeout_sec=10.0):
            self.logger.info("MoveTo action server is available")
        else:
            self.logger.error("MoveTo action server not available after 10 seconds")

    def goal_pose_callback(self, msg: PoseStamped):
        """
        Callback for /goal_pose topic subscription

        Args:
            msg: PoseStamped message containing the target pose
        """
        self.logger.info(
            f"Received goal pose: x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f}"
        )

        # Cancel any existing goal if one is running
        if self.is_processing_goal and self.current_goal_handle is not None:
            self.logger.info("Canceling previous goal to send new one")
            self.current_goal_handle.cancel_goal_async()

        # Send new MoveTo goal
        self.send_move_to_goal(msg)

    def send_move_to_goal(self, pose_stamped: PoseStamped):
        """
        Send a MoveTo action goal

        Args:
            pose_stamped: Target pose to move to
        """
        # Create MoveTo goal
        goal = MoveTo.Goal()
        goal.pose = pose_stamped
        goal.use_speed = self.use_speed
        goal.speed = self.default_speed

        self.logger.info(
            f"Sending MoveTo goal to ({pose_stamped.pose.position.x:.2f}, "
            f"{pose_stamped.pose.position.y:.2f}) at {self.default_speed} m/s"
        )

        # Send goal asynchronously
        self.is_processing_goal = True
        send_goal_future = self.move_to_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback for goal response

        Args:
            future: Future containing the goal handle
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.warn("MoveTo goal was rejected")
            self.is_processing_goal = False
            return

        self.logger.info("MoveTo goal accepted")
        self.current_goal_handle = goal_handle

        # Get result asynchronously
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback for goal result

        Args:
            future: Future containing the action result
        """
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.logger.info(
                f"MoveTo goal succeeded! "
                f"Time: {result.total_time_sec:.2f}s, "
                f"Distance: {result.distance_traveled:.2f}m"
            )
        elif status == 2:  # CANCELED
            self.logger.info("MoveTo goal was canceled")
        elif status == 3:  # ABORTED
            self.logger.warn("MoveTo goal was aborted")
        else:
            self.logger.warn(f"MoveTo goal finished with status: {status}")

        self.is_processing_goal = False
        self.current_goal_handle = None

    def feedback_callback(self, feedback_msg):
        """
        Callback for action feedback

        Args:
            feedback_msg: Feedback message from the action
        """
        feedback = feedback_msg.feedback
        self.logger.info(
            f"MoveTo progress: {feedback.progress:.1%}, "
            f"distance: {feedback.current_distance:.2f}m"
        )

    def set_speed(self, speed: float):
        """
        Set the default speed for MoveTo goals

        Args:
            speed: Speed in m/s (must be positive)
        """
        if speed > 0:
            self.default_speed = speed
            self.logger.info(f"Default speed set to {speed} m/s")
        else:
            self.logger.warn(f"Invalid speed: {speed} (must be positive)")

    def set_use_speed(self, use_speed: bool):
        """
        Set whether to use custom speed or default action speed

        Args:
            use_speed: True to use custom speed, False for default
        """
        self.use_speed = use_speed
        self.logger.info(f"Use custom speed: {use_speed}")


def main(args=None, namespace=""):
    """
    Main entry point for the goal pose client node

    Args:
        args: Command line arguments
        namespace: ROS namespace for the node
    """
    rclpy.init(args=args)
    node = Node("goal_pose_client", namespace=namespace)

    # Create the goal pose client
    goal_pose_client = GoalPoseClient(node)

    # Optional: Configure parameters
    goal_pose_client.set_speed(0.5)  # 0.5 m/s default speed
    goal_pose_client.set_use_speed(True)  # Use custom speed

    try:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    # Run with default namespace
    main()

    # Alternative: Run with specific namespace
    # main(namespace="turtle1")
