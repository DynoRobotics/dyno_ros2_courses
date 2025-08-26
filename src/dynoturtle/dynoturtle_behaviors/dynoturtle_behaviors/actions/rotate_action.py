import rclpy
import math
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import (
    ExternalShutdownException,
    SingleThreadedExecutor,
    MultiThreadedExecutor,
)
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Duration
from async_utils.async_primitives import async_sleep
from transform_utils.transforms import quaternion_from_euler

from geometry_msgs.msg import Twist, PoseStamped
from dynoturtle_interfaces.action import Rotate
from transform_utils.transforms import yaw_from_quaternion


class RotateAction:
    def __init__(self, node: Node):
        self.node = node
        self.node._default_callback_group = ReentrantCallbackGroup()
        self.logger = self.node.get_logger()

        self.logger.info("Initializing rotate action")

        # State
        self.is_processing_goal = False
        self.current_pose = None
        self.start_angle = None
        self.target_angle = None

        # Constants
        self.stop_msg = Twist()
        self.angle_tolerance = 0.05  # radians (~3 degrees)

        # Publishers
        self.cmd_vel_publisher = self.node.create_publisher(
            msg_type=Twist, topic="safe_cmd_vel", qos_profile=10
        )

        # Subscribers
        self.pose_subscriber = self.node.create_subscription(
            msg_type=PoseStamped,
            topic="pose3d",
            callback=self.pose_callback,
            qos_profile=10,
        )

        # Action server
        self.action_server = ActionServer(
            node=node,
            action_type=Rotate,
            action_name="rotate",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def pose_callback(self, msg: PoseStamped):
        """Callback to update current pose from pose3d topic"""
        # Extract yaw angle from quaternion
        self.current_pose = yaw_from_quaternion(msg.pose.orientation)

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def angle_difference(self, target: float, current: float) -> float:
        """Calculate the shortest angular difference between two angles"""
        diff = target - current
        return self.normalize_angle(diff)

    def has_reached_target_angle(self) -> bool:
        """Check if current angle has reached the target angle within tolerance"""
        if self.current_pose is None or self.target_angle is None:
            return False

        angle_diff = abs(self.angle_difference(self.target_angle, self.current_pose))
        return angle_diff <= self.angle_tolerance

    async def goal_callback(self, goal_request: Rotate.Goal) -> GoalResponse:
        if self.is_processing_goal:
            self.logger.warn("Rotate action is already in progress, rejecting new goal")
            return GoalResponse.REJECT

        # Validate goal parameters
        if goal_request.delta_angle == 0:
            self.logger.warn("Invalid angle: delta_angle cannot be zero")
            return GoalResponse.REJECT

        # Convert to radians for validation if needed
        angle_radians = (
            goal_request.delta_angle
            if goal_request.radians
            else goal_request.delta_angle * math.pi / 180
        )

        # Check for reasonable rotation limits (max 4 full rotations)
        max_rotation = 4 * 2 * math.pi  # 4 full rotations in radians
        if abs(angle_radians) > max_rotation:
            angle_degrees = (
                abs(goal_request.delta_angle)
                if goal_request.radians
                else abs(goal_request.delta_angle)
            )
            unit = "radians" if goal_request.radians else "degrees"
            self.logger.warn(
                f"Invalid angle: {angle_degrees} {unit} exceeds maximum allowed rotation of {4 * 360} degrees"
            )
            return GoalResponse.REJECT

        self.logger.info("Accepted new goal")
        return GoalResponse.ACCEPT

    async def cancel_callback(self, goal_handle: ServerGoalHandle) -> Rotate.Result:
        if not self.is_processing_goal:
            self.logger.warn("Trying to cancel a goal but we not running any")
            return CancelResponse.REJECT
        self.logger.info("Goal cancel request accepted")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle) -> Rotate.Result:
        start_timestamp = time.time()
        self.is_processing_goal = True

        # Wait for pose data to be available
        while self.current_pose is None:
            await async_sleep(self.node, 0.1)
            if goal_handle.is_cancel_requested:
                self.is_processing_goal = False
                goal_handle.canceled()
                res = Rotate.Result()
                res.total_time_sec = time.time() - start_timestamp
                return res

        try:
            goal: Rotate.Goal = goal_handle.request
            delta_angle_radians = (
                goal.delta_angle if goal.radians else goal.delta_angle * math.pi / 180
            )

            # Set start angle and calculate target angle
            self.start_angle = self.current_pose
            self.target_angle = self.normalize_angle(
                self.start_angle + delta_angle_radians
            )

            self.logger.info(
                f"Starting rotation: current={self.start_angle:.3f}, target={self.target_angle:.3f}, delta={delta_angle_radians:.3f}"
            )

            await self.rotate(delta_angle_radians, goal_handle)
        except Exception as e:
            self.logger.error(f"Rotate action failed: {e}")
            goal_handle.abort()
        else:
            if goal_handle.is_cancel_requested:
                self.logger.info("Rotate action canceled")
                goal_handle.canceled()
            else:
                self.logger.info("Rotate action succeeded")
                goal_handle.succeed()

        self.is_processing_goal = False

        res = Rotate.Result()
        res.total_time_sec = time.time() - start_timestamp

        return res

    async def rotate(
        self,
        delta_angle: float,
        goal_handle: ServerGoalHandle,
        speed=0.5,
        update_freq=10.0,
    ):

        if delta_angle == 0:
            return

        rotate_msg = Twist()
        rotate_msg.angular.z = speed if delta_angle > 0 else -speed

        feedback_msg = Rotate.Feedback()

        # Add timeout to prevent infinite loops
        max_rotation_time = 30.0  # seconds
        start_time = time.time()

        # Rotate until target angle is reached or timeout/cancel
        while not self.has_reached_target_angle():
            await async_sleep(self.node, 1 / update_freq)

            # Check for cancellation
            if goal_handle.is_cancel_requested:
                break

            # Check for timeout
            if time.time() - start_time > max_rotation_time:
                self.logger.warn(f"Rotation timeout after {max_rotation_time} seconds")
                break

            # Calculate progress based on angle covered
            if self.current_pose is not None and self.start_angle is not None:
                angle_covered = abs(
                    self.angle_difference(self.current_pose, self.start_angle)
                )
                total_angle = abs(delta_angle)
                progress = (
                    min(1.0, angle_covered / total_angle) if total_angle > 0 else 1.0
                )
                feedback_msg.progress = progress
            else:
                feedback_msg.progress = 0.0

            goal_handle.publish_feedback(feedback_msg)
            self.cmd_vel_publisher.publish(rotate_msg)

        # Stop rotation
        self.cmd_vel_publisher.publish(self.stop_msg)

        # Log final result
        if self.current_pose is not None and self.target_angle is not None:
            final_error = abs(
                self.angle_difference(self.target_angle, self.current_pose)
            )
            self.logger.info(
                f"Rotation completed: final_angle={self.current_pose:.3f}, target={self.target_angle:.3f}, error={final_error:.3f}"
            )


def main(args=None, namespace=""):
    rclpy.init(args=args)
    node = Node("rotate_action", namespace=namespace)
    _ = RotateAction(node)

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
    main(namespace="turtle1")
    # main()
