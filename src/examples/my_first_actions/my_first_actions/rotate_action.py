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

from geometry_msgs.msg import Twist
from my_first_interfaces.action import Rotate


class RotateAction:
    def __init__(self, node: Node):
        self.node = node
        self.node._default_callback_group = ReentrantCallbackGroup()
        self.logger = self.node.get_logger()

        self.logger.info("Initializing rotate action")

        # State
        self.is_processing_goal = False

        # Constants
        self.stop_msg = Twist()

        # Publishers
        self.cmd_vel_publisher = self.node.create_publisher(
            msg_type=Twist, topic="cmd_vel", qos_profile=10
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
        try:
            goal: Rotate.Goal = goal_handle.request
            delta_angle_radians = (
                goal.delta_angle if goal.radians else goal.delta_angle * math.pi / 180
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
        update_freq=5.0,
    ):

        if delta_angle == 0:
            return

        rotate_msg = Twist()
        rotate_msg.angular.z = speed if delta_angle > 0 else -speed

        feedback_msg = Rotate.Feedback()

        start_rotating_timestamp = self.node.get_clock().now()
        rotation_duration = Duration(seconds=math.fabs(delta_angle / speed))
        stop_rotating_timestamp = start_rotating_timestamp + rotation_duration
        while self.node.get_clock().now() < stop_rotating_timestamp:
            await async_sleep(self.node, 1 / update_freq)
            if goal_handle.is_cancel_requested:
                break

            # Calculate and assign feedback progress
            current_time = self.node.get_clock().now()
            elapsed_time = current_time - start_rotating_timestamp
            total_time = rotation_duration

            # Calculate progress as a percentage (0.0 to 1.0)
            progress = min(1.0, elapsed_time.nanoseconds / total_time.nanoseconds)
            feedback_msg.progress = progress

            goal_handle.publish_feedback(feedback_msg)
            self.cmd_vel_publisher.publish(rotate_msg)
        self.cmd_vel_publisher.publish(self.stop_msg)


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
