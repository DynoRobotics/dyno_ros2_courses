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

from geometry_msgs.msg import Twist
from my_first_interfaces.action import Move


class MoveAction:
    def __init__(self, node: Node):
        self.node = node
        self.node._default_callback_group = ReentrantCallbackGroup()
        self.logger = self.node.get_logger()

        self.logger.info("Initializing move action")

        # State
        self.is_processing_goal = False

        # Constants
        self.stop_msg = Twist()
        self.default_speed = 0.5  # m/s

        # Publishers
        self.cmd_vel_publisher = self.node.create_publisher(
            msg_type=Twist, topic="cmd_vel", qos_profile=10
        )

        # Action server
        self.action_server = ActionServer(
            node=node,
            action_type=Move,
            action_name="move",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    async def goal_callback(self, goal_request: Move.Goal) -> GoalResponse:
        if self.is_processing_goal:
            self.logger.warn("Move action is already in progress, rejecting new goal")
            return GoalResponse.REJECT

        # Validate goal parameters
        if goal_request.distance == 0:
            self.logger.warn("Invalid distance: distance cannot be zero")
            return GoalResponse.REJECT

        if abs(goal_request.distance) > 10.0:
            self.logger.warn(
                f"Invalid distance: {goal_request.distance}m exceeds maximum allowed distance of 10.0m"
            )
            return GoalResponse.REJECT

        # Validate speed if use_speed is True
        if goal_request.use_speed:
            if goal_request.speed <= 0:
                self.logger.warn(
                    f"Invalid speed: {goal_request.speed} m/s must be greater than 0"
                )
                return GoalResponse.REJECT
            if goal_request.speed > 2.0:
                self.logger.warn(
                    f"Invalid speed: {goal_request.speed} m/s exceeds maximum allowed speed of 2.0 m/s"
                )
                return GoalResponse.REJECT

        self.logger.info("Accepted new goal")
        return GoalResponse.ACCEPT

    async def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        if not self.is_processing_goal:
            self.logger.warn("Trying to cancel a goal but we not running any")
            return CancelResponse.REJECT
        self.logger.info("Goal cancel request accepted")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle) -> Move.Result:
        start_timestamp = time.time()
        self.is_processing_goal = True
        distance_traveled = 0.0

        try:
            goal: Move.Goal = goal_handle.request
            distance = goal.distance

            # Use speed based on use_speed parameter
            if goal.use_speed and goal.speed > 0:
                speed = goal.speed
            else:
                speed = self.default_speed

            self.logger.info(
                f"Moving {distance} meters at {speed} m/s (use_speed: {goal.use_speed})"
            )
            distance_traveled = await self.move(distance, speed, goal_handle)

        except Exception as e:
            self.logger.error(f"Move action failed: {e}")
            goal_handle.abort()
        else:
            if goal_handle.is_cancel_requested:
                self.logger.info("Move action canceled")
                goal_handle.canceled()
            else:
                self.logger.info("Move action succeeded")
                goal_handle.succeed()

        self.is_processing_goal = False

        res = Move.Result()
        res.total_time_sec = time.time() - start_timestamp
        res.distance_traveled = distance_traveled

        return res

    async def move(
        self,
        distance: float,
        speed: float,
        goal_handle: ServerGoalHandle,
        update_freq=5.0,
    ) -> float:

        if distance == 0:
            return 0.0

        move_msg = Twist()
        # Set linear velocity based on direction (positive = forward, negative = backward)
        move_msg.linear.x = speed if distance > 0 else -speed

        feedback_msg = Move.Feedback()

        start_moving_timestamp = self.node.get_clock().now()
        movement_duration = Duration(seconds=abs(distance) / speed)
        stop_moving_timestamp = start_moving_timestamp + movement_duration

        distance_traveled = 0.0

        while self.node.get_clock().now() < stop_moving_timestamp:
            await async_sleep(self.node, 1 / update_freq)
            if goal_handle.is_cancel_requested:
                break

            # Calculate and assign feedback progress
            current_time = self.node.get_clock().now()
            elapsed_time = current_time - start_moving_timestamp
            total_time = movement_duration

            # Calculate progress as a percentage (0.0 to 1.0)
            progress = min(1.0, elapsed_time.nanoseconds / total_time.nanoseconds)

            # Calculate current distance traveled
            distance_traveled = abs(distance) * progress
            if distance < 0:
                distance_traveled = -distance_traveled

            feedback_msg.progress = progress
            feedback_msg.current_distance = distance_traveled

            goal_handle.publish_feedback(feedback_msg)
            self.cmd_vel_publisher.publish(move_msg)

        self.cmd_vel_publisher.publish(self.stop_msg)
        return distance_traveled


def main(args=None, namespace=""):
    rclpy.init(args=args)
    node = Node("move_action", namespace=namespace)
    _ = MoveAction(node)

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
