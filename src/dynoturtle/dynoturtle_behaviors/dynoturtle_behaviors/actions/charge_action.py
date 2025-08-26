import rclpy
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
from async_utils.async_primitives import async_sleep

from std_msgs.msg import Float32
from dynoturtle_interfaces.action import Charge


class ChargeAction:
    def __init__(self, node: Node):
        self.node = node
        self.node._default_callback_group = ReentrantCallbackGroup()
        self.logger = self.node.get_logger()

        self.logger.info("Initializing charge action")

        # State
        self.is_processing_goal = False
        self.current_battery_level = None
        self.target_battery_level = None

        # Subscribers
        self.battery_subscriber = self.node.create_subscription(
            msg_type=Float32,
            topic="sensors/battery_level",
            callback=self.battery_callback,
            qos_profile=10,
        )

        # Action server
        self.action_server = ActionServer(
            node=node,
            action_type=Charge,
            action_name="charge",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def battery_callback(self, msg: Float32):
        """Callback to update current battery level from sensors/battery_level topic"""
        # Convert from percentage (0-100) to fraction (0.0-1.0)
        self.current_battery_level = msg.data / 100.0

    def has_reached_target_level(self) -> bool:
        """Check if current battery level has reached the target level"""
        if self.current_battery_level is None or self.target_battery_level is None:
            return False

        return self.current_battery_level >= self.target_battery_level

    async def goal_callback(self, goal_request: Charge.Goal) -> GoalResponse:
        if self.is_processing_goal:
            self.logger.warn("Charge action is already in progress, rejecting new goal")
            return GoalResponse.REJECT

        # Validate goal parameters
        target_level = goal_request.target_battery_level
        if target_level <= 0.0 or target_level > 1.0:
            self.logger.warn(
                f"Invalid target battery level: {target_level}. Must be between 0.0 and 1.0"
            )
            return GoalResponse.REJECT

        # If no target specified, default to 0.8 (80%)
        if target_level == 0.0:
            target_level = 0.8

        self.logger.info(
            f"Accepted new charge goal: target battery level = {target_level:.1%}"
        )
        return GoalResponse.ACCEPT

    async def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        if not self.is_processing_goal:
            self.logger.warn("Trying to cancel a goal but we not running any")
            return CancelResponse.REJECT
        self.logger.info("Goal cancel request accepted")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle) -> Charge.Result:
        start_timestamp = time.time()
        self.is_processing_goal = True

        # Wait for battery data to be available
        while self.current_battery_level is None:
            await async_sleep(self.node, 0.1)
            if goal_handle.is_cancel_requested:
                self.is_processing_goal = False
                goal_handle.canceled()
                res = Charge.Result()
                res.total_time_sec = time.time() - start_timestamp
                res.final_battery_level = 0.0
                return res

        try:
            goal: Charge.Goal = goal_handle.request

            # Set target battery level (default to 0.8 if not specified or 0.0)
            self.target_battery_level = (
                goal.target_battery_level if goal.target_battery_level > 0.0 else 0.8
            )

            self.logger.info(
                f"Starting charge wait: current={self.current_battery_level:.1%}, target={self.target_battery_level:.1%}"
            )

            # Check if already at target level
            if self.has_reached_target_level():
                self.logger.info("Battery already at target level")
            else:
                await self.wait_for_charge(goal_handle)

        except Exception as e:
            self.logger.error(f"Charge action failed: {e}")
            goal_handle.abort()
        else:
            if goal_handle.is_cancel_requested:
                self.logger.info("Charge action canceled")
                goal_handle.canceled()
            else:
                self.logger.info("Charge action succeeded")
                goal_handle.succeed()

        self.is_processing_goal = False

        res = Charge.Result()
        res.total_time_sec = time.time() - start_timestamp
        res.final_battery_level = (
            self.current_battery_level
            if self.current_battery_level is not None
            else 0.0
        )

        return res

    async def wait_for_charge(
        self,
        goal_handle: ServerGoalHandle,
        update_freq=2.0,  # Check battery level 2 times per second
    ):
        feedback_msg = Charge.Feedback()

        # Add timeout to prevent infinite loops (max 10 minutes)
        max_charge_time = 600.0  # seconds
        start_time = time.time()

        # Wait until target battery level is reached or timeout/cancel
        while not self.has_reached_target_level():
            await async_sleep(self.node, 1 / update_freq)

            # Check for cancellation
            if goal_handle.is_cancel_requested:
                break

            # Check for timeout
            if time.time() - start_time > max_charge_time:
                self.logger.warn(f"Charge timeout after {max_charge_time} seconds")
                break

            # Calculate progress and publish feedback
            if (
                self.current_battery_level is not None
                and self.target_battery_level is not None
            ):
                # Progress is how close we are to the target
                if self.current_battery_level >= self.target_battery_level:
                    progress = 1.0
                else:
                    # Calculate progress from 0 to target level
                    progress = self.current_battery_level / self.target_battery_level
                    progress = max(0.0, min(1.0, progress))  # Clamp to [0, 1]

                feedback_msg.current_battery_level = self.current_battery_level
                feedback_msg.progress = progress
            else:
                feedback_msg.current_battery_level = 0.0
                feedback_msg.progress = 0.0

            goal_handle.publish_feedback(feedback_msg)

        # Log final result
        if (
            self.current_battery_level is not None
            and self.target_battery_level is not None
        ):
            self.logger.info(
                f"Charge completed: final_level={self.current_battery_level:.1%}, target={self.target_battery_level:.1%}"
            )


def main(args=None, namespace=""):
    rclpy.init(args=args)
    node = Node("charge_action", namespace=namespace)
    _ = ChargeAction(node)

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
