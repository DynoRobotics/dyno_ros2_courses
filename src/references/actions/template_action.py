import rclpy
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import (
    ExternalShutdownException,
    SingleThreadedExecutor,
)
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from async_utils.async_primitives import async_sleep

from example_interfaces.action import Fibonacci


class FibonacciAction:
    """
    Template Fibonacci Action Server

    This template demonstrates the standard ROS2 action server pattern
    by implementing a Fibonacci sequence generator. It follows the same
    structure as other action servers in this project and can be used
    as a template for creating new action servers.

    Key Features:
    - Async execution with cancellation support
    - Progress feedback during computation
    - Input validation and error handling
    - Configurable computation parameters
    - Proper state management and cleanup
    """

    def __init__(self, node: Node):
        self.node = node
        self.node._default_callback_group = ReentrantCallbackGroup()
        self.logger = self.node.get_logger()

        self.logger.info("Initializing Fibonacci action server")

        # State management
        self.is_processing_goal = False

        # Configuration parameters
        self.computation_delay = 0.1  # seconds between Fibonacci calculations
        self.max_order = 100  # maximum Fibonacci sequence length for safety
        self.feedback_frequency = 5  # provide feedback every N calculations

        # Action server
        self.action_server = ActionServer(
            node=node,
            action_type=Fibonacci,
            action_name="fibonacci",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    async def goal_callback(self, goal_request: Fibonacci.Goal) -> GoalResponse:
        """
        Validate and accept/reject incoming goal requests

        Args:
            goal_request: The Fibonacci goal containing the order (sequence length)

        Returns:
            GoalResponse: ACCEPT or REJECT
        """
        if self.is_processing_goal:
            self.logger.warn(
                "Fibonacci action is already in progress, rejecting new goal"
            )
            return GoalResponse.REJECT

        # Validate goal parameters
        if goal_request.order < 0:
            self.logger.warn(
                f"Invalid order: {goal_request.order} (must be non-negative)"
            )
            return GoalResponse.REJECT

        if goal_request.order > self.max_order:
            self.logger.warn(
                f"Invalid order: {goal_request.order} exceeds maximum allowed ({self.max_order})"
            )
            return GoalResponse.REJECT

        self.logger.info(f"Accepted Fibonacci goal with order: {goal_request.order}")
        return GoalResponse.ACCEPT

    async def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """
        Handle cancellation requests

        Args:
            goal_handle: The goal handle to cancel

        Returns:
            CancelResponse: ACCEPT or REJECT
        """
        if not self.is_processing_goal:
            self.logger.warn("Trying to cancel a goal but no goal is currently running")
            return CancelResponse.REJECT

        self.logger.info("Goal cancellation request accepted")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        """
        Main execution callback - generates the Fibonacci sequence

        Args:
            goal_handle: The goal handle containing the request and providing feedback/result methods

        Returns:
            Fibonacci.Result: The complete Fibonacci sequence or partial results if cancelled
        """
        start_timestamp = time.time()
        self.is_processing_goal = True

        try:
            goal: Fibonacci.Goal = goal_handle.request
            self.logger.info(f"Starting Fibonacci computation for order: {goal.order}")

            # Generate the Fibonacci sequence
            sequence = await self.generate_fibonacci_sequence(goal.order, goal_handle)

        except Exception as e:
            self.logger.error(f"Fibonacci action failed: {e}")
            goal_handle.abort()
            sequence = []
        else:
            if goal_handle.is_cancel_requested:
                self.logger.info("Fibonacci action was cancelled")
                goal_handle.canceled()
            else:
                self.logger.info("Fibonacci action completed successfully")
                goal_handle.succeed()

        # Clean up state
        self.is_processing_goal = False

        # Prepare result
        result = Fibonacci.Result()
        result.sequence = sequence

        total_time = time.time() - start_timestamp
        self.logger.info(f"Fibonacci computation completed in {total_time:.2f} seconds")

        return result

    async def generate_fibonacci_sequence(
        self, order: int, goal_handle: ServerGoalHandle
    ) -> list:
        """
        Generate Fibonacci sequence with progress feedback and cancellation support

        Args:
            order: Number of Fibonacci numbers to generate
            goal_handle: Goal handle for feedback and cancellation checking

        Returns:
            list: The Fibonacci sequence up to the requested order
        """
        sequence = []
        feedback_msg = Fibonacci.Feedback()

        # Handle edge cases
        if order == 0:
            return sequence

        # Generate Fibonacci sequence iteratively
        a, b = 0, 1

        for i in range(order):
            # Check for cancellation before each calculation
            if goal_handle.is_cancel_requested:
                self.logger.info(f"Cancellation requested at step {i+1}/{order}")
                break

            # Add next Fibonacci number
            if i == 0:
                sequence.append(a)
            elif i == 1:
                sequence.append(b)
            else:
                next_fib = a + b
                sequence.append(next_fib)
                a, b = b, next_fib

            # Provide feedback periodically
            if (i + 1) % self.feedback_frequency == 0 or i == order - 1:
                feedback_msg.sequence = sequence.copy()
                goal_handle.publish_feedback(feedback_msg)
                self.logger.info(f"Progress: {i+1}/{order} numbers computed")

            # Add delay to simulate computation time and allow cancellation
            if i < order - 1:  # Don't delay after the last calculation
                await async_sleep(self.node, self.computation_delay)

        return sequence

    def set_computation_delay(self, delay: float):
        """
        Set the delay between Fibonacci calculations

        Args:
            delay: Delay in seconds (must be non-negative)
        """
        if delay >= 0:
            self.computation_delay = delay
            self.logger.info(f"Computation delay set to {delay} seconds")
        else:
            self.logger.warn(f"Invalid delay: {delay} (must be non-negative)")

    def set_max_order(self, max_order: int):
        """
        Set the maximum allowed Fibonacci order

        Args:
            max_order: Maximum order (must be positive)
        """
        if max_order > 0:
            self.max_order = max_order
            self.logger.info(f"Maximum order set to {max_order}")
        else:
            self.logger.warn(f"Invalid max_order: {max_order} (must be positive)")


def main(args=None, namespace=""):
    """
    Main entry point for the Fibonacci action server

    Args:
        args: Command line arguments
        namespace: ROS namespace for the node
    """
    rclpy.init(args=args)
    node = Node("fibonacci_action", namespace=namespace)

    # Create the Fibonacci action server
    fibonacci_action = FibonacciAction(node)

    # Optional: Configure parameters (could be loaded from ROS parameters)
    fibonacci_action.set_computation_delay(0.1)  # 100ms delay between calculations
    fibonacci_action.set_max_order(50)  # Allow up to 50 Fibonacci numbers

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
