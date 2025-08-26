import rclpy
import time
import sys

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from async_utils.async_primitives import async_sleep


from my_first_interfaces.action import Move


class MoveActionClient(Node):
    def __init__(self, namespace=""):
        super().__init__("move_action_client", namespace=namespace)
        self._default_callback_group = ReentrantCallbackGroup()
        self.logger = self.get_logger()
        self.logger.info("Initializing move action client")

        # Create action client
        self.action_client = ActionClient(self, Move, "move")

        # Wait for action server to be available
        self.logger.info("Waiting for action server...")
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.logger.error("Action server not available after waiting")
            return

        self.logger.info("Action server is available!")

        # Store current goal handle for cancellation
        self.current_goal_handle = None

        self.timer = self.create_timer(0.0, self.demo_sequence)

    def send_goal(self, distance: float, speed: float = 0.5):
        """Send a move goal to the action server."""
        goal_msg = Move.Goal()
        goal_msg.distance = distance
        goal_msg.speed = speed

        direction = "forward" if distance > 0 else "backward"
        self.logger.info(
            f"Sending goal: move {abs(distance)} meters {direction} at {speed} m/s"
        )

        # Send goal and get future
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        # Add callback for when goal is accepted/rejected
        send_goal_future.add_done_callback(self.goal_response_callback)

        return send_goal_future

    def goal_response_callback(self, future):
        """Callback for when goal is accepted or rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info("Goal rejected")
            return

        self.logger.info("Goal accepted")
        self.current_goal_handle = goal_handle

        # Get result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback for when action completes."""
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.logger.info(
                f"Action succeeded! Distance traveled: {result.distance_traveled:.2f} meters, "
                f"Total time: {result.total_time_sec:.2f} seconds"
            )
        elif status == 5:  # CANCELED
            self.logger.info(
                f"Action was canceled. Distance traveled: {result.distance_traveled:.2f} meters, "
                f"Total time: {result.total_time_sec:.2f} seconds"
            )
        elif status == 6:  # ABORTED
            self.logger.info(
                f"Action aborted. Distance traveled: {result.distance_traveled:.2f} meters, "
                f"Total time: {result.total_time_sec:.2f} seconds"
            )
        else:
            self.logger.info(f"Action finished with status: {status}")

        self.current_goal_handle = None

    def feedback_callback(self, feedback_msg):
        """Callback for action feedback."""
        feedback = feedback_msg.feedback
        self.logger.info(
            f"Received feedback: progress = {feedback.progress:.2f}, "
            f"current distance = {feedback.current_distance:.2f} meters"
        )

    def cancel_goal(self):
        """Cancel the current goal if one is active."""
        if self.current_goal_handle is None:
            self.logger.warn("No active goal to cancel")
            return False

        self.logger.info("Canceling current goal...")
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response_callback)
        return True

    def cancel_response_callback(self, future):
        """Callback for cancel response."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.logger.info("Goal successfully canceled")
        else:
            self.logger.info("Goal failed to cancel")

    async def demo_sequence(self):
        """Demonstrate calling and canceling actions."""
        self.timer.cancel()

        self.logger.info("Starting demo sequence...")

        # Send first goal - move 2 meters forward
        self.send_goal(2.0, speed=0.5)

        # Wait a bit then cancel
        await async_sleep(self, 1.0)
        self.cancel_goal()

        sys.exit(0)

        # # Wait for cancellation to complete
        # time.sleep(1.0)

        # # Send another goal - move 1 meter backward
        # self.send_goal(-1.0, speed=0.3)

        # # Let this one complete
        # time.sleep(5.0)

        # # Send final goal - move 1.5 meters forward at higher speed
        # self.send_goal(1.5, speed=0.8)


def main(args=None, namespace=""):
    rclpy.init(args=args)

    action_client = MoveActionClient(namespace=namespace)

    try:
        # Run demo sequence
        # action_client.demo_sequence()

        # Spin to handle callbacks
        executor = SingleThreadedExecutor()
        executor.add_node(action_client)
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        action_client.logger.info("Shutting down...")
    finally:
        action_client.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main(namespace="turtle1")
