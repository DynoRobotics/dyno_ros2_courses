import rclpy
import math
import time

from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import (
    ExternalShutdownException,
    SingleThreadedExecutor,
    MultiThreadedExecutor,
)
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from async_utils.async_primitives import async_sleep

from geometry_msgs.msg import PoseStamped
from dynoturtle_interfaces.action import MoveTo, Move, Rotate
from transform_utils.transforms import yaw_from_quaternion


class SimplePose:
    """Simple pose class to maintain compatibility with existing code"""

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta


class MoveToAction:
    def __init__(self, node: Node):
        self.node = node
        self.node._default_callback_group = ReentrantCallbackGroup()
        self.logger = self.node.get_logger()

        self.logger.info("Initializing move_to action")

        # State
        self.is_processing_goal = False
        self.current_pose = None
        self.start_pose = None
        self.target_pose = None

        # Constants
        self.default_speed = 0.5  # m/s
        self.position_tolerance = 0.1  # meters
        self.angle_tolerance = 0.1  # radians (~6 degrees)

        # Subscribers
        self.pose_subscriber = self.node.create_subscription(
            msg_type=PoseStamped,
            topic="pose3d",
            callback=self.pose_callback,
            qos_profile=10,
        )

        # Action clients for existing Move and Rotate actions
        self.move_client = ActionClient(self.node, Move, "move")
        self.rotate_client = ActionClient(self.node, Rotate, "rotate")

        # Action server
        self.action_server = ActionServer(
            node=node,
            action_type=MoveTo,
            action_name="move_to",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Wait for action servers to be available
        self.logger.info("Waiting for move and rotate action servers...")
        self.move_client.wait_for_server(timeout_sec=10.0)
        self.rotate_client.wait_for_server(timeout_sec=10.0)
        self.logger.info("Action servers are available")

    def pose_callback(self, msg: PoseStamped):
        """Callback to update current pose from pose3d topic"""
        # Convert PoseStamped to SimplePose
        yaw = yaw_from_quaternion(msg.pose.orientation)
        self.current_pose = SimplePose(
            x=msg.pose.position.x, y=msg.pose.position.y, theta=yaw
        )

    def calculate_distance(self, pose1: SimplePose, pose2: SimplePose) -> float:
        """Calculate Euclidean distance between two poses"""
        if pose1 is None or pose2 is None:
            return float("inf")

        dx = pose2.x - pose1.x
        dy = pose2.y - pose1.y
        return math.sqrt(dx * dx + dy * dy)

    def calculate_angle_to_target(
        self, current_pose: SimplePose, target_pose: SimplePose
    ) -> float:
        """Calculate the angle from current pose to target pose"""
        if current_pose is None or target_pose is None:
            return 0.0

        dx = target_pose.x - current_pose.x
        dy = target_pose.y - current_pose.y
        return math.atan2(dy, dx)

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

    def has_reached_target(self) -> bool:
        """Check if current pose has reached the target pose within tolerance"""
        if self.current_pose is None or self.target_pose is None:
            return False

        distance_to_target = self.calculate_distance(
            self.current_pose, self.target_pose
        )
        return distance_to_target <= self.position_tolerance

    async def goal_callback(self, goal_request: MoveTo.Goal) -> GoalResponse:
        if self.is_processing_goal:
            self.logger.warn("MoveTo action is already in progress, rejecting new goal")
            return GoalResponse.REJECT

        # Validate goal parameters
        target_x = goal_request.pose.pose.position.x
        target_y = goal_request.pose.pose.position.y

        # Check for reasonable position limits (assuming 0-11 range for turtlesim)
        if target_x < 0 or target_x > 11 or target_y < 0 or target_y > 11:
            self.logger.warn(
                f"Invalid target position: ({target_x:.2f}, {target_y:.2f}) is outside valid range [0, 11]"
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

    async def execute_callback(self, goal_handle: ServerGoalHandle) -> MoveTo.Result:
        start_timestamp = time.time()
        self.is_processing_goal = True
        distance_traveled = 0.0

        try:
            goal: MoveTo.Goal = goal_handle.request

            # Use speed based on use_speed parameter
            if goal.use_speed and goal.speed > 0:
                speed = goal.speed
            else:
                speed = self.default_speed

            # Wait for initial pose and validate it's available
            pose_timeout_iterations = 50  # 50 * 0.1s = 5 seconds timeout
            pose_wait_count = 0
            while self.current_pose is None:
                if pose_wait_count >= pose_timeout_iterations:
                    self.logger.error("Failed to receive pose data within timeout")
                    raise Exception(
                        "Pose data not available - cannot execute move_to action"
                    )

                await async_sleep(self.node, 0.1)
                pose_wait_count += 1

                if goal_handle.is_cancel_requested:
                    raise Exception("Action canceled while waiting for pose")

            # Set start pose and target pose
            self.start_pose = SimplePose()
            self.start_pose.x = self.current_pose.x
            self.start_pose.y = self.current_pose.y
            self.start_pose.theta = self.current_pose.theta

            # Extract target pose from goal
            target_yaw = yaw_from_quaternion(goal.pose.pose.orientation)
            self.target_pose = SimplePose(
                x=goal.pose.pose.position.x,
                y=goal.pose.pose.position.y,
                theta=target_yaw,
            )

            total_distance = self.calculate_distance(self.start_pose, self.target_pose)

            self.logger.info(
                f"Moving to target position ({self.target_pose.x:.2f}, {self.target_pose.y:.2f}) at {speed} m/s"
            )
            self.logger.info(
                f"Start pose: ({self.start_pose.x:.2f}, {self.start_pose.y:.2f}, {self.start_pose.theta:.2f})"
            )
            self.logger.info(
                f"Target pose: ({self.target_pose.x:.2f}, {self.target_pose.y:.2f}, {self.target_pose.theta:.2f})"
            )
            self.logger.info(f"Total distance: {total_distance:.2f}m")

            distance_traveled = await self.navigate_to_target(speed, goal_handle)

            # Final validation: Check if we actually reached the target pose
            if not goal_handle.is_cancel_requested:
                if not self.has_reached_target():
                    distance_to_target = self.calculate_distance(
                        self.current_pose, self.target_pose
                    )

                    # Retry once if the first attempt was too far off
                    retry_threshold = self.position_tolerance
                    if distance_to_target > retry_threshold:
                        self.logger.warn(
                            f"First attempt too far off target (distance: {distance_to_target:.3f}m > threshold: {retry_threshold:.3f}m). Retrying once..."
                        )

                        # Retry navigation once
                        retry_distance = await self.navigate_to_target(
                            speed, goal_handle
                        )
                        distance_traveled += retry_distance

                        # Check again after retry
                        if not self.has_reached_target():
                            final_distance_to_target = self.calculate_distance(
                                self.current_pose, self.target_pose
                            )
                            self.logger.error(
                                f"MoveTo action failed to reach target pose after retry. Distance to target: {final_distance_to_target:.3f}m (tolerance: {self.position_tolerance}m)"
                            )
                            raise Exception(
                                "Failed to reach target pose within tolerance after retry"
                            )
                        else:
                            self.logger.info("Retry successful - target pose reached!")
                    else:
                        self.logger.error(
                            f"MoveTo action failed to reach target pose. Distance to target: {distance_to_target:.3f}m (tolerance: {self.position_tolerance}m)"
                        )
                        raise Exception("Failed to reach target pose within tolerance")

        except Exception as e:
            self.logger.error(f"MoveTo action failed: {e}")
            goal_handle.abort()
        else:
            if goal_handle.is_cancel_requested:
                self.logger.info("MoveTo action canceled")
                goal_handle.canceled()
            else:
                self.logger.info("MoveTo action succeeded - target pose reached!")
                goal_handle.succeed()

        self.is_processing_goal = False

        res = MoveTo.Result()
        res.total_time_sec = time.time() - start_timestamp
        res.distance_traveled = distance_traveled

        return res

    async def navigate_to_target(
        self,
        speed: float,
        goal_handle: ServerGoalHandle,
    ) -> float:
        """Navigate to target pose using existing Move and Rotate actions"""

        feedback_msg = MoveTo.Feedback()
        total_distance = self.calculate_distance(self.start_pose, self.target_pose)

        # Phase 1: Rotate to face target
        self.logger.info("Phase 1: Rotating to face target")

        # Calculate angle to target
        target_angle = self.calculate_angle_to_target(
            self.current_pose, self.target_pose
        )
        angle_diff = self.angle_difference(target_angle, self.current_pose.theta)

        if abs(angle_diff) > self.angle_tolerance:
            # Create rotate goal
            rotate_goal = Rotate.Goal()
            rotate_goal.delta_angle = angle_diff
            rotate_goal.radians = True
            rotate_goal.use_speed = False  # Use default rotation speed

            # Send rotate goal and wait for acceptance
            self.logger.info(f"Sending rotation goal: {angle_diff:.3f} radians")
            rotate_future = self.rotate_client.send_goal_async(rotate_goal)
            rotate_goal_handle = await rotate_future

            if not rotate_goal_handle.accepted:
                raise Exception("Rotate goal was rejected")

            # Wait for rotation to complete with proper synchronization
            self.logger.info("Waiting for rotation to complete...")
            rotate_result = await rotate_goal_handle.get_result_async()

            if rotate_result.status != 4:  # SUCCEEDED
                raise Exception(f"Rotation failed with status: {rotate_result.status}")

            self.logger.info("Phase 1 completed: Rotation finished successfully")
        else:
            self.logger.info("Phase 1 skipped: Already facing target within tolerance")

        # Phase 2: Move forward to target with chunking and course correction
        self.logger.info("Phase 2: Moving to target position")

        movement_start_pose = SimplePose(
            x=self.current_pose.x, y=self.current_pose.y, theta=self.current_pose.theta
        )

        # Constants for chunking and iteration
        max_move_distance = 9.5  # Slightly less than 10.0m limit for safety
        max_iterations = 5
        iteration = 0
        total_distance_moved = 0.0

        while iteration < max_iterations and not self.has_reached_target():
            if goal_handle.is_cancel_requested:
                break

            # Recalculate distance and angle to target
            distance_to_move = self.calculate_distance(
                self.current_pose, self.target_pose
            )

            if distance_to_move <= self.position_tolerance:
                self.logger.info("Target reached within tolerance")
                break

            # Check if we need to rotate again (course correction)
            target_angle = self.calculate_angle_to_target(
                self.current_pose, self.target_pose
            )
            angle_diff = self.angle_difference(target_angle, self.current_pose.theta)

            if abs(angle_diff) > self.angle_tolerance:
                self.logger.info(
                    f"Course correction needed: rotating {angle_diff:.3f} radians"
                )

                # Create rotate goal for course correction
                rotate_goal = Rotate.Goal()
                rotate_goal.delta_angle = angle_diff
                rotate_goal.radians = True
                rotate_goal.use_speed = False

                # Send rotate goal and wait for completion
                rotate_future = self.rotate_client.send_goal_async(rotate_goal)
                rotate_goal_handle = await rotate_future

                if rotate_goal_handle.accepted:
                    rotate_result = await rotate_goal_handle.get_result_async()
                    if rotate_result.status == 4:  # SUCCEEDED
                        self.logger.info("Course correction completed")
                    else:
                        self.logger.warn(
                            f"Course correction failed with status: {rotate_result.status}"
                        )
                else:
                    self.logger.warn("Course correction rotate goal rejected")

            # Recalculate distance after potential course correction
            distance_to_move = self.calculate_distance(
                self.current_pose, self.target_pose
            )

            if distance_to_move > self.position_tolerance:
                # Implement chunking for distances > max_move_distance
                chunk_distance = min(distance_to_move, max_move_distance)

                self.logger.info(
                    f"Moving {chunk_distance:.3f}m (of {distance_to_move:.3f}m remaining) on iteration {iteration + 1}"
                )

                # Create move goal with chunked distance
                move_goal = Move.Goal()
                move_goal.distance = chunk_distance
                move_goal.speed = speed
                move_goal.use_speed = True

                # Send move goal and wait for acceptance
                move_future = self.move_client.send_goal_async(move_goal)
                move_goal_handle = await move_future

                if not move_goal_handle.accepted:
                    self.logger.warn(f"Move goal rejected on iteration {iteration + 1}")
                    break

                # Wait for movement to complete with proper synchronization
                move_result = await move_goal_handle.get_result_async()

                if move_result.status == 4:  # SUCCEEDED
                    moved_distance = move_result.result.distance_traveled
                    total_distance_moved += abs(moved_distance)
                    self.logger.info(
                        f"Movement iteration {iteration + 1} completed: moved {moved_distance:.3f}m"
                    )
                else:
                    self.logger.warn(
                        f"Movement iteration {iteration + 1} failed with status: {move_result.status}"
                    )
                    # Continue to next iteration instead of failing immediately

                # Update feedback
                if total_distance > 0:
                    current_distance = self.calculate_distance(
                        self.start_pose, self.current_pose
                    )
                    movement_progress = min(1.0, current_distance / total_distance)
                    feedback_msg.progress = 0.25 + (0.75 * movement_progress)
                    feedback_msg.current_distance = current_distance
                    goal_handle.publish_feedback(feedback_msg)

            iteration += 1

        # Calculate final distance traveled
        final_distance = self.calculate_distance(movement_start_pose, self.current_pose)

        if self.has_reached_target():
            self.logger.info("Target reached successfully")
            return final_distance
        else:
            distance_to_target = self.calculate_distance(
                self.current_pose, self.target_pose
            )
            self.logger.warn(
                f"Target not reached after {max_iterations} iterations. Distance remaining: {distance_to_target:.3f}m"
            )
            return final_distance


def main(args=None, namespace=""):
    rclpy.init(args=args)
    node = Node("move_to_action", namespace=namespace)
    _ = MoveToAction(node)

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
