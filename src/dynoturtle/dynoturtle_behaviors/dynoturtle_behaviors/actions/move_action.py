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

from geometry_msgs.msg import Twist, PoseStamped
from dynoturtle_interfaces.action import Move
from transform_utils.transforms import yaw_from_quaternion


class SimplePose:
    """Simple pose class to maintain compatibility with existing code"""

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta


class MoveAction:
    def __init__(self, node: Node):
        self.node = node
        self.node._default_callback_group = ReentrantCallbackGroup()
        self.logger = self.node.get_logger()

        self.logger.info("Initializing move action")

        # State
        self.is_processing_goal = False
        self.current_pose = None
        self.start_pose = None
        self.target_pose = None

        # Constants
        self.stop_msg = Twist()
        self.default_speed = 0.5  # m/s
        self.position_tolerance = 0.2  # meters

        # Progress monitoring constants
        self.progress_check_interval = 2.0  # seconds between progress checks
        self.progress_tolerance_factor = (
            0.3  # minimum fraction of expected progress (30% - tightened)
        )
        self.max_stall_time = (
            1.0  # maximum time allowed without progress before aborting (reduced)
        )

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
            action_type=Move,
            action_name="move",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

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

    def calculate_target_pose(
        self, start_pose: SimplePose, distance: float
    ) -> SimplePose:
        """Calculate target pose based on start pose and distance to move"""
        target_pose = SimplePose()
        target_pose.x = start_pose.x + distance * math.cos(start_pose.theta)
        target_pose.y = start_pose.y + distance * math.sin(start_pose.theta)
        target_pose.theta = start_pose.theta  # Assuming no rotation for move action
        return target_pose

    def has_reached_target(self) -> bool:
        """Check if current pose has reached the target pose within tolerance"""
        if self.current_pose is None or self.target_pose is None:
            return False

        distance_to_target = self.calculate_distance(
            self.current_pose, self.target_pose
        )
        return distance_to_target <= self.position_tolerance

    def calculate_directional_progress(
        self, start_pose: SimplePose, current_pose: SimplePose, target_pose: SimplePose
    ) -> float:
        """
        Calculate progress in the intended direction.
        Returns positive value for progress toward target, negative for movement away from target.
        """
        if start_pose is None or current_pose is None or target_pose is None:
            return 0.0

        # Vector from start to target (intended direction)
        target_vector_x = target_pose.x - start_pose.x
        target_vector_y = target_pose.y - start_pose.y
        target_distance = math.sqrt(target_vector_x**2 + target_vector_y**2)

        if target_distance == 0:
            return 0.0

        # Normalize target vector
        target_unit_x = target_vector_x / target_distance
        target_unit_y = target_vector_y / target_distance

        # Vector from start to current position (actual movement)
        movement_vector_x = current_pose.x - start_pose.x
        movement_vector_y = current_pose.y - start_pose.y

        # Project movement vector onto target direction (dot product)
        directional_progress = (
            movement_vector_x * target_unit_x + movement_vector_y * target_unit_y
        )

        return directional_progress

    def check_progress_health(
        self,
        distance_traveled: float,
        target_distance: float,
        elapsed_time: float,
        speed: float,
    ) -> bool:
        """
        Check if the action is making reasonable progress toward the goal.
        Returns True if progress is healthy, False if action should be aborted.
        """
        if elapsed_time <= 0:
            return True  # Too early to judge

        # Calculate expected distance based on time and actual commanded speed
        progress_ratio = (
            abs(distance_traveled) / abs(target_distance) if target_distance != 0 else 0
        )
        expected_time = abs(target_distance) / speed
        time_ratio = elapsed_time / expected_time

        # If we're taking significantly longer than expected and haven't made proportional progress
        if time_ratio > 2.0 and progress_ratio < 0.3:
            self.logger.warn(
                f"Poor progress detected: {progress_ratio:.1%} complete in {time_ratio:.1f}x expected time "
                f"(speed: {speed:.2f} m/s, expected time: {expected_time:.1f}s)"
            )
            return False

        return True

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

            # Wait for initial pose and validate it's available
            pose_timeout_iterations = 50  # 50 * 0.1s = 5 seconds timeout
            pose_wait_count = 0
            while self.current_pose is None:
                if pose_wait_count >= pose_timeout_iterations:
                    self.logger.error("Failed to receive pose data within timeout")
                    raise Exception(
                        "Pose data not available - cannot execute move action"
                    )

                await async_sleep(self.node, 0.1)
                pose_wait_count += 1

                if goal_handle.is_cancel_requested:
                    raise Exception("Action canceled while waiting for pose")

            # Set start pose and calculate target pose
            self.start_pose = SimplePose()
            self.start_pose.x = self.current_pose.x
            self.start_pose.y = self.current_pose.y
            self.start_pose.theta = self.current_pose.theta

            self.target_pose = self.calculate_target_pose(self.start_pose, distance)

            self.logger.info(
                f"Moving {distance} meters at {speed} m/s (use_speed: {goal.use_speed})"
            )
            self.logger.info(
                f"Start pose: ({self.start_pose.x:.2f}, {self.start_pose.y:.2f}, {self.start_pose.theta:.2f})"
            )
            self.logger.info(
                f"Target pose: ({self.target_pose.x:.2f}, {self.target_pose.y:.2f}, {self.target_pose.theta:.2f})"
            )

            distance_traveled = await self.move(distance, speed, goal_handle)

            # Final validation: Check if we actually reached the target pose
            if not goal_handle.is_cancel_requested:
                if not self.has_reached_target():
                    distance_to_target = self.calculate_distance(
                        self.current_pose, self.target_pose
                    )
                    self.logger.error(
                        f"Move action failed to reach target pose. Distance to target: {distance_to_target:.3f}m (tolerance: {self.position_tolerance}m)"
                    )
                    raise Exception("Failed to reach target pose within tolerance")

        except Exception as e:
            self.logger.error(f"Move action failed: {e}")
            goal_handle.abort()
        else:
            if goal_handle.is_cancel_requested:
                self.logger.info("Move action canceled")
                goal_handle.canceled()
            else:
                self.logger.info("Move action succeeded - target pose reached!")
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

        # Progress monitoring variables
        last_progress_check_time = start_moving_timestamp
        last_progress_distance = 0.0
        stall_start_time = None

        while self.node.get_clock().now() < stop_moving_timestamp:
            await async_sleep(self.node, 1 / update_freq)
            if goal_handle.is_cancel_requested:
                break

            # Check if we've reached the target pose
            if self.has_reached_target():
                self.logger.info("Target pose reached during movement!")
                break

            # Calculate and assign feedback progress
            current_time = self.node.get_clock().now()
            elapsed_time = current_time - start_moving_timestamp
            total_time = movement_duration

            # Calculate progress as a percentage (0.0 to 1.0)
            progress = min(1.0, elapsed_time.nanoseconds / total_time.nanoseconds)

            # Calculate current distance traveled from pose if available
            if self.current_pose is not None and self.start_pose is not None:
                actual_distance = self.calculate_distance(
                    self.start_pose, self.current_pose
                )
                if distance < 0:
                    distance_traveled = -actual_distance
                else:
                    distance_traveled = actual_distance
            else:
                # Fallback to time-based calculation
                distance_traveled = abs(distance) * progress
                if distance < 0:
                    distance_traveled = -distance_traveled

            # Progress monitoring - check if we're making sufficient progress (only if pose data available)
            if (
                self.current_pose is not None
                and self.start_pose is not None
                and self.target_pose is not None
            ):
                time_since_last_check = (
                    current_time - last_progress_check_time
                ).nanoseconds / 1e9

                if time_since_last_check >= self.progress_check_interval:
                    # Calculate directional progress (positive = toward target, negative = away from target)
                    current_directional_progress = self.calculate_directional_progress(
                        self.start_pose, self.current_pose, self.target_pose
                    )

                    # Calculate speed-scaled minimum progress requirement
                    expected_progress = (
                        speed * time_since_last_check * self.progress_tolerance_factor
                    )

                    # Calculate actual progress since last check (use consistent measurement)
                    actual_progress_delta = abs(current_directional_progress) - abs(
                        last_progress_distance
                    )

                    # Check directional correctness
                    is_moving_correctly = (
                        distance > 0 and current_directional_progress >= 0
                    ) or (distance < 0 and current_directional_progress <= 0)

                    # Debug logging
                    self.logger.info(
                        f"Progress check: actual={actual_progress_delta:.3f}m, expected={expected_progress:.3f}m, "
                        f"directional={current_directional_progress:.3f}m, correct_dir={is_moving_correctly}"
                    )

                    # Check if making sufficient progress AND moving in correct direction
                    if (
                        actual_progress_delta < expected_progress
                        or not is_moving_correctly
                    ):
                        # Not making enough progress or moving in wrong direction
                        if stall_start_time is None:
                            stall_start_time = current_time
                            direction_msg = (
                                "wrong direction"
                                if not is_moving_correctly
                                else "insufficient progress"
                            )
                            self.logger.warn(
                                f"Progress stall detected ({direction_msg}): moved {actual_progress_delta:.3f}m in {time_since_last_check:.1f}s "
                                f"(expected: {expected_progress:.3f}m at {speed:.2f} m/s). "
                                f"Directional progress: {current_directional_progress:.3f}m (should be {'positive' if distance > 0 else 'negative'})"
                            )
                        else:
                            stall_duration = (
                                current_time - stall_start_time
                            ).nanoseconds / 1e9
                            if stall_duration >= self.max_stall_time:
                                direction_msg = (
                                    "moving in wrong direction"
                                    if not is_moving_correctly
                                    else "insufficient progress"
                                )
                                self.logger.error(
                                    f"Aborting move action: {direction_msg} for {stall_duration:.1f}s "
                                    f"(max allowed: {self.max_stall_time}s). "
                                    f"Distance traveled: {abs(distance_traveled):.3f}m of {abs(distance):.3f}m target. "
                                    f"Speed: {speed:.2f} m/s, Expected progress: {expected_progress:.3f}m, Actual: {actual_progress_delta:.3f}m. "
                                    f"Directional progress: {current_directional_progress:.3f}m (should be {'positive' if distance > 0 else 'negative'})"
                                )
                                raise Exception(
                                    f"Move action aborted due to {direction_msg}: "
                                    f"stalled for {stall_duration:.1f}s (max: {self.max_stall_time}s)"
                                )
                    else:
                        # Making good progress in correct direction, reset stall timer
                        if stall_start_time is not None:
                            self.logger.info(
                                f"Progress resumed: moved {actual_progress_delta:.3f}m in {time_since_last_check:.1f}s "
                                f"(expected: {expected_progress:.3f}m). Directional progress: {current_directional_progress:.3f}m"
                            )
                            stall_start_time = None

                    # Update progress tracking variables (use absolute directional progress for consistent tracking)
                    last_progress_check_time = current_time
                    last_progress_distance = abs(current_directional_progress)
            else:
                # No pose data available - log warning but don't abort
                if stall_start_time is None:
                    self.logger.warn(
                        "Progress monitoring disabled: pose data not available"
                    )

            feedback_msg.progress = progress
            feedback_msg.current_distance = distance_traveled

            goal_handle.publish_feedback(feedback_msg)
            self.cmd_vel_publisher.publish(move_msg)

        self.cmd_vel_publisher.publish(self.stop_msg)

        # Final distance calculation from pose
        if self.current_pose is not None and self.start_pose is not None:
            final_distance = self.calculate_distance(self.start_pose, self.current_pose)
            if distance < 0:
                distance_traveled = -final_distance
            else:
                distance_traveled = final_distance

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
