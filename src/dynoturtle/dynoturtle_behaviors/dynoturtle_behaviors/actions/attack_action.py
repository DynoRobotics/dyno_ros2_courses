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

from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Bool
from dynoturtle_interfaces.action import Attack
from transform_utils.transforms import yaw_from_quaternion


class AttackAction:
    def __init__(self, node: Node):
        self.node = node
        self.node._default_callback_group = ReentrantCallbackGroup()
        self.logger = self.node.get_logger()

        self.logger.info("Initializing attack action")

        # State
        self.is_processing_goal = False
        self.current_pose = None  # Pose object
        self.target_pose = None  # Pose object
        self.front_bumper_triggered = False

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

        self.front_bumper_subscriber = self.node.create_subscription(
            msg_type=Bool,
            topic="sensors/front_bumper",
            callback=self.front_bumper_callback,
            qos_profile=10,
        )

        # Action server
        self.action_server = ActionServer(
            node=node,
            action_type=Attack,
            action_name="attack",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def pose_callback(self, msg: PoseStamped):
        """Callback to update current pose from pose3d topic"""
        # Store the full Pose object
        self.current_pose = msg.pose

    def front_bumper_callback(self, msg: Bool):
        self.front_bumper_triggered = msg.data

    def target_pose_callback(self, msg: PoseStamped):
        """Callback to update target pose from target robot's pose3d topic"""
        # Store the full Pose object
        self.target_pose = msg.pose

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def angle_difference(target: float, current: float) -> float:
        """Calculate the shortest angular difference between two angles"""
        diff = target - current
        return AttackAction.normalize_angle(diff)

    @staticmethod
    def inverse_transform_point(transform: Pose, world_point: tuple) -> tuple:
        """Convert world space point to local space relative to transform

        Unity equivalent: Transform.InverseTransformPoint()

        Args:
            transform: The reference transform (like Unity Transform)
            world_point: Point in world coordinates (x, y)

        Returns:
            tuple: Point in transform's local space (x, y)
        """
        # Get transform position and rotation
        transform_x = transform.position.x
        transform_y = transform.position.y
        transform_yaw = yaw_from_quaternion(transform.orientation)

        # Translate to transform origin
        relative_x = world_point[0] - transform_x
        relative_y = world_point[1] - transform_y

        # Rotate by negative transform yaw to get local coordinates
        cos_yaw = math.cos(-transform_yaw)
        sin_yaw = math.sin(-transform_yaw)

        local_x = relative_x * cos_yaw - relative_y * sin_yaw
        local_y = relative_x * sin_yaw + relative_y * cos_yaw

        return (local_x, local_y)

    @staticmethod
    def inverse_transform_pose(transform: Pose, world_pose: Pose) -> Pose:
        """Convert world space pose to local space relative to transform

        Unity equivalent: Transform.InverseTransformPoint() + rotation handling

        Args:
            transform: The reference transform
            world_pose: Pose in world coordinates

        Returns:
            Pose: Pose in transform's local space
        """
        # Transform position
        world_point = (world_pose.position.x, world_pose.position.y)
        local_point = AttackAction.inverse_transform_point(transform, world_point)

        # Transform orientation (relative rotation)
        transform_yaw = yaw_from_quaternion(transform.orientation)
        world_yaw = yaw_from_quaternion(world_pose.orientation)
        local_yaw = AttackAction.normalize_angle(world_yaw - transform_yaw)

        # Create local pose
        local_pose = Pose()
        local_pose.position.x = local_point[0]
        local_pose.position.y = local_point[1]
        local_pose.position.z = 0.0

        # Convert local yaw back to quaternion (simplified for 2D)
        local_pose.orientation.x = 0.0
        local_pose.orientation.y = 0.0
        local_pose.orientation.z = math.sin(local_yaw / 2.0)
        local_pose.orientation.w = math.cos(local_yaw / 2.0)

        return local_pose

    @staticmethod
    def magnitude(vector: tuple) -> float:
        """Get vector magnitude (Unity-style)

        Args:
            vector: 2D vector as (x, y) tuple

        Returns:
            float: Vector magnitude
        """
        return math.sqrt(vector[0] * vector[0] + vector[1] * vector[1])

    @staticmethod
    def normalized(vector: tuple) -> tuple:
        """Get normalized unit vector (Unity-style)

        Args:
            vector: 2D vector as (x, y) tuple

        Returns:
            tuple: Normalized vector as (x, y) tuple
        """
        mag = AttackAction.magnitude(vector)
        if mag == 0:
            return (0.0, 0.0)
        return (vector[0] / mag, vector[1] / mag)

    def get_local_target_direction(self) -> tuple:
        """Get target direction in local space (Unity-style)

        Returns:
            tuple: (local_x, local_y) where x=forward/back, y=left/right
        """
        if self.current_pose is None or self.target_pose is None:
            return None

        # Transform target position to local space
        target_point = (self.target_pose.position.x, self.target_pose.position.y)
        local_point = self.inverse_transform_point(self.current_pose, target_point)
        return local_point

    def get_angle_to_target(self) -> float:
        """Get angle to target in local space (Unity-style)

        Returns:
            float: Angle in radians (positive=left, negative=right, 0=straight ahead)
        """
        local_direction = self.get_local_target_direction()
        if local_direction is None:
            return None

        # Calculate angle in local space (0 = straight ahead)
        return math.atan2(local_direction[1], local_direction[0])

    def get_distance_to_target(self) -> float:
        """Calculate distance to target (Unity-style)"""
        local_direction = self.get_local_target_direction()
        if local_direction is None:
            return float("inf")

        return self.magnitude(local_direction)

    def is_target_ahead(self) -> bool:
        """Check if target is in front of robot (Unity-style)"""
        local_direction = self.get_local_target_direction()
        if local_direction is None:
            return False
        return local_direction[0] > 0  # Positive X = ahead

    def is_facing_target(self) -> bool:
        """Check if robot is facing the target within tolerance (Unity-style)"""
        angle_to_target = self.get_angle_to_target()
        if angle_to_target is None:
            return False
        return abs(angle_to_target) <= self.angle_tolerance

    async def goal_callback(self, goal_request: Attack.Goal) -> GoalResponse:
        if self.is_processing_goal:
            self.logger.warn("Attack action is already in progress, rejecting new goal")
            return GoalResponse.REJECT

        self.logger.info("Accepted new goal")
        return GoalResponse.ACCEPT

    async def cancel_callback(self, goal_handle: ServerGoalHandle) -> Attack.Result:
        if not self.is_processing_goal:
            self.logger.warn("Trying to cancel a goal but we not running any")
            return CancelResponse.REJECT
        self.logger.info("Goal cancel request accepted")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle) -> Attack.Result:
        self.is_processing_goal = True

        try:
            goal: Attack.Goal = goal_handle.request
            target_pose_subscriber = self.node.create_subscription(
                msg_type=PoseStamped,
                topic=f"/{goal.robot_name}/pose3d",
                callback=self.target_pose_callback,
                qos_profile=10,
            )

            # Wait for pose data to be available
            while self.current_pose is None:
                await async_sleep(self.node, 0.1)
                if goal_handle.is_cancel_requested:
                    self.is_processing_goal = False
                    goal_handle.canceled()
                    res = Attack.Result()
                    return res

            try:
                await self.attack(goal_handle)
            except Exception as e:
                self.logger.error(f"Attack action failed: {e}")
                goal_handle.abort()
            else:
                if goal_handle.is_cancel_requested:
                    self.logger.info("Attack action canceled")
                    goal_handle.canceled()
                else:
                    self.logger.info("Attack action succeeded")
                    goal_handle.succeed()

        finally:
            self.node.destroy_subscription(target_pose_subscriber)
            self.is_processing_goal = False

        res = Attack.Result()
        return res

    async def attack(
        self,
        goal_handle: ServerGoalHandle,
        linear_speed=0.8,
        angular_speed=0.5,
        update_freq=10.0,
    ):
        """Attack behavior: drive toward target until collision"""

        # Add timeout to prevent infinite loops
        max_attack_time = 30.0  # seconds
        start_time = time.time()

        self.logger.info("Starting attack behavior")

        # Wait for target pose data to be available
        while self.target_pose is None:
            await async_sleep(self.node, 0.1)
            if goal_handle.is_cancel_requested:
                return
            if time.time() - start_time > 5.0:  # 5 second timeout for target data
                self.logger.warn("Timeout waiting for target pose data")
                return

        # Main attack loop (Unity-style)
        while not self.has_hit_target():
            await async_sleep(self.node, 1 / update_freq)

            # Check for cancellation
            if goal_handle.is_cancel_requested:
                break

            # Check for timeout
            if time.time() - start_time > max_attack_time:
                self.logger.warn(f"Attack timeout after {max_attack_time} seconds")
                break

            # Get Unity-style local target direction
            local_direction = self.get_local_target_direction()
            if local_direction is None:
                continue

            # Get angle to target in local space (Unity-style)
            angle_to_target = self.get_angle_to_target()
            if angle_to_target is None:
                continue

            # Create movement command
            twist_msg = Twist()

            # Unity-style control: angle_to_target is directly usable
            # Positive = turn left, Negative = turn right, 0 = straight ahead
            if (
                abs(angle_to_target) <= self.angle_tolerance * 2
            ):  # Allow some tolerance for forward movement
                # Target is roughly ahead - move forward with minor steering
                twist_msg.linear.x = linear_speed
                # Apply proportional steering correction
                twist_msg.angular.z = angular_speed * 0.3 * angle_to_target
            else:
                # Target requires significant turning - turn in place
                twist_msg.linear.x = 0.0
                # Direct steering: positive angle = turn left, negative = turn right
                twist_msg.angular.z = (
                    angular_speed if angle_to_target > 0 else -angular_speed
                )

            # Publish movement command
            self.cmd_vel_publisher.publish(twist_msg)

            # Log progress occasionally (Unity-style info)
            distance = self.get_distance_to_target()
            is_ahead = self.is_target_ahead()
            if int(time.time() * 2) % 10 == 0:  # Log every 5 seconds
                self.logger.info(
                    f"Target: distance={distance:.2f}m, angle={angle_to_target:.2f}rad, "
                    f"local_pos=({local_direction[0]:.2f}, {local_direction[1]:.2f}), "
                    f"ahead={is_ahead}"
                )

        # Stop movement
        self.cmd_vel_publisher.publish(self.stop_msg)

        if self.has_hit_target():
            self.logger.info("Attack successful - target hit!")
        else:
            self.logger.info("Attack stopped")

    def has_hit_target(self):
        return self.front_bumper_triggered


def main(args=None, namespace=""):
    rclpy.init(args=args)
    node = Node("attack_action", namespace=namespace)
    _ = AttackAction(node)

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
