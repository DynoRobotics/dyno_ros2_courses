#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray
from shapely.geometry import Point
from transform_utils.transforms import yaw_from_quaternion
import math
import time


class BumperSimulator:
    """
    Bumper collision simulator that uses Shapely to detect collisions between turtles
    and with the edges of the turtlesim environment.
    """

    def __init__(self, node: Node, default_num_turtles=3):
        """Initialize the BumperSimulator.

        Args:
            node: ROS2 node instance to use for communication
            default_num_turtles: Default number of turtles to simulate
        """
        self.node = node
        self.logger = self.node.get_logger()

        # Declare parameters
        self.node.declare_parameter("num_turtles", default_num_turtles)
        self.node.declare_parameter("turtle_radius", 0.3)
        self.node.declare_parameter("boundary_margin", 0.5)
        self.node.declare_parameter("update_rate", 10.0)

        # Get parameters
        self.num_turtles = (
            self.node.get_parameter("num_turtles").get_parameter_value().integer_value
        )
        self.turtle_radius = (
            self.node.get_parameter("turtle_radius").get_parameter_value().double_value
        )
        self.boundary_margin = (
            self.node.get_parameter("boundary_margin")
            .get_parameter_value()
            .double_value
        )
        update_rate = (
            self.node.get_parameter("update_rate").get_parameter_value().double_value
        )

        # Turtlesim boundaries (0-11 in both x and y)
        self.min_x = 0.0 + self.boundary_margin
        self.max_x = 11.0 - self.boundary_margin
        self.min_y = 0.0 + self.boundary_margin
        self.max_y = 11.0 - self.boundary_margin

        # Charging pad location (center of the area)
        self.charging_pad_x = 5.5  # Center of 0-11 range
        self.charging_pad_y = 5.5  # Center of 0-11 range
        self.charging_pad_radius = 1.0  # Radius of charging area

        # Store turtle poses
        self.turtle_poses = {}

        # Create subscribers for each turtle's pose3d
        self.pose_subscribers = []
        for i in range(1, self.num_turtles + 1):
            turtle_name = f"turtle{i}"
            topic_name = f"/{turtle_name}/pose3d"

            subscriber = self.node.create_subscription(
                PoseStamped,
                topic_name,
                lambda msg, name=turtle_name: self.pose_callback(msg, name),
                10,
            )
            self.pose_subscribers.append(subscriber)

        # Create publishers for each turtle's front bumper
        self.bumper_publishers = {}
        for i in range(1, self.num_turtles + 1):
            turtle_name = f"turtle{i}"
            topic_name = f"/{turtle_name}/sensors/front_bumper"

            publisher = self.node.create_publisher(Bool, topic_name, 10)
            self.bumper_publishers[turtle_name] = publisher

        # Create publisher for bumper visualization
        self.marker_publisher = self.node.create_publisher(
            MarkerArray, "bumper_visualization", 10
        )

        # Store bumper collision states for visualization
        self.bumper_states = {}
        for i in range(1, self.num_turtles + 1):
            turtle_name = f"turtle{i}"
            self.bumper_states[turtle_name] = False

        # Create timer for collision detection
        self.timer = self.node.create_timer(
            1.0 / update_rate, self.collision_detection_callback
        )

        self.logger.info(f"BumperSimulator initialized for {self.num_turtles} turtles")
        self.logger.info(f"Turtle radius: {self.turtle_radius}")
        self.logger.info(f"Boundary margins: {self.boundary_margin}")
        self.logger.info(f"Update rate: {update_rate} Hz")

    def pose_callback(self, msg: PoseStamped, turtle_name: str):
        """Callback for turtle pose updates.

        Args:
            msg: PoseStamped message with turtle pose
            turtle_name: Name of the turtle (e.g., "turtle1")
        """
        self.turtle_poses[turtle_name] = msg

    def get_turtle_heading(self, pose_stamped: PoseStamped) -> float:
        """Extract heading angle from quaternion in PoseStamped message.

        Args:
            pose_stamped: PoseStamped message

        Returns:
            float: Heading angle in radians
        """
        # Use transform_utils to convert quaternion to yaw angle
        return yaw_from_quaternion(pose_stamped.pose.orientation)

    def check_boundary_collision(self, x: float, y: float) -> bool:
        """Check if turtle position collides with turtlesim boundaries.

        Args:
            x: Turtle x position
            y: Turtle y position

        Returns:
            bool: True if collision detected
        """
        return x <= self.min_x or x >= self.max_x or y <= self.min_y or y >= self.max_y

    def check_turtle_collision(self, turtle1_name: str, turtle2_name: str) -> bool:
        """Check if two turtles collide using Shapely geometry.

        Args:
            turtle1_name: Name of first turtle
            turtle2_name: Name of second turtle

        Returns:
            bool: True if collision detected
        """
        if (
            turtle1_name not in self.turtle_poses
            or turtle2_name not in self.turtle_poses
        ):
            return False

        pose1 = self.turtle_poses[turtle1_name].pose.position
        pose2 = self.turtle_poses[turtle2_name].pose.position

        # Create circular geometries for both turtles
        circle1 = Point(pose1.x, pose1.y).buffer(self.turtle_radius)
        circle2 = Point(pose2.x, pose2.y).buffer(self.turtle_radius)

        # Check if circles intersect
        return circle1.intersects(circle2)

    def is_collision_in_front(
        self, turtle_name: str, collision_x: float, collision_y: float
    ) -> bool:
        """Check if collision is in front of the turtle.

        Args:
            turtle_name: Name of the turtle
            collision_x: X coordinate of collision point
            collision_y: Y coordinate of collision point

        Returns:
            bool: True if collision is in front of turtle
        """
        if turtle_name not in self.turtle_poses:
            return False

        pose = self.turtle_poses[turtle_name]
        turtle_x = pose.pose.position.x
        turtle_y = pose.pose.position.y
        turtle_heading = self.get_turtle_heading(pose)

        # Calculate angle from turtle to collision point
        dx = collision_x - turtle_x
        dy = collision_y - turtle_y
        angle_to_collision = math.atan2(dy, dx)

        # Calculate relative angle (difference between collision direction and turtle heading)
        relative_angle = angle_to_collision - turtle_heading

        # Normalize angle to [-pi, pi]
        while relative_angle > math.pi:
            relative_angle -= 2 * math.pi
        while relative_angle < -math.pi:
            relative_angle += 2 * math.pi

        # Consider collision "in front" if within ±90 degrees
        return abs(relative_angle) <= math.pi / 2

    def create_bumper_markers(self) -> MarkerArray:
        """Create visualization markers for all turtle bumpers.

        Returns:
            MarkerArray: Array of markers representing bumper states
        """
        marker_array = MarkerArray()
        current_time = self.node.get_clock().now()

        # Create charging pad marker
        charging_pad_marker = Marker()
        charging_pad_marker.header.frame_id = "map"
        charging_pad_marker.header.stamp = current_time.to_msg()
        charging_pad_marker.ns = "charging_pad"
        charging_pad_marker.id = 0
        charging_pad_marker.type = Marker.CYLINDER
        charging_pad_marker.action = Marker.ADD

        # Position at charging pad location
        charging_pad_marker.pose.position.x = self.charging_pad_x
        charging_pad_marker.pose.position.y = self.charging_pad_y
        charging_pad_marker.pose.position.z = -0.01  # Slightly below ground surface

        # Set orientation
        charging_pad_marker.pose.orientation.w = 1.0

        # Set size
        charging_pad_marker.scale.x = self.charging_pad_radius * 2  # Diameter
        charging_pad_marker.scale.y = self.charging_pad_radius * 2  # Diameter
        charging_pad_marker.scale.z = 0.02  # Very thin

        # Set color (blue for charging pad)
        charging_pad_marker.color.r = 0.0
        charging_pad_marker.color.g = 0.5
        charging_pad_marker.color.b = 1.0
        charging_pad_marker.color.a = 0.6  # Semi-transparent

        marker_array.markers.append(charging_pad_marker)

        # Create charging pad text marker
        charging_text_marker = Marker()
        charging_text_marker.header.frame_id = "map"
        charging_text_marker.header.stamp = current_time.to_msg()
        charging_text_marker.ns = "charging_pad_text"
        charging_text_marker.id = 0
        charging_text_marker.type = Marker.TEXT_VIEW_FACING
        charging_text_marker.action = Marker.ADD

        # Position above charging pad
        charging_text_marker.pose.position.x = self.charging_pad_x
        charging_text_marker.pose.position.y = self.charging_pad_y
        charging_text_marker.pose.position.z = 0.1

        # Set orientation
        charging_text_marker.pose.orientation.w = 1.0

        # Set text
        charging_text_marker.text = "CHARGING PAD"

        # Set size
        charging_text_marker.scale.z = 0.15  # Text height

        # Set color (white text)
        charging_text_marker.color.r = 1.0
        charging_text_marker.color.g = 1.0
        charging_text_marker.color.b = 1.0
        charging_text_marker.color.a = 1.0

        marker_array.markers.append(charging_text_marker)

        for i in range(1, self.num_turtles + 1):
            turtle_name = f"turtle{i}"

            if turtle_name not in self.turtle_poses:
                continue

            # Get turtle pose and heading
            pose = self.turtle_poses[turtle_name]
            turtle_x = pose.pose.position.x
            turtle_y = pose.pose.position.y
            turtle_heading = self.get_turtle_heading(pose)

            # Calculate bumper position (slightly in front of turtle)
            bumper_offset = 0.4  # Distance in front of turtle
            bumper_x = turtle_x + bumper_offset * math.cos(turtle_heading)
            bumper_y = turtle_y + bumper_offset * math.sin(turtle_heading)

            # Create marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = current_time.to_msg()
            marker.ns = "bumpers"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Set position
            marker.pose.position.x = bumper_x
            marker.pose.position.y = bumper_y
            marker.pose.position.z = 0.05  # Slightly above ground

            # Set orientation (same as turtle)
            marker.pose.orientation = pose.pose.orientation

            # Set size
            marker.scale.x = 0.15  # Diameter
            marker.scale.y = 0.15  # Diameter
            marker.scale.z = 0.1  # Height

            # Set color based on collision state
            collision_detected = self.bumper_states.get(turtle_name, False)
            if collision_detected:
                # Red when collision detected
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            else:
                # Green when safe
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8

            marker_array.markers.append(marker)

        return marker_array

    def collision_detection_callback(self):
        """Timer callback for collision detection and bumper state publishing."""
        for i in range(1, self.num_turtles + 1):
            turtle_name = f"turtle{i}"

            if turtle_name not in self.turtle_poses:
                continue

            collision_detected = False
            pose = self.turtle_poses[turtle_name].pose.position

            # Check boundary collisions
            if self.check_boundary_collision(pose.x, pose.y):
                # For boundary collisions, check if turtle is moving towards the boundary
                turtle_heading = self.get_turtle_heading(self.turtle_poses[turtle_name])

                # Determine which boundary is closest and if turtle is heading towards it
                if (
                    (
                        pose.x <= self.min_x and math.cos(turtle_heading) < 0
                    )  # Left boundary
                    or (
                        pose.x >= self.max_x and math.cos(turtle_heading) > 0
                    )  # Right boundary
                    or (
                        pose.y <= self.min_y and math.sin(turtle_heading) < 0
                    )  # Bottom boundary
                    or (pose.y >= self.max_y and math.sin(turtle_heading) > 0)
                ):  # Top boundary
                    collision_detected = True

            # Check collisions with other turtles
            if not collision_detected:
                for j in range(1, self.num_turtles + 1):
                    if i == j:
                        continue

                    other_turtle = f"turtle{j}"
                    if self.check_turtle_collision(turtle_name, other_turtle):
                        # Check if collision is in front of current turtle
                        other_pose = self.turtle_poses[other_turtle].pose.position
                        if self.is_collision_in_front(
                            turtle_name, other_pose.x, other_pose.y
                        ):
                            collision_detected = True
                            break

            # Update bumper state for visualization
            self.bumper_states[turtle_name] = collision_detected

            # Publish bumper state
            bumper_msg = Bool()
            bumper_msg.data = collision_detected
            self.bumper_publishers[turtle_name].publish(bumper_msg)

            if collision_detected:
                self.logger.debug(f"{turtle_name} front bumper triggered")

        # Publish visualization markers
        marker_array = self.create_bumper_markers()
        self.marker_publisher.publish(marker_array)


class BatterySimulator:
    """
    Battery simulator that tracks battery levels for each turtle based on movement
    and publishes battery status and visualization markers.
    """

    def __init__(self, node: Node, default_num_turtles=3):
        """Initialize the BatterySimulator.

        Args:
            node: ROS2 node instance to use for communication
            default_num_turtles: Default number of turtles to simulate
        """
        self.node = node
        self.logger = self.node.get_logger()

        # Declare battery-specific parameters
        self.node.declare_parameter("battery_initial_level", 100.0)
        self.node.declare_parameter(
            "battery_drain_rate_idle", 1.0
        )  # %/second when idle
        self.node.declare_parameter(
            "battery_drain_rate_moving", 0.5
        )  # %/second when moving
        self.node.declare_parameter(
            "battery_drain_rate_rotating", 0.3
        )  # %/second when rotating
        self.node.declare_parameter("battery_update_rate", 5.0)  # Hz
        self.node.declare_parameter("battery_low_threshold", 20.0)  # %
        self.node.declare_parameter("battery_critical_threshold", 10.0)  # %
        self.node.declare_parameter("charging_pad_x", 5.5)  # Center x position
        self.node.declare_parameter("charging_pad_y", 5.5)  # Center y position
        self.node.declare_parameter("charging_pad_radius", 0.6)  # Charging area radius
        self.node.declare_parameter("charging_rate", 2.0)  # %/second when charging

        # Get parameters
        self.num_turtles = (
            self.node.get_parameter("num_turtles").get_parameter_value().integer_value
        )
        self.initial_level = (
            self.node.get_parameter("battery_initial_level")
            .get_parameter_value()
            .double_value
        )
        self.drain_rate_idle = (
            self.node.get_parameter("battery_drain_rate_idle")
            .get_parameter_value()
            .double_value
        )
        self.drain_rate_moving = (
            self.node.get_parameter("battery_drain_rate_moving")
            .get_parameter_value()
            .double_value
        )
        self.drain_rate_rotating = (
            self.node.get_parameter("battery_drain_rate_rotating")
            .get_parameter_value()
            .double_value
        )
        update_rate = (
            self.node.get_parameter("battery_update_rate")
            .get_parameter_value()
            .double_value
        )
        self.low_threshold = (
            self.node.get_parameter("battery_low_threshold")
            .get_parameter_value()
            .double_value
        )
        self.critical_threshold = (
            self.node.get_parameter("battery_critical_threshold")
            .get_parameter_value()
            .double_value
        )
        self.charging_pad_x = (
            self.node.get_parameter("charging_pad_x").get_parameter_value().double_value
        )
        self.charging_pad_y = (
            self.node.get_parameter("charging_pad_y").get_parameter_value().double_value
        )
        self.charging_pad_radius = (
            self.node.get_parameter("charging_pad_radius")
            .get_parameter_value()
            .double_value
        )
        self.charging_rate = (
            self.node.get_parameter("charging_rate").get_parameter_value().double_value
        )

        # Initialize battery levels and movement tracking
        self.battery_levels = {}
        self.last_poses = {}
        self.last_cmd_vels = {}
        self.last_update_time = time.time()
        self.power_drain_multiplier = 1.0  # Default multiplier

        for i in range(1, self.num_turtles + 1):
            turtle_name = f"turtle{i}"
            self.battery_levels[turtle_name] = self.initial_level
            self.last_poses[turtle_name] = None
            self.last_cmd_vels[turtle_name] = None

        # Create subscribers for each turtle's pose3d and cmd_vel
        self.pose_subscribers = []
        self.cmd_vel_subscribers = []

        for i in range(1, self.num_turtles + 1):
            turtle_name = f"turtle{i}"

            # Subscribe to pose3d
            pose_topic = f"/{turtle_name}/pose3d"
            pose_sub = self.node.create_subscription(
                PoseStamped,
                pose_topic,
                lambda msg, name=turtle_name: self.pose_callback(msg, name),
                10,
            )
            self.pose_subscribers.append(pose_sub)

            # Subscribe to cmd_vel
            cmd_vel_topic = f"/{turtle_name}/cmd_vel"
            cmd_vel_sub = self.node.create_subscription(
                Twist,
                cmd_vel_topic,
                lambda msg, name=turtle_name: self.cmd_vel_callback(msg, name),
                10,
            )
            self.cmd_vel_subscribers.append(cmd_vel_sub)

        # Create publishers for each turtle's battery level
        self.battery_publishers = {}
        for i in range(1, self.num_turtles + 1):
            turtle_name = f"turtle{i}"
            topic_name = f"/{turtle_name}/sensors/battery_level"
            publisher = self.node.create_publisher(Float32, topic_name, 10)
            self.battery_publishers[turtle_name] = publisher

        # Create publisher for battery visualization
        self.battery_marker_publisher = self.node.create_publisher(
            MarkerArray, "battery_visualization", 10
        )

        # Create subscriber for power drain multiplier
        self.power_drain_multiplier_subscriber = self.node.create_subscription(
            Float32,
            "/battery_simulator/power_drain_multiplier",
            self.power_drain_multiplier_callback,
            10,
        )

        # Create timer for battery updates
        self.battery_timer = self.node.create_timer(
            1.0 / update_rate, self.battery_update_callback
        )

        self.logger.info(f"BatterySimulator initialized for {self.num_turtles} turtles")
        self.logger.info(f"Initial battery level: {self.initial_level}%")
        self.logger.info(
            f"Drain rates - Idle: {self.drain_rate_idle}%/s, Moving: {self.drain_rate_moving}%/s, Rotating: {self.drain_rate_rotating}%/s"
        )
        self.logger.info(f"Battery update rate: {update_rate} Hz")

    def pose_callback(self, msg: PoseStamped, turtle_name: str):
        """Callback for turtle pose updates.

        Args:
            msg: PoseStamped message with turtle pose
            turtle_name: Name of the turtle (e.g., "turtle1")
        """
        self.last_poses[turtle_name] = msg

    def cmd_vel_callback(self, msg: Twist, turtle_name: str):
        """Callback for turtle cmd_vel updates.

        Args:
            msg: Twist message with velocity commands
            turtle_name: Name of the turtle (e.g., "turtle1")
        """
        self.last_cmd_vels[turtle_name] = msg

    def power_drain_multiplier_callback(self, msg: Float32):
        """Callback for power drain multiplier updates.

        Args:
            msg: Float32 message with multiplier value
        """
        self.power_drain_multiplier = max(0.0, msg.data)  # Ensure non-negative
        self.logger.info(
            f"Power drain multiplier updated to: {self.power_drain_multiplier}"
        )

    def get_movement_state(self, turtle_name: str) -> str:
        """Determine the movement state of a turtle.

        Args:
            turtle_name: Name of the turtle

        Returns:
            str: Movement state ("idle", "moving", "rotating")
        """
        if (
            turtle_name not in self.last_cmd_vels
            or self.last_cmd_vels[turtle_name] is None
        ):
            return "idle"

        cmd_vel = self.last_cmd_vels[turtle_name]
        linear_speed = abs(cmd_vel.linear.x)
        angular_speed = abs(cmd_vel.angular.z)

        # Thresholds for determining movement
        linear_threshold = 0.01
        angular_threshold = 0.01

        if linear_speed > linear_threshold:
            return "moving"
        elif angular_speed > angular_threshold:
            return "rotating"
        else:
            return "idle"

    def get_battery_status(self, level: float) -> str:
        """Get battery status based on level.

        Args:
            level: Battery level percentage

        Returns:
            str: Battery status ("critical", "low", "normal")
        """
        if level <= self.critical_threshold:
            return "critical"
        elif level <= self.low_threshold:
            return "low"
        else:
            return "normal"

    def is_turtle_on_charging_pad(self, turtle_name: str) -> bool:
        """Check if a turtle is on the charging pad.

        Args:
            turtle_name: Name of the turtle

        Returns:
            bool: True if turtle is on charging pad
        """
        if turtle_name not in self.last_poses or self.last_poses[turtle_name] is None:
            return False

        pose = self.last_poses[turtle_name]
        turtle_x = pose.pose.position.x
        turtle_y = pose.pose.position.y

        # Calculate distance from turtle to charging pad center
        distance = math.sqrt(
            (turtle_x - self.charging_pad_x) ** 2
            + (turtle_y - self.charging_pad_y) ** 2
        )

        return distance <= self.charging_pad_radius

    def create_battery_markers(self) -> MarkerArray:
        """Create visualization markers for all turtle batteries.

        Returns:
            MarkerArray: Array of markers representing battery levels
        """
        marker_array = MarkerArray()
        current_time = self.node.get_clock().now()

        # Check if any turtle is on the charging pad
        any_turtle_charging = False
        for i in range(1, self.num_turtles + 1):
            turtle_name = f"turtle{i}"
            if self.is_turtle_on_charging_pad(turtle_name):
                any_turtle_charging = True
                break

        # Create charging pad marker
        charging_pad_marker = Marker()
        charging_pad_marker.header.frame_id = "map"
        charging_pad_marker.header.stamp = current_time.to_msg()
        charging_pad_marker.ns = "charging_pad"
        charging_pad_marker.id = 0
        charging_pad_marker.type = Marker.CYLINDER
        charging_pad_marker.action = Marker.ADD

        # Position at charging pad location
        charging_pad_marker.pose.position.x = self.charging_pad_x
        charging_pad_marker.pose.position.y = self.charging_pad_y
        charging_pad_marker.pose.position.z = (
            -0.05
        )  # Well below ground to avoid tf frame conflict

        # Set orientation
        charging_pad_marker.pose.orientation.w = 1.0

        # Set size (smaller)
        charging_pad_marker.scale.x = self.charging_pad_radius * 2  # Diameter
        charging_pad_marker.scale.y = self.charging_pad_radius * 2  # Diameter
        charging_pad_marker.scale.z = 0.02  # Very thin

        # Set color based on charging status (using colors that don't conflict with text)
        if any_turtle_charging:
            # Orange when a turtle is charging
            charging_pad_marker.color.r = 1.0
            charging_pad_marker.color.g = 0.5
            charging_pad_marker.color.b = 0.0
            charging_pad_marker.color.a = 0.8
        else:
            # Purple when idle
            charging_pad_marker.color.r = 0.5
            charging_pad_marker.color.g = 0.0
            charging_pad_marker.color.b = 1.0
            charging_pad_marker.color.a = 0.6

        marker_array.markers.append(charging_pad_marker)

        # Create charging pad text marker
        charging_text_marker = Marker()
        charging_text_marker.header.frame_id = "map"
        charging_text_marker.header.stamp = current_time.to_msg()
        charging_text_marker.ns = "charging_pad_text"
        charging_text_marker.id = 0
        charging_text_marker.type = Marker.TEXT_VIEW_FACING
        charging_text_marker.action = Marker.ADD

        # Position above charging pad
        charging_text_marker.pose.position.x = self.charging_pad_x
        charging_text_marker.pose.position.y = self.charging_pad_y
        charging_text_marker.pose.position.z = 0.1

        # Set orientation
        charging_text_marker.pose.orientation.w = 1.0

        # Set text based on charging status
        if any_turtle_charging:
            charging_text_marker.text = "CHARGING"
        else:
            charging_text_marker.text = "CHARGING PAD"

        # Set size
        charging_text_marker.scale.z = 0.15  # Text height

        # Set color (white text)
        charging_text_marker.color.r = 1.0
        charging_text_marker.color.g = 1.0
        charging_text_marker.color.b = 1.0
        charging_text_marker.color.a = 1.0

        marker_array.markers.append(charging_text_marker)

        for i in range(1, self.num_turtles + 1):
            turtle_name = f"turtle{i}"

            if (
                turtle_name not in self.last_poses
                or self.last_poses[turtle_name] is None
            ):
                continue

            # Get turtle pose
            pose = self.last_poses[turtle_name]
            turtle_x = pose.pose.position.x
            turtle_y = pose.pose.position.y

            # Battery level
            battery_level = self.battery_levels[turtle_name]
            battery_status = self.get_battery_status(battery_level)

            # Create battery level text marker
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = current_time.to_msg()
            text_marker.ns = "battery_text"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            # Position above battery indicator
            text_marker.pose.position.x = turtle_x
            text_marker.pose.position.y = turtle_y
            text_marker.pose.position.z = 0.9

            # Set orientation
            text_marker.pose.orientation.w = 1.0

            # Set text
            text_marker.text = f"{battery_level:.1f}%"

            # Set size
            text_marker.scale.z = 0.1  # Text height

            # Set color based on battery status
            if battery_status == "critical":
                # Red for critical
                text_marker.color.r = 1.0
                text_marker.color.g = 0.0
                text_marker.color.b = 0.0
                text_marker.color.a = 1.0
            else:
                # Green for normal and low
                text_marker.color.r = 0.0
                text_marker.color.g = 1.0
                text_marker.color.b = 0.0
                text_marker.color.a = 1.0

            marker_array.markers.append(text_marker)

        return marker_array

    def battery_update_callback(self):
        """Timer callback for battery level updates."""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        for i in range(1, self.num_turtles + 1):
            turtle_name = f"turtle{i}"

            # Get movement state and determine drain rate
            movement_state = self.get_movement_state(turtle_name)

            if movement_state == "moving":
                drain_rate = self.drain_rate_moving
            elif movement_state == "rotating":
                drain_rate = self.drain_rate_rotating
            else:
                drain_rate = self.drain_rate_idle

            # Check if turtle is on charging pad
            if self.is_turtle_on_charging_pad(turtle_name):
                # Charge the battery
                battery_charge = self.charging_rate * dt
                self.battery_levels[turtle_name] = min(
                    100.0, self.battery_levels[turtle_name] + battery_charge
                )
            else:
                # Update battery level (drain) with multiplier
                battery_drain = drain_rate * dt * self.power_drain_multiplier
                self.battery_levels[turtle_name] = max(
                    0.0, self.battery_levels[turtle_name] - battery_drain
                )

            # Log battery status changes
            battery_level = self.battery_levels[turtle_name]
            battery_status = self.get_battery_status(battery_level)

            # if battery_status == "critical" and battery_level > 0:
            #     self.logger.warn(
            #         f"{turtle_name} battery critical: {battery_level:.1f}%"
            #     )
            # elif battery_status == "low":
            #     self.logger.info(f"{turtle_name} battery low: {battery_level:.1f}%")

            # Publish battery level
            battery_msg = Float32()
            battery_msg.data = battery_level
            self.battery_publishers[turtle_name].publish(battery_msg)

        # Publish visualization markers
        marker_array = self.create_battery_markers()
        self.battery_marker_publisher.publish(marker_array)


def main(args=None):
    """Main function to run the simulator extensions."""
    rclpy.init(args=args)

    node = Node("simulator_extensions")
    bumper_simulator = BumperSimulator(node)
    battery_simulator = BatterySimulator(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
