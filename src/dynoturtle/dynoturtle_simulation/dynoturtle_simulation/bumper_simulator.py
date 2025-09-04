#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from shapely.geometry import Point
from transform_utils.transforms import yaw_from_quaternion
import math

# Import the shared state manager from main
from dynoturtle_simulation.simulator_state import (
    SimulatorStateManager,
    SimulationConfig,
)


class BumperSimulator:
    """
    Bumper collision simulator that uses the shared SimulatorStateManager
    for all state access and collision detection.
    """

    def __init__(self, node: Node, state_manager: SimulatorStateManager):
        self.node = node
        self.logger = node.get_logger()
        self.state_manager = state_manager
        self.config = state_manager.config

        # Calculated boundaries
        self.min_x = self.config.min_x + self.config.boundary_margin
        self.max_x = self.config.max_x - self.config.boundary_margin
        self.min_y = self.config.min_y + self.config.boundary_margin
        self.max_y = self.config.max_y - self.config.boundary_margin

        # Create publishers for each turtle's front bumper
        self.bumper_publishers = {}
        for turtle_name in self.state_manager.get_all_turtle_names():
            topic_name = f"/{turtle_name}/sensors/front_bumper"
            publisher = self.node.create_publisher(Bool, topic_name, 10)
            self.bumper_publishers[turtle_name] = publisher

        # Create publisher for bumper visualization
        self.marker_publisher = self.node.create_publisher(
            MarkerArray, "bumper_visualization", 10
        )

        # Create timer for collision detection
        self.timer = self.node.create_timer(
            1.0 / self.config.bumper_update_rate, self.collision_detection_callback
        )

        self.logger.info("BumperSimulator initialized with shared state manager")

    def get_turtle_heading(self, pose_stamped: PoseStamped) -> float:
        """Extract heading angle from quaternion in PoseStamped message"""
        return yaw_from_quaternion(pose_stamped.pose.orientation)

    def check_boundary_collision(self, x: float, y: float) -> bool:
        """Check if turtle position collides with simulation boundaries"""
        return x <= self.min_x or x >= self.max_x or y <= self.min_y or y >= self.max_y

    def check_turtle_collision(self, turtle1_name: str, turtle2_name: str) -> bool:
        """Check if two turtles collide using Shapely geometry"""
        turtle1_state = self.state_manager.get_turtle_state(turtle1_name)
        turtle2_state = self.state_manager.get_turtle_state(turtle2_name)

        if (
            not turtle1_state
            or not turtle2_state
            or not turtle1_state.pose
            or not turtle2_state.pose
        ):
            return False

        pose1 = turtle1_state.pose.pose.position
        pose2 = turtle2_state.pose.pose.position

        # Create circular geometries for both turtles
        circle1 = Point(pose1.x, pose1.y).buffer(self.config.turtle_radius)
        circle2 = Point(pose2.x, pose2.y).buffer(self.config.turtle_radius)

        return circle1.intersects(circle2)

    def is_collision_in_front(
        self, turtle_name: str, collision_x: float, collision_y: float
    ) -> bool:
        """Check if collision is in front of the turtle"""
        turtle_state = self.state_manager.get_turtle_state(turtle_name)
        if not turtle_state or not turtle_state.pose:
            return False

        pose = turtle_state.pose
        turtle_x = pose.pose.position.x
        turtle_y = pose.pose.position.y
        turtle_heading = self.get_turtle_heading(pose)

        # Calculate angle from turtle to collision point
        dx = collision_x - turtle_x
        dy = collision_y - turtle_y
        angle_to_collision = math.atan2(dy, dx)

        # Calculate relative angle
        relative_angle = angle_to_collision - turtle_heading

        # Normalize angle to [-pi, pi]
        while relative_angle > math.pi:
            relative_angle -= 2 * math.pi
        while relative_angle < -math.pi:
            relative_angle += 2 * math.pi

        # Consider collision "in front" if within ±90 degrees
        return abs(relative_angle) <= math.pi / 2

    def create_bumper_markers(self) -> MarkerArray:
        """Create visualization markers for all turtle bumpers"""
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

        charging_pad_marker.pose.position.x = self.config.charging_pad_x
        charging_pad_marker.pose.position.y = self.config.charging_pad_y
        charging_pad_marker.pose.position.z = -0.01
        charging_pad_marker.pose.orientation.w = 1.0

        charging_pad_marker.scale.x = self.config.charging_pad_radius * 2
        charging_pad_marker.scale.y = self.config.charging_pad_radius * 2
        charging_pad_marker.scale.z = 0.02

        charging_pad_marker.color.r = 0.0
        charging_pad_marker.color.g = 0.5
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

        charging_text_marker.pose.position.x = self.config.charging_pad_x
        charging_text_marker.pose.position.y = self.config.charging_pad_y
        charging_text_marker.pose.position.z = 0.1
        charging_text_marker.pose.orientation.w = 1.0

        charging_text_marker.text = "CHARGING PAD"
        charging_text_marker.scale.z = 0.15

        charging_text_marker.color.r = 1.0
        charging_text_marker.color.g = 1.0
        charging_text_marker.color.b = 1.0
        charging_text_marker.color.a = 1.0

        marker_array.markers.append(charging_text_marker)

        # Create bumper markers for each turtle
        for i, turtle_name in enumerate(self.state_manager.get_all_turtle_names(), 1):
            turtle_state = self.state_manager.get_turtle_state(turtle_name)
            if not turtle_state or not turtle_state.pose:
                continue

            pose = turtle_state.pose
            turtle_x = pose.pose.position.x
            turtle_y = pose.pose.position.y
            turtle_heading = self.get_turtle_heading(pose)

            # Calculate bumper position
            bumper_offset = 0.4
            bumper_x = turtle_x + bumper_offset * math.cos(turtle_heading)
            bumper_y = turtle_y + bumper_offset * math.sin(turtle_heading)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = current_time.to_msg()
            marker.ns = "bumpers"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = bumper_x
            marker.pose.position.y = bumper_y
            marker.pose.position.z = 0.05
            marker.pose.orientation = pose.pose.orientation

            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.1

            # Color based on collision state
            if turtle_state.collision_state:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8

            marker_array.markers.append(marker)

        return marker_array

    def collision_detection_callback(self):
        """Timer callback for collision detection and bumper state publishing"""
        for turtle_name in self.state_manager.get_all_turtle_names():
            turtle_state = self.state_manager.get_turtle_state(turtle_name)
            if not turtle_state or not turtle_state.pose:
                continue

            collision_detected = False
            pose = turtle_state.pose.pose.position

            # Check boundary collisions
            if self.check_boundary_collision(pose.x, pose.y):
                turtle_heading = self.get_turtle_heading(turtle_state.pose)

                # Check if turtle is moving towards the boundary
                if (
                    (pose.x <= self.min_x and math.cos(turtle_heading) < 0)
                    or (pose.x >= self.max_x and math.cos(turtle_heading) > 0)
                    or (pose.y <= self.min_y and math.sin(turtle_heading) < 0)
                    or (pose.y >= self.max_y and math.sin(turtle_heading) > 0)
                ):
                    collision_detected = True

            # Check collisions with other turtles
            if not collision_detected:
                for other_turtle in self.state_manager.get_all_turtle_names():
                    if turtle_name == other_turtle:
                        continue

                    if self.check_turtle_collision(turtle_name, other_turtle):
                        other_state = self.state_manager.get_turtle_state(other_turtle)
                        if other_state and other_state.pose:
                            other_pose = other_state.pose.pose.position
                            if self.is_collision_in_front(
                                turtle_name, other_pose.x, other_pose.y
                            ):
                                collision_detected = True
                                break

            # Update collision state in state manager
            self.state_manager.update_turtle_collision_state(
                turtle_name, collision_detected
            )

            # Publish bumper state
            bumper_msg = Bool()
            bumper_msg.data = collision_detected
            if turtle_name in self.bumper_publishers:
                self.bumper_publishers[turtle_name].publish(bumper_msg)

            if collision_detected:
                self.logger.debug(f"{turtle_name} front bumper triggered")

        # Publish visualization markers
        marker_array = self.create_bumper_markers()
        self.marker_publisher.publish(marker_array)
