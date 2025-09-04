#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
import math
import time

# Import the shared state manager from simulator_state
from .simulator_state import SimulatorStateManager


class BatterySimulator:
    """
    Battery simulator that uses the shared SimulatorStateManager
    for all state access and battery level management.
    """

    def __init__(self, node: Node, state_manager: SimulatorStateManager):
        self.node = node
        self.logger = node.get_logger()
        self.state_manager = state_manager
        self.config = state_manager.config

        self.last_update_time = time.time()

        # Create publishers for each turtle's battery level
        self.battery_publishers = {}
        for turtle_name in self.state_manager.get_all_turtle_names():
            topic_name = f"/{turtle_name}/sensors/battery_level"
            publisher = self.node.create_publisher(Float32, topic_name, 10)
            self.battery_publishers[turtle_name] = publisher

        # Create publisher for battery visualization
        self.battery_marker_publisher = self.node.create_publisher(
            MarkerArray, "battery_visualization", 10
        )

        # Create timer for battery updates
        self.battery_timer = self.node.create_timer(
            1.0 / self.config.battery_update_rate, self.battery_update_callback
        )

        self.logger.info("BatterySimulator initialized with shared state manager")

    def get_movement_state(self, turtle_name: str) -> str:
        """Determine the movement state of a turtle"""
        turtle_state = self.state_manager.get_turtle_state(turtle_name)
        if not turtle_state or not turtle_state.cmd_vel:
            return "idle"

        cmd_vel = turtle_state.cmd_vel
        linear_speed = abs(cmd_vel.linear.x)
        angular_speed = abs(cmd_vel.angular.z)

        linear_threshold = 0.01
        angular_threshold = 0.01

        if linear_speed > linear_threshold:
            return "moving"
        elif angular_speed > angular_threshold:
            return "rotating"
        else:
            return "idle"

    def get_battery_status(self, level: float) -> str:
        """Get battery status based on level"""
        if level <= self.config.battery_critical_threshold:
            return "critical"
        elif level <= self.config.battery_low_threshold:
            return "low"
        else:
            return "normal"

    def is_turtle_on_charging_pad(self, turtle_name: str) -> bool:
        """Check if a turtle is on the charging pad"""
        turtle_state = self.state_manager.get_turtle_state(turtle_name)
        if not turtle_state or not turtle_state.pose:
            return False

        pose = turtle_state.pose
        turtle_x = pose.pose.position.x
        turtle_y = pose.pose.position.y

        distance = math.sqrt(
            (turtle_x - self.config.charging_pad_x) ** 2
            + (turtle_y - self.config.charging_pad_y) ** 2
        )

        return distance <= self.config.charging_pad_radius

    def create_battery_markers(self) -> MarkerArray:
        """Create visualization markers for all turtle batteries"""
        marker_array = MarkerArray()
        current_time = self.node.get_clock().now()

        # Check if any turtle is charging
        any_turtle_charging = any(
            self.is_turtle_on_charging_pad(name)
            for name in self.state_manager.get_all_turtle_names()
        )

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
        charging_pad_marker.pose.position.z = -0.05
        charging_pad_marker.pose.orientation.w = 1.0

        charging_pad_marker.scale.x = self.config.charging_pad_radius * 2
        charging_pad_marker.scale.y = self.config.charging_pad_radius * 2
        charging_pad_marker.scale.z = 0.02

        if any_turtle_charging:
            charging_pad_marker.color.r = 1.0
            charging_pad_marker.color.g = 0.5
            charging_pad_marker.color.b = 0.0
            charging_pad_marker.color.a = 0.8
        else:
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

        charging_text_marker.pose.position.x = self.config.charging_pad_x
        charging_text_marker.pose.position.y = self.config.charging_pad_y
        charging_text_marker.pose.position.z = 0.1
        charging_text_marker.pose.orientation.w = 1.0

        charging_text_marker.text = (
            "CHARGING" if any_turtle_charging else "CHARGING PAD"
        )
        charging_text_marker.scale.z = 0.15

        charging_text_marker.color.r = 1.0
        charging_text_marker.color.g = 1.0
        charging_text_marker.color.b = 1.0
        charging_text_marker.color.a = 1.0

        marker_array.markers.append(charging_text_marker)

        # Create battery level markers for each turtle
        for i, turtle_name in enumerate(self.state_manager.get_all_turtle_names(), 1):
            turtle_state = self.state_manager.get_turtle_state(turtle_name)
            if not turtle_state or not turtle_state.pose:
                continue

            pose = turtle_state.pose
            turtle_x = pose.pose.position.x
            turtle_y = pose.pose.position.y
            battery_level = turtle_state.battery_level
            battery_status = self.get_battery_status(battery_level)

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = current_time.to_msg()
            text_marker.ns = "battery_text"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = turtle_x
            text_marker.pose.position.y = turtle_y
            text_marker.pose.position.z = 0.9
            text_marker.pose.orientation.w = 1.0

            text_marker.text = f"{battery_level:.1f}%"
            text_marker.scale.z = 0.1

            if battery_status == "critical":
                text_marker.color.r = 1.0
                text_marker.color.g = 0.0
                text_marker.color.b = 0.0
                text_marker.color.a = 1.0
            else:
                text_marker.color.r = 0.0
                text_marker.color.g = 1.0
                text_marker.color.b = 0.0
                text_marker.color.a = 1.0

            marker_array.markers.append(text_marker)

        return marker_array

    def battery_update_callback(self):
        """Timer callback for battery level updates"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        for turtle_name in self.state_manager.get_all_turtle_names():
            turtle_state = self.state_manager.get_turtle_state(turtle_name)
            if not turtle_state:
                continue

            # Get movement state and determine drain rate
            movement_state = self.get_movement_state(turtle_name)

            if movement_state == "moving":
                drain_rate = self.config.battery_drain_rate_moving
            elif movement_state == "rotating":
                drain_rate = self.config.battery_drain_rate_rotating
            else:
                drain_rate = self.config.battery_drain_rate_idle

            # Check if turtle is on charging pad
            if self.is_turtle_on_charging_pad(turtle_name):
                # Charge the battery
                battery_charge = self.config.charging_rate * dt
                new_level = min(100.0, turtle_state.battery_level + battery_charge)
            else:
                # Drain the battery
                power_multiplier = self.state_manager.get_power_drain_multiplier()
                battery_drain = drain_rate * dt * power_multiplier
                new_level = max(0.0, turtle_state.battery_level - battery_drain)

            # Update battery level in state manager
            self.state_manager.update_turtle_battery(turtle_name, new_level)

            # Publish battery level
            battery_msg = Float32()
            battery_msg.data = new_level
            if turtle_name in self.battery_publishers:
                self.battery_publishers[turtle_name].publish(battery_msg)

        # Publish visualization markers
        marker_array = self.create_battery_markers()
        self.battery_marker_publisher.publish(marker_array)
