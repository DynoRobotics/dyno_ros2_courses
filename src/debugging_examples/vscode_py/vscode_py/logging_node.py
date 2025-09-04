#!/usr/bin/env python3
"""
ROS2 Logging Example Node - Comprehensive Demonstration of All Logging Levels

This node demonstrates proper usage of all ROS2 logging levels with practical robotics scenarios.
Each logging level serves a specific purpose in robotics system development and debugging.

Author: Collaborative Individuation Collective
Purpose: Educational example for ROS2 logging best practices
"""

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
import time
import random
import math


class LoggingExampleNode(Node):
    """
    Comprehensive ROS2 logging demonstration node.

    This node simulates a robot control system and demonstrates when and how
    to use each logging level appropriately in robotics applications.
    """

    def __init__(self):
        super().__init__("logging_example_node")

        # Initialize system state
        self.robot_state = "initializing"
        self.battery_level = 100.0
        self.sensor_readings = []
        self.error_count = 0
        self.warning_count = 0
        self.loop_count = 0

        # Publishers for demonstration
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "robot_status", 10)
        self.battery_pub = self.create_publisher(Float64, "battery_level", 10)

        # Timer for main control loop
        self.timer = self.create_timer(1.0, self.control_loop)

        # Demonstrate DEBUG level - detailed system initialization
        self.get_logger().debug("Node initialization started")
        self.get_logger().debug(f"Publishers created: cmd_vel, status, battery")
        self.get_logger().debug(f"Timer created with 1.0s period")

        # Demonstrate INFO level - important system events
        self.get_logger().info("🤖 Logging Example Node initialized successfully")
        self.get_logger().info("System ready for operation")

        # Set initial state
        self.robot_state = "ready"

    def control_loop(self):
        """
        Main control loop demonstrating various logging scenarios.
        """
        self.loop_count += 1

        # Simulate battery drain
        self.battery_level -= random.uniform(0.1, 0.5)

        # Simulate sensor readings
        sensor_value = random.uniform(0, 100)
        self.sensor_readings.append(sensor_value)
        if len(self.sensor_readings) > 10:
            self.sensor_readings.pop(0)

        # DEBUG: Detailed operational information
        self.get_logger().debug(
            f"Loop {self.loop_count}: Battery={self.battery_level:.1f}%, "
            f"Sensor={sensor_value:.1f}"
        )

        # INFO: Normal operational status
        if self.loop_count % 5 == 0:  # Every 5 seconds
            avg_sensor = sum(self.sensor_readings) / len(self.sensor_readings)
            self.get_logger().info(
                f"📊 System status: Battery {self.battery_level:.1f}%, "
                f"Avg sensor reading: {avg_sensor:.1f}"
            )

        # WARN: Potential issues that need attention
        if self.battery_level < 30 and self.battery_level > 15:
            self.warning_count += 1
            self.get_logger().warn(
                f"⚠️  Low battery warning: {self.battery_level:.1f}% remaining"
            )
            self.robot_state = "low_battery"

        # ERROR: Serious problems that affect functionality
        if self.battery_level < 15:
            self.error_count += 1
            self.get_logger().error(
                f"🔋 Critical battery level: {self.battery_level:.1f}% - "
                "Initiating emergency procedures"
            )
            self.robot_state = "emergency"
            self.emergency_shutdown()

        # FATAL: System-critical failures (simulated)
        if sensor_value > 95:  # Simulate critical sensor failure
            self.get_logger().fatal(
                f"💥 CRITICAL SENSOR FAILURE: Reading {sensor_value:.1f} "
                "exceeds safe operating limits - IMMEDIATE SHUTDOWN REQUIRED"
            )
            self.robot_state = "fatal_error"
            self.emergency_shutdown()
            return

        # Demonstrate different logging scenarios based on system state
        self.demonstrate_logging_scenarios()

        # Publish status
        self.publish_status()

    def demonstrate_logging_scenarios(self):
        """
        Demonstrate various logging scenarios common in robotics.
        """

        # Navigation logging example
        if self.loop_count % 3 == 0:
            target_x, target_y = random.uniform(-5, 5), random.uniform(-5, 5)
            distance = math.sqrt(target_x**2 + target_y**2)

            self.get_logger().debug(
                f"🎯 Navigation: Target ({target_x:.2f}, {target_y:.2f}), "
                f"Distance: {distance:.2f}m"
            )

            if distance > 10:
                self.get_logger().warn(
                    f"🚨 Navigation target very far: {distance:.2f}m - "
                    "May exceed operational range"
                )

        # Sensor validation logging
        if len(self.sensor_readings) >= 3:
            recent_readings = self.sensor_readings[-3:]
            variance = max(recent_readings) - min(recent_readings)

            if variance > 20:
                self.get_logger().warn(
                    f"📈 High sensor variance detected: {variance:.1f} - "
                    "Possible sensor instability"
                )
            elif variance < 1:
                self.get_logger().debug(
                    f"📉 Stable sensor readings: variance {variance:.1f}"
                )

        # Communication logging example
        if self.loop_count % 7 == 0:
            connection_quality = random.uniform(0, 100)

            if connection_quality > 80:
                self.get_logger().debug(
                    f"📡 Excellent connection quality: {connection_quality:.1f}%"
                )
            elif connection_quality > 50:
                self.get_logger().info(
                    f"📡 Good connection quality: {connection_quality:.1f}%"
                )
            elif connection_quality > 20:
                self.get_logger().warn(
                    f"📡 Poor connection quality: {connection_quality:.1f}% - "
                    "May affect remote control"
                )
            else:
                self.get_logger().error(
                    f"📡 Connection lost: {connection_quality:.1f}% - "
                    "Switching to autonomous mode"
                )

        # Performance monitoring
        if self.loop_count % 10 == 0:
            cpu_usage = random.uniform(10, 90)
            memory_usage = random.uniform(20, 80)

            self.get_logger().info(
                f"💻 Performance: CPU {cpu_usage:.1f}%, Memory {memory_usage:.1f}%"
            )

            if cpu_usage > 80:
                self.get_logger().warn(
                    f"🔥 High CPU usage: {cpu_usage:.1f}% - "
                    "Consider reducing computational load"
                )

            if memory_usage > 70:
                self.get_logger().warn(
                    f"🧠 High memory usage: {memory_usage:.1f}% - "
                    "Memory cleanup may be needed"
                )

    def emergency_shutdown(self):
        """
        Demonstrate emergency shutdown logging.
        """
        self.get_logger().error("🛑 Initiating emergency shutdown sequence")

        # Stop all movement
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info("✋ All movement commands stopped")

        # Log shutdown steps
        self.get_logger().info("💾 Saving critical system state")
        self.get_logger().info("📡 Sending emergency status to base station")
        self.get_logger().warn(
            "⚠️  Robot entering safe mode - manual intervention required"
        )

    def publish_status(self):
        """
        Publish current robot status.
        """
        # Publish status message
        status_msg = String()
        status_msg.data = f"{self.robot_state}|{self.battery_level:.1f}|{self.error_count}|{self.warning_count}"
        self.status_pub.publish(status_msg)

        # Publish battery level
        battery_msg = Float64()
        battery_msg.data = self.battery_level
        self.battery_pub.publish(battery_msg)

        # Publish movement command based on state
        cmd_vel = Twist()
        if self.robot_state == "ready":
            # Normal operation - gentle movement
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0.1
        elif self.robot_state == "low_battery":
            # Reduced speed operation
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.05
        else:
            # Emergency or error state - stop
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd_vel)

    def demonstrate_logging_levels(self):
        """
        Explicit demonstration of each logging level with explanations.
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎓 LOGGING LEVELS DEMONSTRATION")
        self.get_logger().info("=" * 60)

        # DEBUG - Detailed diagnostic information
        self.get_logger().debug(
            "🔍 DEBUG: Detailed diagnostic information for developers"
        )
        self.get_logger().debug(
            "   Use for: Variable values, function entry/exit, detailed state"
        )
        self.get_logger().debug(
            "   Example: Loop iteration 42, sensor_value=23.7, state=ready"
        )

        # INFO - General information about system operation
        self.get_logger().info(
            "ℹ️  INFO: General information about normal system operation"
        )
        self.get_logger().info(
            "   Use for: System startup, normal state changes, milestones"
        )
        self.get_logger().info(
            "   Example: Node initialized, Mission started, Target reached"
        )

        # WARN - Potentially harmful situations
        self.get_logger().warn(
            "⚠️  WARN: Potentially harmful situations that need attention"
        )
        self.get_logger().warn(
            "   Use for: Recoverable errors, performance issues, unusual conditions"
        )
        self.get_logger().warn(
            "   Example: Low battery, High CPU usage, Connection unstable"
        )

        # ERROR - Error events that might still allow operation
        self.get_logger().error(
            "❌ ERROR: Error events that affect functionality but allow operation"
        )
        self.get_logger().error(
            "   Use for: Failed operations, missing resources, communication failures"
        )
        self.get_logger().error(
            "   Example: Sensor failure, Navigation error, Service unavailable"
        )

        # FATAL - Very severe error events that will lead to application abort
        self.get_logger().fatal(
            "💥 FATAL: Critical errors that require immediate shutdown"
        )
        self.get_logger().fatal(
            "   Use for: System-critical failures, safety violations, unrecoverable errors"
        )
        self.get_logger().fatal(
            "   Example: Hardware failure, Safety limit exceeded, Memory corruption"
        )

        self.get_logger().info("=" * 60)


def main(args=None):
    """
    Main function to run the logging example node.
    """
    # Initialize ROS2
    rclpy.init(args=args)

    # Create and configure the node
    node = LoggingExampleNode()

    # Log startup information
    node.get_logger().info("🚀 Starting ROS2 Logging Example Node")
    node.get_logger().info("📝 This node demonstrates all ROS2 logging levels")
    node.get_logger().info(
        "🎯 Watch the logs to see different severity levels in action"
    )

    # Demonstrate logging levels explicitly
    node.demonstrate_logging_levels()

    try:
        # Run the node
        node.get_logger().info("🔄 Entering main execution loop")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("🛑 Keyboard interrupt received")

    except Exception as e:
        node.get_logger().fatal(f"💥 Unexpected error: {str(e)}")

    finally:
        # Cleanup
        node.get_logger().info("🧹 Cleaning up and shutting down")
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Logging example node shutdown complete")


if __name__ == "__main__":
    main()
