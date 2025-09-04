#!/usr/bin/env python3
"""
Test node for inverse transform functions
Subscribes to turtle1 and turtle2 poses and prints relative pose
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from geometry_msgs.msg import PoseStamped

from transform_utils.transforms import (
    inverse_transform_pose,
    transform_pose,
    yaw_from_quaternion,
    distance_2d,
)
import math


class TurtleRelativePoseNode(Node):
    """
    Node that subscribes to turtle1 and turtle2 poses and prints relative pose
    using the new inverse transform functions
    """

    def __init__(self):
        super().__init__("turtle_relative_pose_node")

        self.get_logger().info("Starting turtle relative pose test node")

        # Store poses
        self.turtle1_pose = None
        self.turtle2_pose = None

        # Subscribers
        self.turtle1_sub = self.create_subscription(
            PoseStamped, "/turtle1/pose3d", self.turtle1_pose_callback, 10
        )

        self.turtle2_sub = self.create_subscription(
            PoseStamped, "/turtle2/pose3d", self.turtle2_pose_callback, 10
        )

        # Timer to print relative pose periodically
        self.timer = self.create_timer(2.0, self.print_relative_pose)

        self.get_logger().info("Subscribed to /turtle1/pose3d and /turtle2/pose3d")
        self.get_logger().info("Will print relative pose every 2 seconds")

    def turtle1_pose_callback(self, msg: PoseStamped):
        """Callback for turtle1 pose"""
        self.turtle1_pose = msg.pose

    def turtle2_pose_callback(self, msg: PoseStamped):
        """Callback for turtle2 pose"""
        self.turtle2_pose = msg.pose

    def print_relative_pose(self):
        """Print relative pose of turtle2 with respect to turtle1"""
        if self.turtle1_pose is None or self.turtle2_pose is None:
            self.get_logger().info("Waiting for both turtle poses...")
            return

        try:
            # Use turtle1 as reference frame (like Unity Transform)
            # Calculate turtle2's pose relative to turtle1
            turtle2_relative = inverse_transform_pose(
                self.turtle1_pose, self.turtle2_pose
            )

            # Extract information
            turtle1_yaw = yaw_from_quaternion(self.turtle1_pose.orientation)
            turtle2_yaw = yaw_from_quaternion(self.turtle2_pose.orientation)
            turtle2_relative_yaw = yaw_from_quaternion(turtle2_relative.orientation)

            # Calculate distance
            distance = distance_2d(self.turtle1_pose, self.turtle2_pose)

            # Print world poses
            self.get_logger().info("=" * 60)
            self.get_logger().info("WORLD POSES:")
            self.get_logger().info(
                f"  Turtle1: ({self.turtle1_pose.position.x:.2f}, {self.turtle1_pose.position.y:.2f}) @ {math.degrees(turtle1_yaw):.1f}°"
            )
            self.get_logger().info(
                f"  Turtle2: ({self.turtle2_pose.position.x:.2f}, {self.turtle2_pose.position.y:.2f}) @ {math.degrees(turtle2_yaw):.1f}°"
            )

            # Print relative pose (turtle2 relative to turtle1)
            self.get_logger().info("RELATIVE POSE (Turtle2 relative to Turtle1):")
            self.get_logger().info(
                f"  Position: ({turtle2_relative.position.x:.2f}, {turtle2_relative.position.y:.2f})"
            )
            self.get_logger().info(
                f"  Orientation: {math.degrees(turtle2_relative_yaw):.1f}°"
            )
            self.get_logger().info(f"  Distance: {distance:.2f}m")

            # Unity-style directional information
            if turtle2_relative.position.x > 0.1:
                direction_x = "AHEAD"
            elif turtle2_relative.position.x < -0.1:
                direction_x = "BEHIND"
            else:
                direction_x = "ALIGNED"

            if turtle2_relative.position.y > 0.1:
                direction_y = "LEFT"
            elif turtle2_relative.position.y < -0.1:
                direction_y = "RIGHT"
            else:
                direction_y = "ALIGNED"

            self.get_logger().info(
                f"  Direction: Turtle2 is {direction_x} and to the {direction_y} of Turtle1"
            )

            # Test inverse operation - convert back to world coordinates
            turtle2_world_back = transform_pose(self.turtle1_pose, turtle2_relative)
            error_x = abs(turtle2_world_back.position.x - self.turtle2_pose.position.x)
            error_y = abs(turtle2_world_back.position.y - self.turtle2_pose.position.y)

            self.get_logger().info("VERIFICATION:")
            self.get_logger().info(
                f"  Transform back error: ({error_x:.6f}, {error_y:.6f})"
            )

            if error_x < 1e-6 and error_y < 1e-6:
                self.get_logger().info("  ✓ Inverse transform working correctly!")
            else:
                self.get_logger().warn("  ✗ Inverse transform has errors!")

        except Exception as e:
            self.get_logger().error(f"Error calculating relative pose: {e}")
            import traceback

            traceback.print_exc()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    node = TurtleRelativePoseNode()

    try:
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        node.get_logger().info("Node started. Press Ctrl+C to stop.")
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
