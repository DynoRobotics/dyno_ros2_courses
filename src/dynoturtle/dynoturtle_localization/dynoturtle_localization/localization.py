#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from turtlesim.msg import Pose
from tf2_ros import TransformBroadcaster
from transform_utils.transforms import quaternion_from_euler


class Localization:
    """
    Localization node that subscribes to turtlesim pose and publishes:
    1. ~/pose3d topic with PoseStamped message
    2. Transform between map and namespace+link frame
    """

    def __init__(self, node: Node):

        self.node = node
        self.logger = self.node.get_logger()

        # Get namespace and create frame names
        self.namespace = self.node.get_namespace().strip("/")
        if self.namespace:
            self.child_frame = f"{self.namespace}_link"
        else:
            self.child_frame = "turtle1_link"

        self.parent_frame = "map"

        # Create publishers
        self.pose_publisher = self.node.create_publisher(PoseStamped, "pose3d", 10)

        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(node)

        # Subscribe to turtlesim pose
        self.pose_subscription = self.node.create_subscription(
            Pose,
            "pose",  # Default turtlesim pose topic
            self.pose_callback,
            10,
        )

        self.logger.info(f"Localization node started")
        self.logger.info(f"Namespace: {self.namespace}")
        self.logger.info(f"Child frame: {self.child_frame}")
        self.logger.info(f"Publishing pose3d on: {self.node.get_name()}/pose3d")
        self.logger.info(
            f"Publishing transform: {self.parent_frame} -> {self.child_frame}"
        )

    def pose_callback(self, msg: Pose):
        """
        Callback for turtlesim pose messages.
        Publishes both PoseStamped and Transform messages.
        """
        current_time = self.node.get_clock().now()

        # Create and publish PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = current_time.to_msg()
        pose_stamped.header.frame_id = self.parent_frame

        # Set position (turtlesim uses 2D coordinates)
        pose_stamped.pose.position.x = msg.x
        pose_stamped.pose.position.y = msg.y
        pose_stamped.pose.position.z = 0.0

        # Convert theta to quaternion
        # Turtlesim theta is rotation around z-axis
        quat = quaternion_from_euler(0, 0, msg.theta)
        pose_stamped.pose.orientation = quat

        # Publish pose
        self.pose_publisher.publish(pose_stamped)

        # Create and publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame

        # Set translation
        transform.transform.translation.x = msg.x
        transform.transform.translation.y = msg.y
        transform.transform.translation.z = 0.0

        # Set rotation (same quaternion as pose)
        transform.transform.rotation = quat

        # Broadcast transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None, namespace=""):
    rclpy.init(args=args)
    node = rclpy.create_node("localization", namespace=namespace)
    localization_node = Localization(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        localization_node.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(namespace="turtle1")
