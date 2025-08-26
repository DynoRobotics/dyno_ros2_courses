"""Safety node for DynoTurtle that relays safe velocity commands."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.time import Time


class Safety:
    """Safety class that subscribes to safe_cmd_vel and publishes to cmd_vel."""

    def __init__(self, node: Node):
        """Initialize the Safety class.

        Args:
            node: ROS2 node instance to use for communication
        """
        self.node = node

        # Declare parameters
        self.node.declare_parameter("bumper_timeout", 1.0)  # seconds
        self.bumper_timeout = (
            self.node.get_parameter("bumper_timeout").get_parameter_value().double_value
        )

        # Bumper state tracking
        self.front_bumper_triggered = False
        self.last_bumper_time = None

        # Create subscriber for safe velocity commands
        self.safe_cmd_vel_sub = self.node.create_subscription(
            Twist, "safe_cmd_vel", self.safe_cmd_vel_callback, 10
        )

        # Create subscriber for front bumper sensor
        self.front_bumper_sub = self.node.create_subscription(
            Bool, "sensors/front_bumper", self.front_bumper_callback, 10
        )

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)

        self.node.get_logger().info(
            f"Safety node initialized - relaying safe_cmd_vel to cmd_vel with bumper timeout: {self.bumper_timeout}s"
        )

    def front_bumper_callback(self, msg: Bool):
        """Callback for front bumper sensor.

        Args:
            msg: Bool message indicating if front bumper is triggered
        """
        self.front_bumper_triggered = msg.data
        self.last_bumper_time = self.node.get_clock().now()

    def is_bumper_data_recent(self) -> bool:
        """Check if bumper data is recent enough to be trusted.

        Returns:
            bool: True if bumper data is recent, False otherwise
        """
        if self.last_bumper_time is None:
            return False

        current_time = self.node.get_clock().now()
        time_diff = (current_time - self.last_bumper_time).nanoseconds / 1e9
        return time_diff <= self.bumper_timeout

    def safe_cmd_vel_callback(self, msg: Twist):
        """Callback for safe velocity commands.

        Args:
            msg: Twist message containing safe velocity commands
        """
        # Create a copy of the message to potentially modify
        safe_msg = Twist()
        safe_msg.linear.x = msg.linear.x
        safe_msg.linear.y = msg.linear.y
        safe_msg.linear.z = msg.linear.z
        safe_msg.angular.x = msg.angular.x
        safe_msg.angular.y = msg.angular.y
        safe_msg.angular.z = msg.angular.z

        # Check if we have recent bumper data
        if not self.is_bumper_data_recent():
            safe_msg.linear.x = 0.0
            safe_msg.linear.y = 0.0
            safe_msg.linear.z = 0.0
            safe_msg.angular.x = 0.0
            safe_msg.angular.y = 0.0
            safe_msg.angular.z = 0.0
        elif self.front_bumper_triggered and safe_msg.linear.x > 0.0:
            # Block forward movement if front bumper is triggered
            safe_msg.linear.x = 0.0

        # Publish the safe command
        self.cmd_vel_pub.publish(safe_msg)

        self.node.get_logger().debug(
            f"Published safe command - linear: [{safe_msg.linear.x:.2f}, {safe_msg.linear.y:.2f}, {safe_msg.linear.z:.2f}], "
            f"angular: [{safe_msg.angular.x:.2f}, {safe_msg.angular.y:.2f}, {safe_msg.angular.z:.2f}]"
        )


def main(args=None, namespace=""):
    """Main function to run the safety node."""
    rclpy.init(args=args)

    node = Node("safety_node", namespace=namespace)
    safety = Safety(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
