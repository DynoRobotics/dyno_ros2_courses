import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Listener:
    def __init__(self, node: Node):
        self.node = node

        self.subscription = self.node.create_subscription(
            String, "talker", self.listener_callback, 10
        )

        self.node.get_logger().info("Listener initialized")

    def listener_callback(self, msg):
        self.node.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    node = Node("listener")
    _ = Listener(node)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
