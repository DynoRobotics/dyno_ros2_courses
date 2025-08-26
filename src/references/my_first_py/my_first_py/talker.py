import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Talker:
    def __init__(self, node: Node):
        self.node = node
        self.counter = 0

        self.publisher = self.node.create_publisher(String, "talker", 10)

        self.timer = self.node.create_timer(1.0, self.timer_callback)

        self.node.get_logger().info("Talker initialized")

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello! Count: {self.counter}"

        self.publisher.publish(msg)
        self.node.get_logger().info(f'Published: "{msg.data}"')

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    node = Node("talker")
    _ = Talker(node)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
