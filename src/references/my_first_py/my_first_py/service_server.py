import rclpy
import rclpy.executors

from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from async_utils.async_primitives import async_sleep, future_with_timeout
from rclpy.qos import qos_profile_services_default
import example_interfaces.srv


class ServiceServer:
    def __init__(self, node: Node):
        self.node = node
        self.node._default_callback_group = ReentrantCallbackGroup()

        self.logger = self.node.get_logger()

        self.test_service = self.node.create_service(
            example_interfaces.srv.AddTwoInts,
            "test_service",
            self.test_service_callback,
        )

    async def test_service_callback(self, request, response):
        response.sum = request.a + request.b
        self.logger.info(f"Adding {request.a} to {request.b} results in {response.sum}")
        await async_sleep(self.node, 1.0)
        self.logger.info("Finished waiting")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Node("service_client")
    _ = ServiceServer(node)

    try:
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
