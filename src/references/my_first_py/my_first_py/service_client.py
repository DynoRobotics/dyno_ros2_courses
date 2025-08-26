import rclpy
import rclpy.executors

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from async_utils.async_primitives import (
    async_sleep,
    future_with_timeout,
    gather_with_timeout,
)

import example_interfaces.srv


class ServiceClient:
    def __init__(self, node: Node):
        self.node = node
        self.node._default_callback_group = ReentrantCallbackGroup()

        self.logger = self.node.get_logger()

        self.test_client = self.node.create_client(
            example_interfaces.srv.AddTwoInts, "test_service"
        )

        self.timer = self.node.create_timer(0.0, self.timer_callback)

    async def timer_callback(self):
        self.timer.cancel()

        await self.fast_slow_timeout()
        # await self.simultaneous_calls()

    async def fast_slow_timeout(self):
        request = example_interfaces.srv.AddTwoInts.Request()
        request.a = 38
        request.b = 4

        try:
            future = self.test_client.call_async(request)
            response = await future_with_timeout(self.node, future, 0.5)
            self.logger.info(
                f"test service call succeeded, the result is {response.sum}!"
            )
        except TimeoutError as e:
            self.logger.error(str(e))

        try:
            future = self.test_client.call_async(request)
            response = await future_with_timeout(self.node, future, 1.5)
            self.logger.info(
                f"test service call succeeded, the result is {response.sum}!"
            )
        except TimeoutError as e:
            self.logger.error(str(e))

    async def simultaneous_calls(self):
        request = example_interfaces.srv.AddTwoInts.Request()
        request.a = 38
        request.b = 4

        try:
            first_future = self.test_client.call_async(request)
            second_future = self.test_client.call_async(request)

            responses = await gather_with_timeout(
                self.node, [first_future, second_future], timeout_sec=1.5
            )

            for index, response in enumerate(responses):
                self.logger.info(f"Got response {index}: {response.sum}")

        except TimeoutError as e:
            self.logger.error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = Node("service_server")
    _ = ServiceClient(node)

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
