"""This module provides some async primitives for working with rclpy."""

import rclpy
import rclpy.node
import rclpy.task
import rclpy.time

from rclpy.action import ActionClient


def duration_to_sec(duration: rclpy.time.Duration):
    return duration.nanoseconds / 1e9


def duration_from_sec(sec: float):
    seconds = int(sec)
    nanoseconds = int((sec - seconds) * 1e9)

    return rclpy.time.Duration(seconds=seconds, nanoseconds=nanoseconds)


async def wait_for_message_with_timeout(
    node: rclpy.node.Node, msg_type, topic: str, timeout_sec: float
):
    first_done_future = rclpy.task.Future()

    def msg_callback(msg):
        first_done_future.set_result(True)

    def timer_callback():
        first_done_future.set_result(False)

    timer = node.create_timer(timeout_sec, timer_callback)
    _ = node.create_subscription(msg_type, topic, msg_callback, 10)

    await first_done_future

    if not first_done_future.result():
        timer.cancel()
        timer.destroy()
        raise TimeoutError(f"Wait for message timed out after {timeout_sec} seconds")

    timer.cancel()
    timer.destroy()


async def future_with_timeout(
    node: rclpy.node.Node, future: rclpy.task.Future, timeout_sec: float
):
    """
    Wait for a future to complete, with a timeout.

    Args
    ----
        node: An rclpy node used to trigger the timeout.
        future: The future to wait for.
        timeout_sec: The maximum time to wait for the future to complete.

    Returns
    -------
        The result of the future.

    Raises
    ------
        TimeoutError: If the future does not complete within the timeout.

    """
    first_done_future = rclpy.task.Future()

    def done_callback(arg=None):
        first_done_future.set_result(None)

    timer = node.create_timer(timeout_sec, done_callback)
    future.add_done_callback(done_callback)

    await first_done_future

    if not future.done():
        timer.cancel()
        timer.destroy()
        raise TimeoutError(f"Future timed out after {timeout_sec} seconds")

    timer.cancel()
    timer.destroy()

    return future.result()


async def wait_for_action_server(action_client: ActionClient, timeout_sec):
    """
    Wait for an action server to become ready, with a timeout.

    Args
    ----
        action_client: The action client to wait for.
        timeout_sec: The maximum time to wait for the action server to become ready.

    Raises
    ------
        TimeoutError: If the action server does not become ready within the timeout.

    """
    server_is_ready_future = rclpy.task.Future()

    start_time = action_client._node.get_clock().now()

    def check_server_ready():
        now = action_client._node.get_clock().now()
        if action_client.server_is_ready():
            server_is_ready_future.set_result(None)
        elif now - start_time > duration_from_sec(timeout_sec):
            server_is_ready_future.set_result(None)

    timer = action_client._node.create_timer(0.1, check_server_ready)
    await server_is_ready_future
    timer.cancel()
    duration_since_started = action_client._node.get_clock().now() - start_time
    if duration_to_sec(duration_since_started) > timeout_sec:
        raise TimeoutError(
            f"Action server did not become ready after {timeout_sec} seconds"
        )


async def gather_with_timeout(node, futures, timeout_sec):
    """
    Wait for a list of futures to all complete concurrently, with a single timeout.

    Uses a single timer racing against all futures completing.
    """
    if not futures:
        return []

    # Create coordination future
    gather_future = rclpy.task.Future()
    timeout_occurred = False

    # Track results
    results = [None] * len(futures)
    completed_count = 0

    def future_done_callback(index, future):
        nonlocal completed_count
        if not timeout_occurred:
            results[index] = future.result()
            completed_count += 1
            if completed_count == len(futures):
                gather_future.set_result(results)

    def timeout_callback():
        nonlocal timeout_occurred
        timeout_occurred = True
        gather_future.set_exception(
            TimeoutError(f"gather_with_timeout timed out after {timeout_sec} seconds")
        )

    # Set up single timeout timer
    timer = node.create_timer(timeout_sec, timeout_callback)

    # Attach callbacks to all futures
    for i, future in enumerate(futures):
        future.add_done_callback(lambda fut, idx=i: future_done_callback(idx, fut))

    try:
        result = await gather_future
        return result
    finally:
        timer.cancel()
        timer.destroy()


async def async_sleep(node: rclpy.node.Node, seconds: float):
    """Sleep for a given number of seconds, using an rclpy async mechanism."""
    future = rclpy.task.Future()

    def sleep_callback():
        future.set_result(None)

    timer = node.create_timer(seconds, sleep_callback)
    await future

    timer.cancel()
    timer.destroy()

    return None
