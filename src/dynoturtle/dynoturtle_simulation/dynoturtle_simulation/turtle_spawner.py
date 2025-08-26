#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import math


class TurtleSpawner:
    """
    Class that spawns multiple turtles at preset positions using turtlesim/srv/Spawn service.
    """

    def __init__(self, node: Node, default_num_turtles=3):
        self.node = node
        self.logger = self.node.get_logger()

        # Declare parameter for number of turtles
        self.node.declare_parameter("num_turtles", default_num_turtles)

        # Create service client for spawning turtles
        self.spawn_client = self.node.create_client(Spawn, "/spawn")

        # Wait for the spawn service to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Waiting for /spawn service...")

        self.logger.info("Turtle spawner initialized")

        # Spawn turtles after a short delay
        self.timer = self.node.create_timer(1.0, self.spawn_turtles_callback)
        self.spawned = False

    def get_preset_positions(self, num_turtles):
        """
        Generate preset positions for turtles based on number of turtles.
        Positions are arranged in a circle around the center (5.5, 5.5).
        """
        positions = []
        center_x, center_y = 5.5, 5.5
        radius = 2.0

        if num_turtles == 1:
            # Single turtle at center
            positions.append((center_x, center_y, 0.0))
        else:
            # Multiple turtles arranged in a circle
            for i in range(num_turtles):
                angle = 2 * math.pi * i / num_turtles
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
                theta = angle + math.pi / 2  # Face towards center
                positions.append((x, y, theta))

        return positions

    def spawn_turtles_callback(self):
        """
        Timer callback to spawn turtles once.
        """
        if self.spawned:
            return

        # Get number of turtles parameter (total turtles including turtle1)
        total_turtles = (
            self.node.get_parameter("num_turtles").get_parameter_value().integer_value
        )

        # Calculate how many additional turtles to spawn (turtle1 already exists)
        additional_turtles = max(0, total_turtles - 1)

        if additional_turtles == 0:
            self.logger.info(
                f"Total turtles requested: {total_turtles}. turtle1 already exists, no additional turtles to spawn."
            )
        else:
            self.logger.info(
                f"Total turtles requested: {total_turtles}. Spawning {additional_turtles} additional turtles..."
            )

        # Get preset positions for all turtles (including turtle1)
        all_positions = self.get_preset_positions(total_turtles)

        # Spawn additional turtles (skip first position which is for turtle1)
        for i in range(additional_turtles):
            position_index = i + 1  # Skip first position (for turtle1)
            x, y, theta = all_positions[position_index]
            turtle_name = f"turtle{i + 2}"  # Start from turtle2
            self.spawn_turtle(x, y, theta, turtle_name)

        self.spawned = True
        self.timer.cancel()  # Stop the timer after spawning

    def spawn_turtle(self, x, y, theta, name=""):
        """
        Spawn a single turtle at the specified position.
        """
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            lambda f: self.spawn_response_callback(f, name, x, y, theta)
        )

    def spawn_response_callback(self, future, name, x, y, theta):
        """
        Callback for spawn service response.
        """
        try:
            response = future.result()
            self.logger.info(
                f"Successfully spawned turtle '{response.name}' at position "
                f"({x:.2f}, {y:.2f}) with theta {theta:.2f}"
            )
        except Exception as e:
            self.logger.error(f"Failed to spawn turtle '{name}': {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("turtle_spawner")

    turtle_spawner = TurtleSpawner(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
