#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from dynoturtle_simulation.battery_simulator import BatterySimulator
from dynoturtle_simulation.bumper_simulator import BumperSimulator
from dynoturtle_simulation.simulator_state import (
    SimulationConfig,
    SimulatorStateManager,
)


def main(args=None):
    """Main function to run the refactored simulator with shared state management."""
    rclpy.init(args=args)

    # Create the main node
    node = Node("simulator_main")
    logger = node.get_logger()

    try:
        # Declare and get parameters
        node.declare_parameter("num_turtles", 2)

        # Create simulation configuration from parameters
        config = SimulationConfig(
            num_turtles=node.get_parameter("num_turtles")
            .get_parameter_value()
            .integer_value,
        )

        # Create the central state manager
        state_manager = SimulatorStateManager(node, config)

        # Create simulator components using the shared state manager
        bumper_simulator = BumperSimulator(node, state_manager)
        battery_simulator = BatterySimulator(node, state_manager)

        logger.info("Refactored simulator system initialized successfully")
        logger.info(
            f"Configuration: {config.num_turtles} turtles, "
            f"bumper rate: {config.bumper_update_rate}Hz, "
            f"battery rate: {config.battery_update_rate}Hz"
        )

        # Spin the node
        rclpy.spin(node)

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, shutting down...")
    except Exception as e:
        logger.error(f"Error in simulator main: {e}")
    finally:
        # Clean shutdown
        try:
            node.destroy_node()
        except:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
