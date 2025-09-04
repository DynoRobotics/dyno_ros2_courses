import os
import launch
from launch import LaunchDescription
from launch.actions import GroupAction, ExecuteProcess
from launch_ros.actions import PushRosNamespace, Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    """
    Function to set up launch actions that depend on launch arguments.
    This is called at runtime when launch arguments are available.
    """
    # Get the actual value of num_turtles from the launch context
    num_turtles_value = int(context.launch_configurations["num_turtles"])

    launch_dir = os.path.join(
        get_package_share_directory("dynoturtle_bringup"), "launch"
    )

    # Get the launch configuration for passing to nodes
    num_turtles = LaunchConfiguration("num_turtles")

    turtlesim = Node(
        package="turtlesim", executable="turtlesim_node", name="turtlesim_node"
    )

    turtle_spawner = Node(
        package="dynoturtle_simulation",
        executable="turtle_spawner",
        name="turtle_spawner",
        parameters=[{"num_turtles": num_turtles}],
    )

    simulator_extensions = Node(
        package="dynoturtle_simulation",
        executable="simulator_extensions",
        name="simulator_extensions",
        parameters=[{"num_turtles": num_turtles}],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dynoturtle_bringup"), "rviz", "default.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    py_trees_tree_viewer = ExecuteProcess(
        cmd=["py-trees-tree-viewer"],
    )

    # Create turtle instances dynamically based on num_turtles argument
    turtles = []
    for i in range(0, num_turtles_value):
        turtle_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "individual_turtle.launch.py")
            )
        )

        turtles.append(
            GroupAction(actions=[PushRosNamespace(f"turtle{i+1}"), turtle_launch])
        )

    launch_list = [
        turtlesim,
        simulator_extensions,
        turtle_spawner,
        rviz,
        py_trees_tree_viewer,
    ]
    launch_list.extend(turtles)

    return launch_list


def generate_launch_description():
    # Declare launch argument for number of turtles
    num_turtles_arg = DeclareLaunchArgument(
        "num_turtles",
        default_value="1",
        description="Number of turtles to spawn in the simulation",
    )

    # Use OpaqueFunction to handle launch configuration evaluation at runtime
    opaque_function = OpaqueFunction(function=launch_setup)

    return LaunchDescription(
        [
            num_turtles_arg,
            opaque_function,
        ]
    )


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
