import os
import launch
from launch import LaunchDescription
from launch.actions import GroupAction, ExecuteProcess
from launch_ros.actions import PushRosNamespace, Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    num_turtles = 2

    launch_dir = os.path.join(
        get_package_share_directory("dynoturtle_bringup"), "launch"
    )

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

    web_bridge = Node(package="web_bridge", executable="web_bridge", name="web_bridge")

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

    turtles = []
    for i in range(0, num_turtles):
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
        web_bridge,
        rviz,
        py_trees_tree_viewer,
    ]
    launch_list.extend(turtles)

    return LaunchDescription(launch_list)


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
