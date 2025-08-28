import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import OpaqueFunction

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue


def launch_robot_state_publisher(context, *args, **kwargs):
    """Function to launch robot_state_publisher with namespace-aware URDF"""

    # Get the current namespace from the launch context
    # This will capture the namespace pushed by PushRosNamespace
    namespace = context.launch_configurations.get("ros_namespace", "").strip("/")

    description_dir = os.path.join(
        get_package_share_directory("dynoturtle_description"), "urdf"
    )

    # Get URDF via xacro with the current namespace
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_dir, "dynoturtle.xacro"]),
            " ",
            f"turtle_namespace:={namespace}",
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content)}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
    )

    return [robot_state_publisher]


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("dynoturtle_bringup"), "launch"
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "localization.launch.py")
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "navigation.launch.py"))
    )

    safety = Node(package="dynoturtle_safety", executable="safety", name="safety")

    move_action = Node(
        package="dynoturtle_behaviors", executable="move_action", name="move_action"
    )

    rotate_action = Node(
        package="dynoturtle_behaviors", executable="rotate_action", name="rotate_action"
    )

    charge_action = Node(
        package="dynoturtle_behaviors", executable="charge_action", name="charge_action"
    )

    behavior_tree = Node(
        package="dynoturtle_behaviors", executable="simple_tree", name="main_tree"
    )

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_robot_state_publisher),
            safety,
            localization,
            navigation,
            move_action,
            rotate_action,
            charge_action,
            behavior_tree,
        ]
    )


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
