import os
import launch
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    move_to_action = Node(
        package="dynoturtle_navigation",
        executable="move_to_action",
        name="move_to_action",
    )

    behavior_tree = Node(
        package="dynoturtle_behaviors", executable="simple_tree", name="simple_tree"
    )

    return LaunchDescription(
        [
            safety,
            localization,
            navigation,
            move_action,
            rotate_action,
            move_to_action,
            behavior_tree,
        ]
    )


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
