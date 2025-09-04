import os
import launch
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    navigation = Node(
        package="dynoturtle_navigation",
        executable="move_to_action",
        name="move_to_action",
    )

    goal_pose_client = Node(
        package="dynoturtle_navigation",
        executable="goal_pose_client",
        name="goal_pose_clinet",
    )

    return LaunchDescription([navigation, goal_pose_client])


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
