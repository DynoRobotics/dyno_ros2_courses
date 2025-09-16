import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.actions import OpaqueFunction, DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    bringup_dir = get_package_share_directory("dynobot_bringup")
    description_dir = os.path.join(
        get_package_share_directory("dynobot_description"), "urdf"
    )

    joy2twist_params = os.path.join(bringup_dir, "params", "joy2twist.yaml")

    # Get URDF via xacro with the current namespace
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_dir, "dynobot.xacro"]),
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content)}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    localization = Node(
        package="dynobot_localization",
        executable="localization",
        name="localization",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    joy2twist = Node(
        package="joy2twist",
        executable="joy2twist",
        parameters=[joy2twist_params],
        output={"screen"},
        emulate_tty="true",
        condition=IfCondition(use_sim_time),
    )

    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        output={"screen"},
        emulate_tty="true",
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(localization)
    ld.add_action(joy2twist)
    ld.add_action(joy_linux_node)

    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
