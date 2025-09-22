import os, time
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

from launch.conditions import IfCondition, UnlessCondition


def kill_gazebo():
    # Kill lingering Gazebo processes from previous runs
    for name in ["gzserver", "gzclient"]:
        while pid := os.popen(f"pidof {name}").read().strip():
            print(f"shutting down {name}...")
            os.system(f"kill -9 {pid}")
            time.sleep(1)
        print(f"{name} is shutdown!")


def kill_daemon():
    pid = (
        os.popen("ps aux | grep -i ros2-daemon | grep -v grep | awk '{print $2}'")
        .read()
        .strip()
    )
    if pid:
        print(f"killing daemon with pid: {pid}")
        os.system(f"kill -9 {pid}")
        time.sleep(1)


def kill_ros_processes():
    pids = (
        os.popen("ps aux | grep -i ros-args | grep -v grep | awk '{print $2}'")
        .read()
        .strip()
    )
    pids = [pid.strip() for pid in pids.split("\n") if pid.strip() != ""]
    for pid in pids:
        print(f"shutting down ros process with pid: {pid}")
        os.system(f"kill -9 {pid}")
        time.sleep(1)
    if pids:
        time.sleep(3)


def launch_setup(context, *args, **kwargs):
    """
    Function to set up launch actions that depend on launch arguments.
    This is called at runtime when launch arguments are available.
    """
    kill_gazebo()
    kill_daemon()
    kill_ros_processes()

    bringup_dir = get_package_share_directory("dynobot_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    rviz_dir = os.path.join(bringup_dir, "rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    rviz_config_file = PathJoinSubstitution([rviz_dir, "default.rviz"])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    dynobot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "robot.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "gazebo.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_sim_time")),
    )

    lidar_hw_stub = Node(
        package="dynobot_sensors",
        executable="lidar_hw_stub",
        name="lidar_hw_stub",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
    )

    launch_list = [use_sim_time_arg, rviz, dynobot, gazebo, lidar_hw_stub]

    return launch_list


def generate_launch_description():

    # Use OpaqueFunction to handle launch configuration evaluation at runtime
    opaque_function = OpaqueFunction(function=launch_setup)

    return LaunchDescription(
        [
            opaque_function,
        ]
    )


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
