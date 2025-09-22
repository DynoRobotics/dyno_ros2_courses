import os
import os.path

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    pkg_dynobot_gazebo_worlds = get_package_share_directory("dynobot_gazebo_worlds")

    pkg_dynobot_bringup = get_package_share_directory("dynobot_bringup")
    pkg_gazebo_ros = get_package_share_directory("ros_gz_sim")

    bridge_params = os.path.join(pkg_dynobot_bringup, "params", "gz_bridge.yaml")

    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    world_name = LaunchConfiguration(
        "world_name",
        default=["office_gz.world"],  ## OFFICE SCAN
        # default=["basic.world"],  ## EMPTY WORLD
    )

    gazebo_gui = LaunchConfiguration("headless", default=1)

    # # Set the path to the world file
    world_file = PathJoinSubstitution(
        [pkg_dynobot_gazebo_worlds, "worlds", world_name]  ## OFFICE SCAN
        # [
        #     "/opt/dependencies_ws/src/dyno_gazebo_worlds",
        #     "worlds",
        #     world_name,
        # ]  ## EMPTY WORLD
    )
    world_launch_configuration = SetLaunchConfiguration(name="world", value=world_file)

    # Set the path to the SDF model files.
    gazebo_models_path = os.path.join(pkg_dynobot_gazebo_worlds, "models")
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gui": gazebo_gui,
            "gdb": "false",
            "verbose": "false",
            "gz_args": [
                "-r -v2 ",
                world_file,
            ],  # v2: >= Info, v3 >= Debug, v4 >= Everything
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "robot",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.1",
            "-Y",
            "0.0",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
    )

    ld = LaunchDescription()
    ld.add_action(world_launch_configuration)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(ros_gz_bridge)

    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
